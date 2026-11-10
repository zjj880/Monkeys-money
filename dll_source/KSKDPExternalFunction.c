/* OrcaFlex 11.4 x64 -> KSKDP TCP controller. Build with the matching OrcFxAPI.h. */
#define WIN32_LEAN_AND_MEAN
#define UNICODE
#define _UNICODE
#define _IndexedVectorsAndMatrices_
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <wchar.h>
#include <math.h>
#include "OrcFxAPI.h"

#define EXPORT __declspec(dllexport)
#define MAX_THRUSTERS 64
#define PACKET_SIZE 8192
#define PI 3.14159265358979323846
typedef INT_PTR (__stdcall *GetCallerProc)(TOrcFxAPIHandle,int*);
typedef void (__stdcall *SetCallerProc)(TOrcFxAPIHandle,INT_PTR,int*);
typedef int (__stdcall *GetTagProc)(TOrcFxAPIHandle,LPCWSTR,LPWSTR,int*);
typedef void (__stdcall *UnitsProc)(TOrcFxAPIHandle,LPCWSTR,double*,int*);
typedef void (__stdcall *ErrorProc)(TExtFnInfoW*,LPCWSTR,int*);
typedef void (__stdcall *RegisterProc)(TExtFnInfoW*,TExtFnResultInfoW*,int*);
typedef void (__stdcall *DataProc)(TOrcFxAPIHandle,LPCWSTR,int,double*,int*);
static GetCallerProc getCaller;
static SetCallerProc setCaller;
static GetTagProc getTag;
static UnitsProc getUnits;
static ErrorProc recordError;
static RegisterProc registerResult;
static DataProc getData;
static SRWLOCK creationLock = SRWLOCK_INIT;

typedef struct {
    SOCKET socket;
    int wsaStarted, refs, n, failed, hasResult, timeoutMs;
    double lastTime, lengthFactor, forceFactor, momentFactor, timeFactor;
    double force[6], thrust[MAX_THRUSTERS], angle[MAX_THRUSTERS];
    double orientation[3][3];
    char error[512];
    CRITICAL_SECTION lock;
} Session;

EXPORT void __stdcall InitializeOrcFxAPI(HMODULE module) {
    getCaller=(GetCallerProc)GetProcAddress(module,"C_GetCallerLong");
    setCaller=(SetCallerProc)GetProcAddress(module,"C_SetCallerLong");
    getTag=(GetTagProc)GetProcAddress(module,"C_GetTagW");
    getUnits=(UnitsProc)GetProcAddress(module,"C_GetUnitsConversionFactorW");
    recordError=(ErrorProc)GetProcAddress(module,"C_RecordExternalFunctionErrorW");
    registerResult=(RegisterProc)GetProcAddress(module,"C_RegisterExternalFunctionResultW");
    getData=(DataProc)GetProcAddress(module,"C_GetDataDoubleW");
}
EXPORT void __stdcall RegisterCapabilities(LPDWORD flags) {
    if(flags) *flags |= efcUnicode | efcImplicitIntegration;
}
static void report(TExtFnInfoW *info,const char *message) {
    wchar_t wide[600]; int status;
    MultiByteToWideChar(CP_UTF8,0,message,-1,wide,600);
    wide[599]=0;
    if(recordError) recordError(info,wide,&status);
}
static int fail(Session *s,const char *message) {
    s->failed=1; snprintf(s->error,sizeof(s->error),"KSKDP: %s",message);
    if(s->socket!=INVALID_SOCKET) { closesocket(s->socket); s->socket=INVALID_SOCKET; }
    return 0;
}
static void destroy(Session *s) {
    if(!s) return;
    if(s->socket!=INVALID_SOCKET) closesocket(s->socket);
    if(s->wsaStarted) WSACleanup();
    DeleteCriticalSection(&s->lock); free(s);
}
static int tag(TOrcFxAPIHandle object,const wchar_t *name,wchar_t *out,size_t cap) {
    int status, length=getTag(object,name,NULL,&status);
    if(status==stTagNotFound) return 0;
    if(status!=stOK || length<1 || (size_t)length>cap) return -1;
    getTag(object,name,out,&status);
    return status==stOK ? 1 : -1;
}
static int number(TOrcFxAPIHandle object,const wchar_t *name,double fallback,double *value) {
    wchar_t text[128], *end; int present=tag(object,name,text,128);
    if(!present) { *value=fallback; return 1; }
    if(present<0) return 0;
    *value=wcstod(text,&end);
    while(*end==L' ' || *end==L'\t') ++end;
    return end!=text && !*end && isfinite(*value);
}
static int vectorTag(TOrcFxAPIHandle object,const wchar_t *name,double *values,int n) {
    wchar_t text[4096], *p=text, *end; int present=tag(object,name,text,4096);
    if(!present) return 1;
    if(present<0) return 0;
    for(int i=0;i<n;++i) {
        values[i]=wcstod(p,&end);
        if(end==p || !isfinite(values[i])) return 0;
        p=end; while(*p==L' ' || *p==L'\t') ++p;
        if(i<n-1) { if(*p++!=L',') return 0; }
    }
    return !*p;
}
static int append(char *buffer,size_t *used,const char *format,...) {
    va_list args; va_start(args,format);
    int count=vsnprintf(buffer+*used,PACKET_SIZE-*used,format,args); va_end(args);
    if(count<0 || (size_t)count>=PACKET_SIZE-*used) return 0;
    *used+=(size_t)count; return 1;
}
static SOCKET connectOnce(const char *ip,int port,int timeout) {
    SOCKET sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(sock==INVALID_SOCKET) return sock;
    struct sockaddr_in address={0}; address.sin_family=AF_INET;
    address.sin_port=htons((u_short)port);
    if(inet_pton(AF_INET,ip,&address.sin_addr)!=1) { closesocket(sock); return INVALID_SOCKET; }
    u_long mode=1; ioctlsocket(sock,FIONBIO,&mode);
    int result=connect(sock,(struct sockaddr*)&address,sizeof(address));
    if(result==SOCKET_ERROR && WSAGetLastError()!=WSAEWOULDBLOCK) {
        closesocket(sock); return INVALID_SOCKET;
    }
    fd_set writeSet,errorSet; FD_ZERO(&writeSet); FD_ZERO(&errorSet);
    FD_SET(sock,&writeSet); FD_SET(sock,&errorSet);
    struct timeval wait={timeout/1000,(timeout%1000)*1000};
    int error=0, size=sizeof(error);
    if(select(0,NULL,&writeSet,&errorSet,&wait)<=0 ||
       getsockopt(sock,SOL_SOCKET,SO_ERROR,(char*)&error,&size)!=0 || error) {
        closesocket(sock); return INVALID_SOCKET;
    }
    mode=0; ioctlsocket(sock,FIONBIO,&mode);
    int yes=1; setsockopt(sock,IPPROTO_TCP,TCP_NODELAY,(char*)&yes,sizeof(yes));
    return sock;
}
static int sendAll(Session *s,const char *message) {
    int sent=0, length=(int)strlen(message);
    DWORD timeout=(DWORD)s->timeoutMs;
    setsockopt(s->socket,SOL_SOCKET,SO_SNDTIMEO,(char*)&timeout,sizeof(timeout));
    while(sent<length) {
        int count=send(s->socket,message+sent,length-sent,0);
        if(count<=0) return fail(s,"TCP send failed. Check that main.py is still running.");
        sent+=count;
    }
    return 1;
}
static int receiveFrame(Session *s,char *buffer) {
    size_t used=0; ULONGLONG deadline=GetTickCount64()+s->timeoutMs;
    while(used<PACKET_SIZE-1) {
        ULONGLONG now=GetTickCount64();
        if(now>=deadline) return fail(s,"TCP response timed out. Check the Python console.");
        DWORD timeout=(DWORD)(deadline-now);
        setsockopt(s->socket,SOL_SOCKET,SO_RCVTIMEO,(char*)&timeout,sizeof(timeout));
        int count=recv(s->socket,buffer+used,(int)(PACKET_SIZE-1-used),0);
        if(count<=0) return fail(s,"TCP response missing (disconnect or timeout). Check the Python console.");
        used+=count; buffer[used]=0;
        char *end=strchr(buffer,'*');
        if(end) {
            for(char *p=end+1;*p;++p) if(*p!='\r' && *p!='\n' && *p!=' ')
                return fail(s,"Unexpected extra data after the TCP frame.");
            end[1]=0; return 1;
        }
    }
    return fail(s,"TCP frame exceeds the interface buffer size.");
}
static int split(char *buffer,char **tokens,int cap) {
    int count=0; char *p=buffer;
    for(;;) {
        if(count==cap) return -1;
        tokens[count++]=p; char *comma=strchr(p,',');
        if(!comma) return count;
        *comma=0; p=comma+1;
    }
}
static int finiteNumber(const char *text,double *value) {
    char *end; *value=strtod(text,&end);
    return end!=text && !*end && isfinite(*value);
}
static int parseForce(Session *s,char *frame) {
    char *tokens[2*MAX_THRUSTERS+9];
    int count=split(frame,tokens,2*MAX_THRUSTERS+9);
    if(count!=2*s->n+9 || strcmp(tokens[0],"$FORCE") ||
       strcmp(tokens[6],"THRUST") || strcmp(tokens[7+s->n],"ANGLE") ||
       strcmp(tokens[8+2*s->n],"*"))
        return fail(s,"Invalid $FORCE frame or thruster-count mismatch.");
    double values[5+2*MAX_THRUSTERS]; int k=0;
    for(int i=1;i<=5;++i) if(!finiteNumber(tokens[i],&values[k++])) return fail(s,"Non-finite force in TCP response.");
    for(int i=0;i<s->n;++i) if(!finiteNumber(tokens[7+i],&values[k++])) return fail(s,"Invalid thrust in TCP response.");
    for(int i=0;i<s->n;++i) if(!finiteNumber(tokens[8+s->n+i],&values[k++])) return fail(s,"Invalid angle in TCP response.");
    /* Python order: Fx,Fy,Mz,Mx,My. OrcaFlex order: Fx,Fy,Fz,Mx,My,Mz. */
    s->force[0]=values[0]*s->forceFactor; s->force[1]=values[1]*s->forceFactor; s->force[2]=0;
    s->force[3]=values[3]*s->momentFactor; s->force[4]=values[4]*s->momentFactor; s->force[5]=values[2]*s->momentFactor;
    memcpy(s->thrust,values+5,s->n*sizeof(double));
    memcpy(s->angle,values+5+s->n,s->n*sizeof(double)); return 1;
}
static double initialData(TOrcFxAPIHandle object,const wchar_t *name) {
    int status; double value=0; getData(object,name,0,&value,&status);
    return status==stOK && isfinite(value) ? value : 0;
}
static Session *create(TExtFnInfoW *info) {
    Session *s=(Session*)calloc(1,sizeof(Session));
    if(!s) { report(info,"KSKDP: Memory allocation failed."); return NULL; }
    s->socket=INVALID_SOCKET; s->timeoutMs=30000; InitializeCriticalSection(&s->lock);
    wchar_t host[128]; char ip[128]="127.0.0.1";
    double port=23011,n=4,target[3],freedom[3]={2,2,2},draught=0,gravity=0;
    double usage[MAX_THRUSTERS]; int status;
    const wchar_t *units[4]={L"LL",L"FF",L"FF.LL",L"TT"};
    double factors[4];
    for(int i=0;i<4;++i) {
        getUnits(info->ObjectHandle,units[i],&factors[i],&status);
        if(status!=stOK || !isfinite(factors[i]) || factors[i]<=0) { fail(s,"Cannot read model units."); goto bad; }
    }
    s->lengthFactor=factors[0]; s->forceFactor=1000*factors[1];
    s->momentFactor=1000*factors[2]; s->timeFactor=factors[3];
    target[0]=initialData(info->ObjectHandle,L"InitialX")/s->lengthFactor;
    target[1]=initialData(info->ObjectHandle,L"InitialY")/s->lengthFactor;
    target[2]=initialData(info->ObjectHandle,L"InitialHeading");
    int present=tag(info->ObjectHandle,L"KSKDP_IP",host,128);
    if(present<0 || (present && !WideCharToMultiByte(CP_UTF8,0,host,-1,ip,128,NULL,NULL))) { fail(s,"Invalid KSKDP_IP tag."); goto bad; }
    if(!number(info->ObjectHandle,L"KSKDP_Port",port,&port) || port<1 || port>65535 || floor(port)!=port ||
       !number(info->ObjectHandle,L"KSKDP_ThrusterCount",n,&n) || n<1 || n>MAX_THRUSTERS || floor(n)!=n ||
       !number(info->ObjectHandle,L"TargetX",target[0],&target[0]) ||
       !number(info->ObjectHandle,L"TargetY",target[1],&target[1]) ||
       !number(info->ObjectHandle,L"TargetHeading",target[2],&target[2]) ||
       !number(info->ObjectHandle,L"Draught",0,&draught) ||
       !number(info->ObjectHandle,L"GravityHeight",0,&gravity)) { fail(s,"Invalid numeric vessel tag."); goto bad; }
    s->n=(int)n;
    for(int i=0;i<s->n;++i) usage[i]=1;
    if(s->n==4) { s->angle[2]=1.5708; s->angle[3]=1.5708; }
    if(!vectorTag(info->ObjectHandle,L"Freedom",freedom,3) ||
       !vectorTag(info->ObjectHandle,L"ThrustLimit",usage,s->n) ||
       !vectorTag(info->ObjectHandle,L"KSKDP_InitialAngles",s->angle,s->n)) { fail(s,"Invalid vector vessel tag (use comma-separated numbers)."); goto bad; }
    for(int i=0;i<3;++i) if(freedom[i]<0 || freedom[i]>3 || floor(freedom[i])!=freedom[i]) { fail(s,"Freedom must contain three integers in 0..3."); goto bad; }
    for(int i=0;i<s->n;++i) if(usage[i]<0 || usage[i]>1) { fail(s,"ThrustLimit values must be fractions in 0..1."); goto bad; }
    if(info->LengthOfStateData>0) { fail(s,"Resuming controller state is unsupported. Reset and start a new simulation."); goto bad; }
    WSADATA wsa; if(WSAStartup(MAKEWORD(2,2),&wsa)!=0) { fail(s,"WSAStartup failed."); goto bad; } s->wsaStarted=1;
    s->socket=connectOnce(ip,(int)port,3000);
    if(s->socket==INVALID_SOCKET) { fail(s,"Cannot connect to TCP Manager. Start KSKDP/run_kskdp.cmd and check KSKDP_IP / KSKDP_Port."); goto bad; }
    char packet[PACKET_SIZE]; size_t used=0;
    int ok=append(packet,&used,"$PARA,%d,TARGET,%.17g,%.17g,%.17g,FREEDOM,%d,%d,%d,DRAUGHT,%.17g,GRAVITY,%.17g,THRUSTLIMIT",
        s->n,target[0],target[1],target[2]*PI/180,(int)freedom[0],(int)freedom[1],(int)freedom[2],draught,gravity);
    for(int i=0;i<s->n;++i) ok=ok && append(packet,&used,",%.17g",usage[i]);
    ok=ok && append(packet,&used,",*");
    if(!ok) { fail(s,"Parameter frame too long."); goto bad; }
    if(!sendAll(s,packet) || !receiveFrame(s,packet)) goto bad;
    char *tokens[4]; int count=split(packet,tokens,4); double workerPort;
    if(count!=3 || strcmp(tokens[0],"$PORT") || strcmp(tokens[2],"*") ||
       !finiteNumber(tokens[1],&workerPort) || workerPort<1 || workerPort>65535 || floor(workerPort)!=workerPort) { fail(s,"Manager did not return a valid $PORT frame."); goto bad; }
    closesocket(s->socket); s->socket=INVALID_SOCKET;
    ULONGLONG deadline=GetTickCount64()+10000;
    do { s->socket=connectOnce(ip,(int)workerPort,250); if(s->socket!=INVALID_SOCKET) break; Sleep(25); } while(GetTickCount64()<deadline);
    if(s->socket==INVALID_SOCKET) { fail(s,"Cannot connect to assigned Worker. Check the Python console for initialization errors."); goto bad; }
    return s;
bad:
    report(info,s->error); destroy(s); return NULL;
}
static int slot(const wchar_t *name,int *global) {
    const wchar_t *names[12]={vdnLocalAppliedForceX,vdnLocalAppliedForceY,vdnLocalAppliedForceZ,
        vdnLocalAppliedMomentX,vdnLocalAppliedMomentY,vdnLocalAppliedMomentZ,
        vdnGlobalAppliedForceX,vdnGlobalAppliedForceY,vdnGlobalAppliedForceZ,
        vdnGlobalAppliedMomentX,vdnGlobalAppliedMomentY,vdnGlobalAppliedMomentZ};
    if(!name) return -1;
    for(int i=0;i<12;++i) {
        size_t length=wcslen(names[i]);
        if(!wcsncmp(name,names[i],length) && (name[length]==0 || name[length]==L'[')) { *global=i>=6; return i%6; }
    }
    return -1;
}
static double output(Session *s,int index,int global) {
    if(!global) return s->force[index];
    int axis=index%3,offset=index<3?0:3;
    double value=0;
    for(int i=0;i<3;++i) value+=s->force[offset+i]*s->orientation[i][axis];
    return value;
}
static int calculate(Session *s,TExtFnInfoW *info) {
    const TVesselInstantaneousCalculationData *v=(const TVesselInstantaneousCalculationData*)info->lpInstantaneousCalculationData;
    if(!v || v->Size<(int)sizeof(*v)) return fail(s,"Use this external function on vessel applied loads only.");
    memcpy(s->orientation,v->Orientation,sizeof(s->orientation));
    double time=info->SimulationTime/s->timeFactor;
    if(!isfinite(time)) return fail(s,"Invalid simulation time.");
    /* EKFV1 and PID2 use dt=1 second. Hold commands between controller samples. */
    if(s->hasResult && time<s->lastTime-1e-8) return fail(s,"Simulation time moved backwards. Use fixed time steps and reset the simulation.");
    if(s->hasResult && (!info->NewTimeStep || time<s->lastTime+1-1e-8)) return 1;
    if(s->hasResult && time>s->lastTime+1+1e-6) return fail(s,"Controller sample interval is not 1 second. Use a fixed simulation step that divides 1 second (for example 0.1 s).");
    double x=v->Position[0]/s->lengthFactor,y=v->Position[1]/s->lengthFactor;
    double heading=atan2(v->Orientation[0][1],v->Orientation[0][0]);
    if(!isfinite(x) || !isfinite(y) || !isfinite(heading)) return fail(s,"Invalid vessel pose.");
    char packet[PACKET_SIZE]; size_t used=0;
    int ok=append(packet,&used,"$POSE,%.17g,%.17g,%.17g,$THRUST",x,y,heading);
    for(int i=0;i<s->n;++i) ok=ok && append(packet,&used,",%.17g",s->thrust[i]);
    ok=ok && append(packet,&used,",$ANGLE");
    for(int i=0;i<s->n;++i) ok=ok && append(packet,&used,",%.17g",s->angle[i]);
    ok=ok && append(packet,&used,",*");
    if(!ok) return fail(s,"Pose frame too long.");
    if(!sendAll(s,packet) || !receiveFrame(s,packet) || !parseForce(s,packet)) return 0;
    s->lastTime=time; s->hasResult=1; return 1;
}
EXPORT void __stdcall CalcThrusterForce(TExtFnInfoW *info) {
    if(!info) return;
    if(!getCaller || !setCaller || !getTag || !getUnits || !recordError || !registerResult || !getData) {
        report(info,"KSKDP: OrcFxAPI was not initialized with the required functions."); return;
    }
    int global=0,index=slot(info->lpDataName,&global),status;
    if(index<0) { report(info,"KSKDP: Only vessel local/global applied force and moment components are supported."); return; }
    Session *s=(Session*)info->lpData;
    switch(info->Action) {
    case eaRegisterResults: {
        TExtFnResultInfoW result={sizeof(result),index+6*global,info->lpDataName,index<3?L"FF":L"FF.LL"};
        registerResult(info,&result,&status);
        if(status!=stOK) report(info,"KSKDP: Could not register load result.");
        break;
    }
    case eaInitialise:
        info->Value=0; info->UpdateDuringStatics=FALSE; info->CanResumeSimulation=FALSE;
        AcquireSRWLockExclusive(&creationLock);
        s=(Session*)getCaller(info->ObjectHandle,&status);
        if(status!=stOK) { report(info,"KSKDP: Cannot get shared controller session."); s=NULL; }
        else if(!s) {
            s=create(info);
            if(s) {
                setCaller(info->ObjectHandle,(INT_PTR)s,&status);
                if(status!=stOK) { report(info,"KSKDP: Cannot store shared controller session."); destroy(s); s=NULL; }
            }
        }
        if(s) { ++s->refs; info->lpData=s; }
        ReleaseSRWLockExclusive(&creationLock); break;
    case eaCalculate:
        if(!s) { info->Value=0; report(info,"KSKDP: Controller initialization failed."); break; }
        EnterCriticalSection(&s->lock);
        if(s->failed || !calculate(s,info)) { info->Value=0; report(info,s->error); }
        else info->Value=output(s,index,global);
        LeaveCriticalSection(&s->lock); break;
    case eaFinalise:
        if(!s) break;
        AcquireSRWLockExclusive(&creationLock);
        info->lpData=NULL;
        if(--s->refs==0) { setCaller(info->ObjectHandle,0,&status); destroy(s); }
        ReleaseSRWLockExclusive(&creationLock); break;
    case eaLogResultCreate:
        if(s) {
            double *log=(double*)malloc(12*sizeof(double));
            if(!log) { report(info,"KSKDP: Log allocation failed."); break; }
            EnterCriticalSection(&s->lock);
            for(int i=0;i<12;++i) log[i]=output(s,i%6,i>=6);
            LeaveCriticalSection(&s->lock);
            info->lpLogData=log; info->LengthOfLogData=12*sizeof(double);
        } break;
    case eaLogResultDestroy:
        free(info->lpLogData); info->lpLogData=NULL; info->LengthOfLogData=0; break;
    case eaDeriveResult:
        if(info->lpLogData && info->LengthOfLogData==12*sizeof(double) && info->ResultID>=0 && info->ResultID<12)
            info->Value=((double*)info->lpLogData)[info->ResultID];
        else report(info,"KSKDP: Invalid saved load-result data."); break;
    case eaStoreStateCreate:
        info->lpStateData=NULL; info->LengthOfStateData=0; break;
    case eaStoreStateDestroy: break;
    }
}
