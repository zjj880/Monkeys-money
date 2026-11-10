@echo off
setlocal
where cl >nul 2>nul
if errorlevel 1 (
  echo Open an x64 Native Tools Command Prompt for Visual Studio first.
  exit /b 2
)
if not exist "%~dp0sdk\OrcFxAPI.h" (
  echo Missing dll_source\sdk\OrcFxAPI.h from OrcaFlex 11.4.
  exit /b 2
)
cl /nologo /LD /O2 /std:c11 /I "%~dp0sdk" "%~dp0verified\KSKDPExternalFunction_verified.c" /link /OUT:"%~dp0..\KSKDPExternalFunction_verified.dll" Ws2_32.lib
exit /b %errorlevel%
