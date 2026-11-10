import numpy as np
import para_def as para
from scipy.linalg import block_diag
from scipy import sparse
from scipy.optimize import lsq_linear
import osqp


class QP2(object):
    """
    Robust QP thrust allocation.

    Fixes the common "allocation output suddenly freezes" problem by:
    1) keeping the dT/dAlpha local-QP formulation;
    2) changing the nonlinear backtracking from "must improve by 0.1% or hold old command"
       to "choose the best nonzero candidate and accept no-worse/small-worse steps";
    3) adding a bounded least-squares fallback when OSQP fails, instead of returning with
       the previous thrust/angle unchanged.

    External interface is kept the same:
        q = QP2()
        ok = q.QPLoop(tauLC, pose)
        tau, T, alpha = q.getState()
    """

    def __init__(self):
        # Static parameters
        self.thrusterXList = np.asarray(para.ThrusterXList, dtype=np.float64).reshape(-1, 1)
        self.thrusterYList = np.asarray(para.ThrusterYList, dtype=np.float64).reshape(-1, 1)
        self.thrusterZList = np.asarray(para.ThrusterZList, dtype=np.float64).reshape(-1, 1)

        self.thrusterMaxAlphaList = np.asarray(para.ThrusterMaxAlphaList, dtype=np.float64).reshape(-1, 1)
        self.thrusterMinAlphaList = np.asarray(para.ThrusterMinAlphaList, dtype=np.float64).reshape(-1, 1)
        self.thrusterMaxDAlphaList = np.asarray(para.ThrusterMaxDAlphaList, dtype=np.float64).reshape(-1, 1)

        self.thrusterMaxTList = np.asarray(para.ThrusterMaxTList, dtype=np.float64).reshape(-1, 1)
        self.thrusterMinTList = np.asarray(para.ThrusterMinTList, dtype=np.float64).reshape(-1, 1)
        self.thrusterMaxDTList = np.asarray(para.ThrusterMaxDTList, dtype=np.float64).reshape(-1, 1)

        # Dynamic states
        self.targetLCForce = np.zeros((3, 1), dtype=np.float64)
        self.thrusterAngle = np.asarray(para.ThrusterAngle, dtype=np.float64).reshape(-1, 1)
        self.thrustNum = int(self.thrusterXList.shape[0])
        self.thrusterForce = np.zeros((self.thrustNum, 1), dtype=np.float64)
        self.J = np.eye(3, dtype=np.float64)

        # Weights
        self.W_energy = float(getattr(para, "W_W2", 10.0))
        self.W_angle_rate = float(getattr(para, "W_Omega2", 10.0))
        self.W_slack_base = float(getattr(para, "W_Q2", 1.0e7))
        self.W_deltaT = float(getattr(para, "W_DeltaT2", 800.0))
        self.W_deltaAlpha = float(getattr(para, "W_DeltaAlpha2", self.W_angle_rate))

        # Slack and numerical scaling
        self.slack_bound = np.asarray(getattr(para, "QP_SlackBound", [1.0e3, 1.0e3, 1.0e5]),
                                      dtype=np.float64).reshape(3, 1)
        self.err_scale = np.asarray(getattr(para, "FLim", [700.0, 700.0, 30000.0]),
                                    dtype=np.float64).reshape(3, 1)
        self.err_scale = np.maximum(np.abs(self.err_scale), 1.0)

        # Backtracking parameters
        self.enable_backtracking = bool(getattr(para, "QP_Backtracking", True))
        self.backtracking_scales = self._sanitize_scales(
            getattr(para, "QP_BacktrackingScales", [1.0, 0.5, 0.25, 0.125])
        )
        # Accept a candidate as long as it is not meaningfully worse than the old state.
        # This is the key change: the old code required score < old_score * 0.999,
        # which can reject all normal small updates and freeze the output.
        self.backtracking_accept_tol = float(getattr(para, "QP_BacktrackingAcceptTol", 5.0e-3))
        # If all nonlinear candidates are slightly worse, still apply the smallest step
        # to avoid deadlock. Set this False if you prefer strict non-increasing error.
        self.force_small_step_when_stalled = bool(getattr(para, "QP_ForceSmallStepWhenStalled", True))

        # OSQP parameters
        self.osqp_max_iter = int(getattr(para, "QP_MaxIter", 16000))
        self.osqp_polish = bool(getattr(para, "QP_Polish", True))
        self.osqp_eps_abs = float(getattr(para, "QP_EpsAbs", 1e-3))
        self.osqp_eps_rel = float(getattr(para, "QP_EpsRel", 1e-3))

        self.debug = bool(getattr(para, "QP_Debug", False))
        self.last_info = {}

    @staticmethod
    def _sanitize_scales(scales):
        arr = []
        for s in scales:
            try:
                s = float(s)
            except Exception:
                continue
            if np.isfinite(s) and s > 0.0:
                arr.append(min(s, 1.0))
        arr.append(1.0)
        arr = sorted(set(arr), reverse=True)
        return tuple(arr)

    def SetPara(self, gravityHeight, thrustUsage):
        """Update thruster z and available thrust ratio without accumulating old values."""
        base_z = np.asarray(para.ThrusterZList, dtype=np.float64).reshape(-1, 1)
        self.thrusterZList = base_z - float(gravityHeight)

        usage = np.asarray(thrustUsage, dtype=np.float64).reshape(-1, 1)
        base_max_t = np.asarray(para.ThrusterMaxTList, dtype=np.float64).reshape(-1, 1)
        base_min_t = np.asarray(para.ThrusterMinTList, dtype=np.float64).reshape(-1, 1)
        self.thrusterMaxTList = base_max_t * usage
        self.thrusterMinTList = base_min_t * usage

    def _calc_B3_and_dBT(self, angle, force_for_derivative):
        """Return B(alpha) and d(BT)/dAlpha at current force."""
        n = self.thrustNum
        B = np.zeros((3, n), dtype=np.float64)
        dBT = np.zeros((3, n), dtype=np.float64)

        for i in range(n):
            ca = np.cos(angle[i, 0])
            sa = np.sin(angle[i, 0])
            x_i = self.thrusterXList[i, 0]
            y_i = self.thrusterYList[i, 0]
            T_i = force_for_derivative[i, 0]

            B[0, i] = ca
            B[1, i] = sa
            B[2, i] = x_i * sa - y_i * ca

            dBT[0, i] = -sa * T_i
            dBT[1, i] = ca * T_i
            dBT[2, i] = (x_i * ca + y_i * sa) * T_i

        return B, dBT

    def _calc_tau3(self, angle, force):
        B, _ = self._calc_B3_and_dBT(angle, force)
        return B @ force

    def _weighted_error_norm(self, tau, target):
        return float(np.linalg.norm((target - tau) / self.err_scale))

    def _calc_step_bounds(self, T0, alpha0):
        dT_min = np.maximum(self.thrusterMinTList - T0, -self.thrusterMaxDTList)
        dT_max = np.minimum(self.thrusterMaxTList - T0, self.thrusterMaxDTList)
        # Avoid numerical infeasible bound caused by roundoff.
        dT_min = np.minimum(dT_min, dT_max)

        d_alpha_min = np.maximum(self.thrusterMinAlphaList - alpha0, -self.thrusterMaxDAlphaList)
        d_alpha_max = np.minimum(self.thrusterMaxAlphaList - alpha0, self.thrusterMaxDAlphaList)
        d_alpha_min = np.minimum(d_alpha_min, d_alpha_max)
        return dT_min, dT_max, d_alpha_min, d_alpha_max

    def _apply_backtracking(self, T0, alpha0, dT, dAlpha, tau0):
        old_score = self._weighted_error_norm(tau0, self.targetLCForce)

        # Degenerate case: already exactly on target or QP step is zero.
        if np.linalg.norm(dT) + np.linalg.norm(dAlpha) < 1.0e-12:
            self.last_info.update({"step_mode": "zero_step", "old_score": old_score, "best_score": old_score})
            return T0, alpha0

        candidates = []
        for scale in self.backtracking_scales:
            T_cand = np.clip(T0 + scale * dT, self.thrusterMinTList, self.thrusterMaxTList)
            alpha_cand = np.clip(alpha0 + scale * dAlpha,
                                 self.thrusterMinAlphaList, self.thrusterMaxAlphaList)
            tau_cand = self._calc_tau3(alpha_cand, T_cand)
            score = self._weighted_error_norm(tau_cand, self.targetLCForce)
            candidates.append((score, scale, T_cand, alpha_cand))

        candidates.sort(key=lambda item: item[0])
        best_score, best_scale, best_T, best_alpha = candidates[0]

        if best_score <= old_score * (1.0 + self.backtracking_accept_tol):
            mode = "accepted_best_candidate"
            T_new, alpha_new = best_T, best_alpha
        elif self.force_small_step_when_stalled:
            # Critical anti-freeze logic: do not return T0/alpha0 after OSQP has produced
            # a valid direction. Use the smallest candidate as a trust-region step.
            smallest_scale = min(self.backtracking_scales)
            T_new = np.clip(T0 + smallest_scale * dT, self.thrusterMinTList, self.thrusterMaxTList)
            alpha_new = np.clip(alpha0 + smallest_scale * dAlpha,
                                self.thrusterMinAlphaList, self.thrusterMaxAlphaList)
            mode = "forced_small_step"
        else:
            T_new, alpha_new = T0, alpha0
            mode = "held_old_state"

        self.last_info.update({
            "step_mode": mode,
            "old_score": old_score,
            "best_score": best_score,
            "best_scale": best_scale,
        })
        if self.debug and mode in ("forced_small_step", "held_old_state"):
            print(f"[QP2] backtracking {mode}: old_score={old_score:.6g}, "
                  f"best_score={best_score:.6g}, best_scale={best_scale}")
        return T_new, alpha_new

    def _fallback_bounded_lsq(self, B, tau0, T0, alpha0, dT_min, dT_max, reason):
        """
        Emergency fallback when OSQP fails.
        It updates thrust magnitude with fixed azimuth angle by bounded least squares.
        This is deliberately simple and stable; it prevents the outer loop from holding
        a stale command forever.
        """
        try:
            A = B / self.err_scale
            b = (self.targetLCForce - tau0).flatten() / self.err_scale.flatten()
            res = lsq_linear(A, b,
                             bounds=(dT_min.flatten(), dT_max.flatten()),
                             lsmr_tol='auto', max_iter=80, verbose=0)
            dT = np.asarray(res.x, dtype=np.float64).reshape(-1, 1)
            self.thrusterForce = np.clip(T0 + dT, self.thrusterMinTList, self.thrusterMaxTList)
            self.thrusterAngle = alpha0.copy()
            self.last_info.update({"solver_status": reason, "step_mode": "fallback_bounded_lsq"})
            if self.debug:
                print(f"[QP2] OSQP failed ({reason}); fallback bounded LSQ applied.")
            return False
        except Exception as e:
            self.last_info.update({"solver_status": reason, "step_mode": "fallback_failed", "fallback_error": str(e)})
            if self.debug:
                print(f"[QP2] fallback failed after OSQP status {reason}: {e}")
            return False

    def QPLoop(self, tauLC, pose):
        self.targetLCForce = np.asarray(tauLC, dtype=np.float64).reshape(3, 1)
        pose = np.asarray(pose, dtype=np.float64).reshape(3, 1)

        yaw = pose[2, 0]
        self.J = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw),  np.cos(yaw), 0.0],
            [0.0,          0.0,         1.0]
        ], dtype=np.float64)

        n = self.thrustNum
        m = 2 * n + 3          # variables: [dT, dAlpha, slack]
        T0 = self.thrusterForce.copy()
        alpha0 = self.thrusterAngle.copy()

        B, dBT = self._calc_B3_and_dBT(alpha0, T0)
        tau0 = B @ T0
        dT_min, dT_max, d_alpha_min, d_alpha_max = self._calc_step_bounds(T0, alpha0)

        # Linearized equality:
        # B*dT + dBT*dAlpha + slack = target - B(alpha0)*T0
        Aeq = np.hstack((B, dBT, np.eye(3)))
        beq = self.targetLCForce - tau0

        lb = np.vstack((dT_min, d_alpha_min, -self.slack_bound))
        ub = np.vstack((dT_max, d_alpha_max,  self.slack_bound))

        P_dT = (self.W_deltaT + self.W_energy) * np.eye(n, dtype=np.float64)
        P_dAlpha = self.W_deltaAlpha * np.eye(n, dtype=np.float64)
        P_slack = self.W_slack_base * np.eye(3, dtype=np.float64)
        P_real = block_diag(P_dT, P_dAlpha, P_slack)

        q_real = np.zeros((m, 1), dtype=np.float64)
        q_real[:n, 0:1] = self.W_energy * T0

        # Scale real variable x = D*z for OSQP conditioning.
        scale_dT = np.maximum(np.abs(self.thrusterMaxDTList), 10.0)
        scale_alpha = np.maximum(np.abs(self.thrusterMaxDAlphaList), 0.01)
        scale_slack = np.maximum(np.abs(self.slack_bound), 1.0)
        D_vec = np.vstack((scale_dT, scale_alpha, scale_slack)).flatten()

        P_scaled = P_real * np.outer(D_vec, D_vec)
        q_scaled = q_real.flatten() * D_vec
        Aeq_scaled = Aeq * D_vec

        A_sparse = sparse.vstack([
            sparse.csc_matrix(Aeq_scaled),
            sparse.diags(D_vec, format='csc')
        ], format='csc')
        l_bound = np.vstack((beq, lb)).flatten()
        u_bound = np.vstack((beq, ub)).flatten()

        self.last_info = {}
        prob = osqp.OSQP()
        try:
            prob.setup(sparse.csc_matrix(P_scaled), q_scaled, A_sparse, l_bound, u_bound,
                       verbose=False,
                       max_iter=self.osqp_max_iter,
                       polish=self.osqp_polish,
                       eps_abs=self.osqp_eps_abs,
                       eps_rel=self.osqp_eps_rel)
            prob.warm_start(x=np.zeros(m, dtype=np.float64))
            res = prob.solve()
            status = str(res.info.status)
            self.last_info["solver_status"] = status

            if status not in ("solved", "solved inaccurate") or res.x is None:
                return self._fallback_bounded_lsq(B, tau0, T0, alpha0, dT_min, dT_max, status)

            x_real = (res.x * D_vec).reshape(-1, 1)
            dT = x_real[:n]
            dAlpha = x_real[n:2 * n]

            if self.enable_backtracking:
                self.thrusterForce, self.thrusterAngle = self._apply_backtracking(
                    T0, alpha0, dT, dAlpha, tau0
                )
            else:
                self.thrusterForce = np.clip(T0 + dT, self.thrusterMinTList, self.thrusterMaxTList)
                self.thrusterAngle = np.clip(alpha0 + dAlpha,
                                             self.thrusterMinAlphaList,
                                             self.thrusterMaxAlphaList)
                self.last_info["step_mode"] = "no_backtracking"

            return True

        except Exception as e:
            # Do not silently freeze. Try the bounded-LSQ fallback once.
            return self._fallback_bounded_lsq(B, tau0, T0, alpha0, dT_min, dT_max, f"exception: {e}")

    def getState(self):
        B = np.zeros((5, self.thrustNum), dtype=np.float64)
        for i in range(self.thrustNum):
            ca = np.cos(self.thrusterAngle[i, 0])
            sa = np.sin(self.thrusterAngle[i, 0])
            x_i = self.thrusterXList[i, 0]
            y_i = self.thrusterYList[i, 0]
            z_i = self.thrusterZList[i, 0]

            B[0, i] = ca
            B[1, i] = sa
            B[2, i] = x_i * sa - y_i * ca
            B[3, i] = -z_i * sa
            B[4, i] = z_i * ca

        tau_local = B @ self.thrusterForce
        return tau_local.flatten(), self.thrusterForce.flatten(), self.thrusterAngle.flatten()
