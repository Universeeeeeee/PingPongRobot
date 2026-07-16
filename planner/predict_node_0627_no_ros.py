import time
import math
import argparse
import threading
import numpy as np
import zmq

try:
    import rclpy
    from std_msgs.msg import Float64MultiArray
except ImportError:
    rclpy = None
    Float64MultiArray = None


# ==========================================
# 0. 基于论文推导的球拍指令计算模块 (世界坐标系)
# ==========================================
def _solve_v_outgoing(predictor, p_racket: np.ndarray, p_landing: np.ndarray,
                      dt_flight: float, n_iter: int = 4) -> np.ndarray:
    """Newton iteration: find v_out so the ball reaches p_landing in dt_flight
    under the full physics model (drag + spin_az + gravity).

    Starts from the parabolic estimate with g_eff, then corrects via:
        v += (p_target - p_predicted) / dt_flight
    Converges in 3-4 iterations for typical outgoing trajectories (<10 m, <0.6 s).
    """
    g_eff = predictor.g + predictor.spin_az
    v = (p_landing - p_racket) / dt_flight + np.array([0.0, 0.0, 0.5 * g_eff * dt_flight])
    for _ in range(n_iter):
        p_pred, _ = predictor.predict_state_at_time(p_racket, v, dt_flight)
        v = v + (p_landing - p_pred) / dt_flight
    return v


def calculate_racket_command(p_racket: np.ndarray, v_incoming: np.ndarray,
                             p_landing: np.ndarray, dt_flight: float, Cr: float = 0.70,
                             predictor=None):
    """Compute required racket velocity for a given outgoing target.

    predictor: FastTrajectoryPredictor instance.  When provided, v_outgoing is
    solved under the full drag + spin_az + gravity model.  Without it, falls
    back to the parabolic (g only) approximation.
    """
    if predictor is not None:
        v_outgoing = _solve_v_outgoing(predictor, p_racket, p_landing, dt_flight)
    else:
        g_eff = 9.81
        v_outgoing = (p_landing - p_racket) / dt_flight + np.array([0.0, 0.0, 0.5 * g_eff * dt_flight])

    v_diff = v_outgoing - v_incoming
    norm_v_diff = np.linalg.norm(v_diff)

    if norm_v_diff < 1e-6:
        return np.zeros(3)

    u = v_diff / norm_v_diff
    v_racket_mag = (np.dot(v_outgoing, u) + Cr * np.dot(v_incoming, u)) / (1.0 + Cr)
    return v_racket_mag * u


# ==========================================
# 1. 物理轨迹预测器 (带桌面反弹模型，世界坐标系)
# ==========================================
class FastTrajectoryPredictor:
    def __init__(self, table_height: float = 0.02):
        self.m = 0.0040
        self.g = 9.81
        self.spin_az = 3.5        # topspin Magnus downward accel (m/s²)
        self.Cd_x = 0.000454
        self.Cd_y = 0.0008
        self.Cd_z = 0.0005
        self.table_height = table_height

        self.k = np.array([0.50, 0.94, -0.905])
        self.b = np.array([-0.51, 0.0, 0.0])

    def predict_state_at_time(self, p0: np.ndarray, v0: np.ndarray, t: float):
        x0, y0, z0 = p0
        Vx0, Vy0, Vz0 = v0
        if t < 1e-9: return np.copy(p0), np.copy(v0)

        Vx = self.m * Vx0 / (self.Cd_x * Vx0 * t + self.m) if Vx0 > 0 else self.m * Vx0 / (self.m - self.Cd_x * Vx0 * t)
        x = x0 + (self.m / self.Cd_x) * np.log(1 + self.Cd_x * Vx0 * t / self.m) if Vx0 > 0 else x0 - (self.m / self.Cd_x) * np.log(1 - self.Cd_x * Vx0 * t / self.m)

        Vy = self.m * Vy0 / (self.Cd_y * Vy0 * t + self.m) if Vy0 > 0 else self.m * Vy0 / (self.m - self.Cd_y * Vy0 * t)
        y = y0 + (self.m / self.Cd_y) * np.log(1 + self.Cd_y * Vy0 * t / self.m) if Vy0 > 0 else y0 - (self.m / self.Cd_y) * np.log(1 - self.Cd_y * Vy0 * t / self.m)

        g_eff = self.g + self.spin_az
        mg_Cd = np.sqrt(self.m * g_eff / self.Cd_z)
        sqrt_term = np.sqrt(self.Cd_z * g_eff / self.m)

        if Vz0 >= 0:
            t_peak = (1 / sqrt_term) * np.arctan(Vz0 / mg_Cd) if mg_Cd > 1e-9 else 0.0
            if t <= t_peak:
                Vz = mg_Cd * np.tan(-sqrt_term * t + np.arctan(Vz0 / mg_Cd))
                log_val = np.cos(sqrt_term * t) + (Vz0 / mg_Cd) * np.sin(sqrt_term * t)
                z = z0 + (self.m / self.Cd_z) * np.log(log_val) if log_val > 1e-9 else z0
            else:
                p_peak, v_peak = self.predict_state_at_time(p0, v0, t_peak)
                t_fall = t - t_peak
                exp_term = np.exp(2 * t_fall * sqrt_term)
                Vz = mg_Cd * (1 - exp_term) / (1 + exp_term) + v_peak[2]
                z = p_peak[2] - (self.m / (2 * self.Cd_z)) * np.log(((1 + exp_term) ** 2) / (4 * exp_term)) + v_peak[2] * t_fall
        else:
            ratio_v = np.clip(-Vz0 / mg_Cd, 0.0, 1.0 - 1e-6)
            phase = np.arctanh(ratio_v)
            Vz = -mg_Cd * np.tanh(sqrt_term * t + phase)
            z = z0 + (mg_Cd / sqrt_term) * (np.log(np.cosh(phase)) - np.log(np.cosh(sqrt_term * t + phase)))

        return np.array([x, y, z]), np.array([Vx, Vy, Vz])

    def predict_hitting_point(self, p_init: np.ndarray, v_init: np.ndarray, target_x: float = -1.37, max_time: float = 2.0, dt: float = 0.01):
        t_total = 0.0
        t_seg = 0.0
        max_bounces = 3
        bounce_count = 0

        p0, v0 = np.copy(p_init), np.copy(v_init)
        p_curr, v_curr = p0, v0

        if p0[2] < self.table_height:
            p0[2] = self.table_height
            p_curr = p0.copy()

        while t_total < max_time:
            t_seg += dt
            t_total += dt
            p_next, v_next = self.predict_state_at_time(p0, v0, t_seg)

            if p_curr[0] >= target_x and p_next[0] < target_x:
                ratio = (target_x - p_curr[0]) / (p_next[0] - p_curr[0])
                hit_y = p_curr[1] + ratio * (p_next[1] - p_curr[1])
                hit_z = p_curr[2] + ratio * (p_next[2] - p_curr[2])
                hit_pos = np.array([target_x, hit_y, hit_z])

                hit_vx = v_curr[0] + ratio * (v_next[0] - v_curr[0])
                hit_vy = v_curr[1] + ratio * (v_next[1] - v_curr[1])
                hit_vz = v_curr[2] + ratio * (v_next[2] - v_curr[2])
                hit_vel = np.array([hit_vx, hit_vy, hit_vz])

                hit_t = (t_total - dt) + ratio * dt
                return hit_pos, hit_vel, hit_t

            if p_curr[2] >= self.table_height and p_next[2] < self.table_height:
                ratio = (self.table_height - p_curr[2]) / (p_next[2] - p_curr[2])
                bounce_t_seg = (t_seg - dt) + ratio * dt

                p_impact, v_impact = self.predict_state_at_time(p0, v0, bounce_t_seg)
                p_impact[2] = self.table_height
                v_reb = self.k * v_impact + self.b

                if abs(v_reb[2]) < 0.1:
                    return None, None, None

                bounce_count += 1
                if bounce_count > max_bounces:
                    return None, None, None

                p0, v0 = p_impact, v_reb
                t_seg = 0.0
                t_total = (t_total - dt) + ratio * dt
                p_next, v_next = p0, v0

            p_curr, v_curr = p_next, v_next

        return None, None, None

    def compute_full_trajectory(self, p_init: np.ndarray, v_init: np.ndarray,
                                target_x: float = -1.47, max_time: float = 2.0, dt: float = 0.005):
        points = [p_init.copy()]
        t_total = 0.0
        t_seg = 0.0
        bounce_count = 0
        max_bounces = 3

        p0, v0 = np.copy(p_init), np.copy(v_init)
        p_curr = p0.copy()

        while t_total < max_time:
            t_seg += dt
            t_total += dt
            p_next, v_next = self.predict_state_at_time(p0, v0, t_seg)

            if p_curr[0] >= target_x and p_next[0] < target_x:
                ratio = (target_x - p_curr[0]) / (p_next[0] - p_curr[0])
                hit_y = p_curr[1] + ratio * (p_next[1] - p_curr[1])
                hit_z = p_curr[2] + ratio * (p_next[2] - p_curr[2])
                points.append(np.array([target_x, hit_y, hit_z]))
                break

            if p_curr[2] >= self.table_height and p_next[2] < self.table_height:
                ratio = (self.table_height - p_curr[2]) / (p_next[2] - p_curr[2])
                bounce_t_seg = (t_seg - dt) + ratio * dt

                p_impact, v_impact = self.predict_state_at_time(p0, v0, bounce_t_seg)
                p_impact[2] = self.table_height
                v_reb = self.k * v_impact + self.b

                if abs(v_reb[2]) < 0.1:
                    break

                bounce_count += 1
                if bounce_count > max_bounces:
                    break

                points.append(p_impact.copy())
                p0, v0 = p_impact, v_reb
                t_seg = 0.0
                t_total = (t_total - dt) + ratio * dt
                p_next = p0.copy()

            points.append(p_next.copy())
            p_curr = p_next.copy()

        return np.array(points)


# ==========================================
# 2. 3D 实时可视化线程 + 时间预测误差曲线
# ==========================================
class TrajectoryVisualizer:
    """
    在独立线程中运行 matplotlib 可视化：
      - Figure 1: 3D 轨迹图
      - Figure 2: 预测到达时间误差随 t_strike 变化曲线（有符号，单位 ms）
    """

    def __init__(self, table_height: float = 0.02, target_x: float = -1.47):
        self.table_height = table_height
        self.target_x = target_x

        self._lock = threading.Lock()

        # ---- 3D 图数据 ----
        self._pred_pts: np.ndarray = np.empty((0, 3))
        self._pred_hit: np.ndarray = None
        self._pred_dirty = False
        self._real_pts: list = []
        self._real_dirty = False
        self._rally_count = 0
        self._status_text = "等待来球..."
        self._hit_history: list = []
        self._bbox: tuple = None
        self._bbox_dirty = False

        # ---- 时间误差曲线数据 ----
        # 每帧记录 (t_strike, predicted_absolute_hit_time)
        self._frame_predictions: list = []

        # finalize_rally 后填充，格式: (t_arr, err_ms_arr)
        self._err_curve: tuple = None
        self._err_dirty = False

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    #  外部接口（主循环调用，线程安全）
    # ------------------------------------------------------------------ #

    def new_rally(self, rally_count: int):
        with self._lock:
            self._rally_count = rally_count
            self._real_pts = []
            self._hit_history = []
            self._frame_predictions = []
            self._pred_pts = np.empty((0, 3))
            self._pred_hit = None
            self._status_text = f"Ball #{rally_count} — Tracking..."
            self._real_dirty = True
            self._pred_dirty = True

    def update_predicted(self, traj_pts: np.ndarray, hit_pos: np.ndarray):
        with self._lock:
            self._pred_pts = traj_pts.copy()
            self._pred_hit = hit_pos.copy() if hit_pos is not None else None
            if hit_pos is not None:
                self._hit_history.append((float(hit_pos[1]), float(hit_pos[2])))
            self._pred_dirty = True

    def record_prediction(self, t_strike: float, predicted_abs_hit_time: float,
                          hit_pos: np.ndarray):
        """
        每帧记录：
          t_strike              — 距击球剩余时间 (s)
          predicted_abs_hit_time — 预测的绝对到达时刻 (mocap 秒)
          hit_pos               — 预测击球点 (世界坐标，用于位置误差曲线)
        """
        with self._lock:
            self._frame_predictions.append((
                float(t_strike),
                float(predicted_abs_hit_time),
                np.array([float(hit_pos[1]), float(hit_pos[2])])  # 只存 Y/Z
            ))

    def update_real(self, pos: np.ndarray):
        with self._lock:
            self._real_pts.append(pos.copy())
            self._real_dirty = True

    def set_status(self, text: str):
        with self._lock:
            self._status_text = text

    def finalize_rally(self, real_hit_y: float, real_hit_z: float, real_hit_time: float):
        """
        球局结束时调用。
          real_hit_y/z  — 真实穿越 target_x 平面的 Y/Z 坐标 (m)
          real_hit_time — 真实穿越时刻 (mocap 秒)
        """
        with self._lock:
            # ---- bbox ----
            if len(self._hit_history) >= 2:
                yz = np.array(self._hit_history)
                self._bbox = (
                    float(yz[:, 0].min()), float(yz[:, 0].max()),
                    float(yz[:, 1].min()), float(yz[:, 1].max()),
                )
                self._bbox_dirty = True
                print(f"[BBox] Y=[{self._bbox[0]:.3f}, {self._bbox[1]:.3f}]  "
                      f"Z=[{self._bbox[2]:.3f}, {self._bbox[3]:.3f}]  "
                      f"共 {len(self._hit_history)} 个预测点")

            n_frames = len(self._frame_predictions)
            if n_frames == 0:
                return

            t_arr    = np.array([f[0] for f in self._frame_predictions])
            pred_abs = np.array([f[1] for f in self._frame_predictions])
            pred_yz  = np.array([f[2] for f in self._frame_predictions])  # (N, 2)

            # ---- 位置误差 ----
            real_yz   = np.array([real_hit_y, real_hit_z])
            err_pos   = pred_yz - real_yz                      # (N, 2) m
            err_y_cm  = np.abs(err_pos[:, 0]) * 100.0
            err_z_cm  = np.abs(err_pos[:, 1]) * 100.0
            err_tot_cm = np.linalg.norm(err_pos, axis=1) * 100.0

            # ---- 时间误差（有符号，ms）----
            err_ms = (pred_abs - real_hit_time) * 1000.0

            self._err_curve = (t_arr, err_y_cm, err_z_cm, err_tot_cm, err_ms)
            self._err_dirty = True
            print(f"[误差曲线] {n_frames} 帧 | "
                  f"最终位置误差 {err_tot_cm[-1]:.1f} cm | "
                  f"最终时间误差 {err_ms[-1]:+.1f} ms")

    # ------------------------------------------------------------------ #
    #  内部渲染循环
    # ------------------------------------------------------------------ #

    def _run(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        plt.ion()

        # ---- Figure 1: 3D 轨迹 ----
        fig3d = plt.figure(figsize=(16, 10), facecolor="#0d1117")
        ax3d = fig3d.add_subplot(111, projection="3d", facecolor="#0d1117")
        fig3d.suptitle("🏓 Trajectory Visualizer", color="#e6edf3",
                       fontsize=13, fontweight="bold", y=0.98)
        self._style_axes(ax3d)
        self._draw_table(ax3d)

        pred_line,   = ax3d.plot([], [], [], color="#58a6ff", lw=1.8,
                                 label="Predicted", alpha=0.9, zorder=5)
        real_line,   = ax3d.plot([], [], [], color="#f78166", lw=1.8,
                                 label="Actual",    alpha=0.9, zorder=5)
        hit_scatter  = ax3d.scatter([], [], [], color="#ffa657", s=80,
                                    marker="*", label="Hit point", zorder=6, depthshade=False)
        ball_scatter = ax3d.scatter([], [], [], color="#f78166", s=40,
                                    marker="o", zorder=7, depthshade=False)
        bbox_line,   = ax3d.plot([], [], [], color="#39d353", lw=1.5,
                                 linestyle="--", label="Pred bbox", alpha=0.85, zorder=6)
        ax3d.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d",
                    labelcolor="#e6edf3", fontsize=9)
        status_txt = fig3d.text(0.5, 0.01, "Waiting for ball...", ha="center",
                                color="#8b949e", fontsize=9)
        plt.figure(fig3d.number)
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])

        # ---- Figure 2: 位置误差 + 时间误差（双子图）----
        fig_err = plt.figure(figsize=(10, 8), facecolor="#0d1117")
        fig_err.suptitle("Prediction Error vs Time-to-Strike",
                         color="#e6edf3", fontsize=12, fontweight="bold", y=0.98)

        # 上图：位置误差
        ax_pos = fig_err.add_subplot(211, facecolor="#0d1117")
        self._style_err_ax(ax_pos,
                           ylabel="Position Error (cm)",
                           title="Strike Position Error")

        line_tot, = ax_pos.plot([], [], color="#58a6ff",
                                lw=2.0, label="Total |err|", zorder=4)
        line_y,   = ax_pos.plot([], [], color="#ffa657",
                                lw=1.5, linestyle="--", label="Y error", zorder=3)
        line_z,   = ax_pos.plot([], [], color="#39d353",
                                lw=1.5, linestyle="--", label="Z error", zorder=3)

        ax_pos.legend(loc="upper right", facecolor="#161b22", edgecolor="#30363d",
                      labelcolor="#e6edf3", fontsize=9)
        ax_pos.set_xlim(0.65, -0.02)
        ax_pos.set_ylim(0, 20)

        pos_txt = ax_pos.text(0.02, 0.96, "waiting...",
                              transform=ax_pos.transAxes,
                              color="#8b949e", fontsize=8, va="top")

        # 下图：时间误差
        ax_time = fig_err.add_subplot(212, facecolor="#0d1117")
        self._style_err_ax(ax_time,
                           ylabel="Time Prediction Error (ms)",
                           title="Arrival Time Error  (signed: + = predicted late)")

        line_err, = ax_time.plot([], [], color="#58a6ff",
                                 lw=2.0, label="Time error (signed)", zorder=4)
        ax_time.axhline(0.0, color="#8b949e", lw=1.0, linestyle="--",
                        label="Zero error", zorder=5)

        ax_time.legend(loc="upper right", facecolor="#161b22", edgecolor="#30363d",
                       labelcolor="#e6edf3", fontsize=9)
        ax_time.set_xlim(0.65, -0.02)
        ax_time.set_ylim(-50, 50)

        time_txt = ax_time.text(0.02, 0.96, "waiting...",
                                transform=ax_time.transAxes,
                                color="#8b949e", fontsize=8, va="top")

        plt.figure(fig_err.number)
        plt.tight_layout(rect=[0, 0.02, 1, 0.96])

        # ---- 渲染主循环 ----
        while plt.fignum_exists(fig3d.number):
            needs_3d  = False
            needs_err = False

            with self._lock:
                # -- 3D 数据 --
                if self._pred_dirty:
                    pred = self._pred_pts.copy()
                    hit  = self._pred_hit.copy() if self._pred_hit is not None else None
                    self._pred_dirty = False
                    needs_3d = True
                else:
                    pred, hit = None, None

                if self._real_dirty:
                    real = np.array(self._real_pts) if self._real_pts else np.empty((0, 3))
                    self._real_dirty = False
                    needs_3d = True
                else:
                    real = None

                if self._bbox_dirty:
                    bbox = self._bbox
                    self._bbox_dirty = False
                    needs_3d = True
                else:
                    bbox = None

                status = self._status_text
                rally  = self._rally_count

                # -- 误差数据 --
                if self._err_dirty:
                    err_curve_snap = self._err_curve
                    err_rally      = self._rally_count
                    self._err_dirty = False
                    needs_err = True
                else:
                    err_curve_snap = None
                    err_rally      = 0

            # ---- 更新 3D 图 ----
            if needs_3d:
                if pred is not None:
                    if len(pred) > 1:
                        pred_line.set_data(pred[:, 0], pred[:, 1])
                        pred_line.set_3d_properties(pred[:, 2])
                    else:
                        pred_line.set_data([], [])
                        pred_line.set_3d_properties([])
                    if hit is not None:
                        hit_scatter._offsets3d = ([hit[0]], [hit[1]], [hit[2]])
                    else:
                        hit_scatter._offsets3d = ([], [], [])

                if real is not None:
                    if len(real) > 1:
                        real_line.set_data(real[:, 0], real[:, 1])
                        real_line.set_3d_properties(real[:, 2])
                        ball_scatter._offsets3d = ([real[-1, 0]], [real[-1, 1]], [real[-1, 2]])
                    else:
                        real_line.set_data([], [])
                        real_line.set_3d_properties([])
                        ball_scatter._offsets3d = ([], [], [])

                if bbox is not None:
                    y_min, y_max, z_min, z_max = bbox
                    bx = self.target_x
                    by = [y_min, y_max, y_max, y_min, y_min]
                    bz = [z_min, z_min, z_max, z_max, z_min]
                    bbox_line.set_data([bx] * 5, by)
                    bbox_line.set_3d_properties(bz)

                status_txt.set_text(f"Ball #{rally}  |  {status}")
                fig3d.canvas.draw_idle()

            # ---- 更新误差曲线（双子图）----
            if needs_err and err_curve_snap is not None:
                t_arr, err_y, err_z, err_tot, err_ms = err_curve_snap

                # 上图：位置误差
                line_tot.set_data(t_arr, err_tot)
                line_y.set_data(t_arr,   err_y)
                line_z.set_data(t_arr,   err_z)
                max_pos = err_tot.max() if len(err_tot) > 0 else 20
                ax_pos.set_ylim(0, max(20, max_pos * 1.15))
                final_pos = err_tot[-1] if len(err_tot) > 0 else float('nan')
                pos_txt.set_text(
                    f"Ball #{err_rally}  |  {len(t_arr)} frames  "
                    f"|  final pos err = {final_pos:.1f} cm"
                )

                # 下图：时间误差
                line_err.set_data(t_arr, err_ms)
                max_abs = np.abs(err_ms).max() if len(err_ms) > 0 else 50
                margin  = max(50.0, max_abs * 1.2)
                ax_time.set_ylim(-margin, margin)
                final_t = err_ms[-1] if len(err_ms) > 0 else float('nan')
                time_txt.set_text(
                    f"Ball #{err_rally}  |  final time err = {final_t:+.1f} ms"
                )

                fig_err.canvas.draw_idle()

            try:
                fig3d.canvas.flush_events()
            except Exception:
                pass
            try:
                fig_err.canvas.flush_events()
            except Exception:
                pass

            time.sleep(0.04)

        plt.close("all")

    # ------------------------------------------------------------------ #
    #  样式工具
    # ------------------------------------------------------------------ #

    @staticmethod
    def _style_axes(ax):
        for spine in [ax.xaxis, ax.yaxis, ax.zaxis]:
            spine.pane.fill = False
            spine.pane.set_edgecolor("#21262d")
            spine._axinfo["grid"]["color"] = "#21262d"
        ax.tick_params(colors="#8b949e", labelsize=7)
        ax.set_xlabel("X (m)", color="#8b949e", fontsize=8, labelpad=4)
        ax.set_ylabel("Y (m)", color="#8b949e", fontsize=8, labelpad=4)
        ax.set_zlabel("Z (m)", color="#8b949e", fontsize=8, labelpad=4)
        ax.set_xlim(-2.2,  1.5)
        ax.set_ylim(-1.1,  1.1)
        ax.set_zlim(-0.05, 1.4)
        ax.view_init(elev=18, azim=-55)

    @staticmethod
    def _style_err_ax(ax, ylabel: str, title: str):
        ax.set_facecolor("#0d1117")
        ax.tick_params(colors="#8b949e", labelsize=8)
        ax.set_xlabel("Time to Strike (s)", color="#8b949e", fontsize=9)
        ax.set_ylabel(ylabel, color="#8b949e", fontsize=9)
        ax.set_title(title, color="#c9d1d9", fontsize=10, pad=6)
        for spine in ax.spines.values():
            spine.set_edgecolor("#21262d")
        ax.grid(True, color="#21262d", linewidth=0.6, linestyle="--")
        ax.xaxis.label.set_color("#8b949e")
        ax.yaxis.label.set_color("#8b949e")
        for label in ax.get_xticklabels() + ax.get_yticklabels():
            label.set_color("#8b949e")

    @staticmethod
    def _draw_table(ax):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        th = 0.02
        tx = [-1.37, 1.37]
        ty = [-0.7625, 0.7625]
        table_verts = [
            [tx[0], ty[0], th], [tx[1], ty[0], th],
            [tx[1], ty[1], th], [tx[0], ty[1], th]
        ]
        table_poly = Poly3DCollection([table_verts], alpha=0.18,
                                      facecolor="#1f6feb", edgecolor="#388bfd", linewidth=0.8)
        ax.add_collection3d(table_poly)
        net_h = 0.02 + 0.1525
        net_xs = [0, 0, 0, 0]
        net_ys = [-0.7625, 0.7625, 0.7625, -0.7625]
        net_zs = [th, th, net_h, net_h]
        net_poly = Poly3DCollection([[list(zip(net_xs, net_ys, net_zs))[i] for i in range(4)]],
                                    alpha=0.25, facecolor="#ffffff", edgecolor="#aaaaaa", linewidth=0.6)
        ax.add_collection3d(net_poly)
        corners = [
            [tx[0], ty[0], th], [tx[1], ty[0], th],
            [tx[1], ty[1], th], [tx[0], ty[1], th], [tx[0], ty[0], th]
        ]
        c = np.array(corners)
        ax.plot(c[:, 0], c[:, 1], c[:, 2], color="#388bfd", lw=0.8, alpha=0.6)


# ==========================================
# 4. 主程序
# ==========================================

def main(args=None):
    parser = argparse.ArgumentParser(description="Table Tennis Strategy Node")
    parser.add_argument("--render", action="store_true",
                        help="开启 3D 实时轨迹可视化")
    parser.add_argument("--no-ros", action="store_true",
                        help="诊断模式：不初始化 ROS2，不发布 /wbc_racket_command，只打印预测结果")
    cli_args, remaining = parser.parse_known_args()
    RENDER = cli_args.render
    NO_ROS = cli_args.no_ros

    if not NO_ROS and rclpy is None:
        raise RuntimeError("ROS2/rclpy is not available. Use --no-ros for Mac-side ZMQ prediction diagnostics.")

    ros_node = None
    cmd_pub = None
    if not NO_ROS:
        rclpy.init(args=remaining if remaining else args)
        ros_node = rclpy.create_node('table_tennis_commander_node')
        cmd_pub = ros_node.create_publisher(Float64MultiArray, '/wbc_racket_command', 10)
        ros_node.get_logger().info("🏓 策略节点启动，将在 /wbc_racket_command 持续发布精简指令 (Size=14)...")
        if RENDER:
            ros_node.get_logger().info("🎨 可视化模式已开启")
    else:
        print("[NO-ROS] 诊断模式启动：跳过 ROS2，只验证 ZMQ 输入和预测逻辑。")
        if RENDER:
            print("[NO-ROS] 可视化模式已开启")

    ZMQ_SUB_PORT = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://127.0.0.1:{ZMQ_SUB_PORT}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, 100)

    print("[系统初始化] 已连接动捕数据源，等待接收数据...")

    predictor = FastTrajectoryPredictor(table_height=0.02)

    TARGET_HIT_X = -1.47
    LOCK_TIME_THRESHOLD = 0.62

    ROBOT_BASE_ORIGIN_WORLD = np.array([-1.87, 0.0, -0.76])

    p_landing_target = np.array([0.685, 0.0, 0.02])
    desired_flight_time = 0.6

    ball_incoming = False
    has_crossed_target = False
    pending_lock = False

    rally_count = 0
    publish_count = 0

    # 是否已经产生过至少一次有效预测。
    # default 只允许出现在首次有效预测之前；
    # 一旦 has_ever_predicted=True，后续即使上一球已经过期、下一球还没预测成功，
    # 也继续发布上一球的 pred 指令，只是 t_strike 持续下降并最低夹到 -0.9。
    has_ever_predicted = False

    locked_hit_pos = None
    locked_hit_vel = None
    locked_absolute_hit_time = None
    locked_v_racket_cmd = None
    locked_p_racket_relative = None
    locked_v_racket_cmd_relative = None

    prev_pos_m = None
    prev_mocap_time = None   # 用于插值真实穿越时刻

    current_robot_base_pos = np.array([-1.8, 0.0, 0.0])
    current_robot_base_quat = np.array([0.0, 0.0, 0.0, 1.0])

    wall_time_at_lock = None
    mocap_time_at_lock = None
    last_mocap_time = None

    last_flag = 0.

    # ------------------------------------------------------------
    # 默认指令：节点启动后、还没有可用击球预测前持续发布。
    # 目的：让下游 WBC 控制器一开始就有一个合理的球拍目标，
    # 避免等待第一颗球时目标为空或停留在不确定状态。
    # 注意：base pose 使用动捕实时给出的 current_robot_base_pos / quat；
    #      t_strike 固定为 -0.9，表示“当前没有有效来球/击球时间”。
    # ------------------------------------------------------------
    DEFAULT_T_STRIKE = -0.9
    DEFAULT_P_RACKET_RELATIVE = np.array([0.3, 0.0, 0.8], dtype=float)
    DEFAULT_V_RACKET_RELATIVE = np.array([1.1, 0.0, 0.3], dtype=float)

    initial_msg_data = [
        float(current_robot_base_pos[0]), float(current_robot_base_pos[1]), float(current_robot_base_pos[2]),
        float(current_robot_base_quat[0]), float(current_robot_base_quat[1]), float(current_robot_base_quat[2]), float(current_robot_base_quat[3]),
        float(DEFAULT_T_STRIKE),
        float(DEFAULT_P_RACKET_RELATIVE[0]), float(DEFAULT_P_RACKET_RELATIVE[1]), float(DEFAULT_P_RACKET_RELATIVE[2]),
        float(DEFAULT_V_RACKET_RELATIVE[0]), float(DEFAULT_V_RACKET_RELATIVE[1]), float(DEFAULT_V_RACKET_RELATIVE[2])
    ]
    if not NO_ROS:
        ros_msg = Float64MultiArray()
        ros_msg.data = initial_msg_data

    debug_log = open("/tmp/tmp.txt", "w", buffering=1)
    debug_log.write("# timestamp  ball_count  t_strike  "
                    "base_x base_y base_z  qx qy qz qw  "
                    "p_x p_y p_z  v_x v_y v_z\n")

    viz: TrajectoryVisualizer = TrajectoryVisualizer(
        table_height=0.02, target_x=TARGET_HIT_X
    ) if RENDER else None
    last_no_ros_print_s = 0.0
    last_no_ros_mode = None
    last_ball_diag_print_s = 0.0
    ball_diag_window_start_s = time.time()
    ball_diag_count = 0
    last_ball_diag = None

    def should_keep_running():
        return True if NO_ROS else rclpy.ok()

    def incoming_gate_reason(pos_m, vel_m):
        if not (vel_m[0] < -1.0):
            return f"vx_not_incoming:{vel_m[0]:+.3f}"
        if not (-1.37 < pos_m[0] < 1.37):
            return f"x_out:{pos_m[0]:+.3f}"
        if not ((-1.525 / 2) < pos_m[1] < (1.525 / 2)):
            return f"y_out:{pos_m[1]:+.3f}"
        if not (0.0 < pos_m[2] < 1.0):
            return f"z_out:{pos_m[2]:+.3f}"
        return "pass"

    def maybe_print_ball_diag(wall_now):
        nonlocal last_ball_diag_print_s, ball_diag_window_start_s, ball_diag_count
        if not NO_ROS or wall_now - last_ball_diag_print_s < 1.0:
            return
        elapsed = max(1e-6, wall_now - ball_diag_window_start_s)
        hz = ball_diag_count / elapsed
        if last_ball_diag is None:
            print(f"[NO-ROS] ball_hz={hz:.1f} latest_ball=none")
        else:
            pos_m, vel_m, reason = last_ball_diag
            print(
                f"[NO-ROS] ball_hz={hz:.1f} "
                f"pos=({pos_m[0]:+.3f},{pos_m[1]:+.3f},{pos_m[2]:+.3f}) "
                f"vel=({vel_m[0]:+.3f},{vel_m[1]:+.3f},{vel_m[2]:+.3f}) "
                f"gate={reason}"
            )
        last_ball_diag_print_s = wall_now
        ball_diag_window_start_s = wall_now
        ball_diag_count = 0

    def publish_or_print(cur_msg, mode, t_strike, p_cmd, v_cmd, wall_now):
        nonlocal last_no_ros_print_s, last_no_ros_mode
        if NO_ROS:
            if mode != last_no_ros_mode or wall_now - last_no_ros_print_s >= 1.0:
                print(
                    f"[NO-ROS] mode={mode} t_strike={t_strike:.4f} "
                    f"base=({cur_msg[0]:+.3f},{cur_msg[1]:+.3f},{cur_msg[2]:+.3f}) "
                    f"p=({p_cmd[0]:+.3f},{p_cmd[1]:+.3f},{p_cmd[2]:+.3f}) "
                    f"v=({v_cmd[0]:+.3f},{v_cmd[1]:+.3f},{v_cmd[2]:+.3f})"
                )
                last_no_ros_print_s = wall_now
                last_no_ros_mode = mode
            return

        ros_msg = Float64MultiArray()
        ros_msg.data = cur_msg
        cmd_pub.publish(ros_msg)

    try:
        while should_keep_running():
            latest_ball_msg = None

            try:
                payload = socket.recv_pyobj()
                if payload.get("type") == 'robot':
                    current_robot_base_pos = np.array(payload['pos'], dtype=float) / 1000.0
                    current_robot_base_quat = np.array(payload['quat'], dtype=float)
                elif payload.get("type") == 'ball':
                    latest_ball_msg = payload
            except zmq.Again:
                pass

            while True:
                try:
                    extra_msg = socket.recv_pyobj(flags=zmq.NOBLOCK)
                    if extra_msg.get("type") == 'robot':
                        current_robot_base_pos = np.array(extra_msg['pos'], dtype=float) / 1000.0
                        current_robot_base_quat = np.array(extra_msg['quat'], dtype=float)
                    elif extra_msg.get("type") == 'ball':
                        latest_ball_msg = extra_msg
                except zmq.Again:
                    break

            wall_now = time.time()

            if latest_ball_msg is not None:
                current_time = latest_ball_msg['t']
                pos_m = np.array(latest_ball_msg['pos'], dtype=float) / 1000.0
                vel_m = np.array(latest_ball_msg['vel'], dtype=float) / 1000.0
                current_speed_x_m = vel_m[0]
                ball_diag_count += 1
                gate_reason = incoming_gate_reason(pos_m, vel_m)
                last_ball_diag = (pos_m.copy(), vel_m.copy(), gate_reason)

                if viz is not None and viz._rally_count > 0:
                    viz.update_real(pos_m)

                if gate_reason == "pass":

                    if not ball_incoming:
                        ball_incoming = True
                        has_crossed_target = False
                        pending_lock = False
                        publish_count = -1
                        prev_pos_m = None
                        prev_mocap_time = None
                        # 保留旧的 locked_* 指令，直到新预测成功覆盖，避免空窗期停发消息
                        print(f"\n[追踪中] 发现新来球，开始追踪...")

                        if viz is not None:
                            viz.new_rally(rally_count + 1)

                    if pos_m[0] > TARGET_HIT_X:
                        hit_pos, hit_vel, flight_time = predictor.predict_hitting_point(
                            pos_m, vel_m, target_x=TARGET_HIT_X
                        )

                        if pos_m[2] < 0.1:
                            hit_pos = None

                        if hit_pos is not None and flight_time <= LOCK_TIME_THRESHOLD:

                            locked_hit_pos = hit_pos.copy()
                            locked_hit_vel = hit_vel.copy()

                            # ---- 每帧更新预测绝对到达时刻 ----
                            locked_absolute_hit_time = current_time + flight_time

                            locked_v_racket_cmd = calculate_racket_command(
                                p_racket=locked_hit_pos,
                                v_incoming=locked_hit_vel,
                                p_landing=p_landing_target,
                                dt_flight=desired_flight_time,
                                Cr=0.70,
                                predictor=predictor,
                            )

                            locked_p_racket_relative = locked_hit_pos - ROBOT_BASE_ORIGIN_WORLD
                            locked_v_racket_cmd_tmp = locked_v_racket_cmd.copy()
                            locked_v_racket_cmd_tmp[0] += 0.6
                            locked_v_racket_cmd_relative = locked_v_racket_cmd_tmp.copy()

                            # 第一次有效预测之后，系统永久进入 pred 发布语义：
                            # - 新预测出现时，更新 locked_*；
                            # - 没有新预测时，沿用上一组 locked_*；
                            # - t_strike 从正数继续下降，最低夹到 DEFAULT_T_STRIKE=-0.9。
                            has_ever_predicted = True

                            if not pending_lock:
                                # 第一次锁定：记录 wall_time / mocap_time 用于无数据帧的时间估算
                                wall_time_at_lock = wall_now
                                mocap_time_at_lock = current_time
                                pending_lock = True
                                publish_count = 0
                                print(f"[追踪中] 首次锁定 | t_flight={flight_time:.3f}s | "
                                      f"击球点 Y={locked_hit_pos[1]:.3f} Z={locked_hit_pos[2]:.3f} | "
                                      f"球位 ({pos_m[0]:+.3f}, {pos_m[1]:+.3f}, {pos_m[2]:+.3f})")
                            else:
                                print(f"[追踪中] 时间更新 | t_flight={flight_time:.3f}s | "
                                      f"击球点 Y={locked_hit_pos[1]:.3f} Z={locked_hit_pos[2]:.3f} | "
                                      f"球位 ({pos_m[0]:+.3f}, {pos_m[1]:+.3f}, {pos_m[2]:+.3f})")

                            # ---- 记录当前帧：(t_strike, 预测绝对到达时刻) ----
                            cur_t_strike = locked_absolute_hit_time - current_time  # == flight_time
                            if viz is not None:
                                viz.record_prediction(cur_t_strike, locked_absolute_hit_time,
                                                      locked_hit_pos)

                            if viz is not None:
                                full_traj = predictor.compute_full_trajectory(
                                    pos_m, vel_m, target_x=TARGET_HIT_X
                                )
                                viz.update_predicted(full_traj, locked_hit_pos)
                                viz.set_status(
                                    f"Locked | t_flight={flight_time:.3f}s | "
                                    f"Hit Y={locked_hit_pos[1]:.3f} Z={locked_hit_pos[2]:.3f}"
                                )

                # ---- 检测球是否穿越 target_x，计算真实穿越时刻 ----
                if ball_incoming and not has_crossed_target and prev_pos_m is not None:
                    if prev_pos_m[0] >= TARGET_HIT_X and pos_m[0] < TARGET_HIT_X:
                        has_crossed_target = True
                        ball_incoming = False

                        # 线性插值得到真实穿越的空间位置
                        ratio = (TARGET_HIT_X - prev_pos_m[0]) / (pos_m[0] - prev_pos_m[0])
                        real_hit_y = prev_pos_m[1] + ratio * (pos_m[1] - prev_pos_m[1])
                        real_hit_z = prev_pos_m[2] + ratio * (pos_m[2] - prev_pos_m[2])

                        # 线性插值得到真实穿越的 mocap 时刻
                        if prev_mocap_time is not None:
                            real_hit_time = prev_mocap_time + ratio * (current_time - prev_mocap_time)
                        else:
                            real_hit_time = current_time  # fallback

                        # ---- 触发误差曲线更新（传入真实位置和到达时刻）----
                        if viz is not None:
                            viz.finalize_rally(real_hit_y, real_hit_z, real_hit_time)

                        print(f"[Ball #{rally_count}] 事后校验 | "
                              f"真实穿越 Y={real_hit_y:.4f} Z={real_hit_z:.4f} "
                              f"T={real_hit_time:.4f}s", end="")
                        if locked_hit_pos is not None and locked_absolute_hit_time is not None:
                            error_y = real_hit_y - locked_hit_pos[1]
                            error_z = real_hit_z - locked_hit_pos[2]
                            time_error_ms = (locked_absolute_hit_time - real_hit_time) * 1000
                            print(f" | 位置误差 {math.hypot(error_y, error_z) * 1000:.1f}mm"
                                  f" (dY={error_y * 1000:+.1f} dZ={error_z * 1000:+.1f})"
                                  f" | 时间误差 {time_error_ms:+.1f}ms")
                            if viz is not None:
                                viz.set_status(
                                    f"Hit | Time err={time_error_ms:+.1f}ms | "
                                    f"Pos err {math.hypot(error_y, error_z) * 1000:.1f}mm"
                                )
                        else:
                            print()

                if current_speed_x_m > 0.3 and pos_m[0] > -1.0:
                    if ball_incoming:
                        ball_incoming = False
                elif pos_m[0] < TARGET_HIT_X:
                    if ball_incoming:
                        ball_incoming = False

                prev_pos_m = pos_m.copy()
                prev_mocap_time = current_time   # 保存上一帧 mocap 时刻
                last_mocap_time = current_time

            maybe_print_ball_diag(wall_now)

            # ---------- 指令下发 ----------
            # 持续发布 14 维指令。
            # 语义：
            # - 首次有效预测出现之前：mode=default，发布默认球拍目标，t_strike=-0.9。
            # - 首次有效预测出现之后：mode 永远保持 pred。
            #   如果当前没有新预测，就沿用上一组 locked_* 指令，t_strike 按时间继续下降，
            #   最低夹到 DEFAULT_T_STRIKE=-0.9，直到下一次有效预测刷新 locked_*。
            use_predicted_command = False
            publish_t_strike = DEFAULT_T_STRIKE
            p_cmd_publish = DEFAULT_P_RACKET_RELATIVE.copy()
            v_cmd_publish = DEFAULT_V_RACKET_RELATIVE.copy()

            if has_ever_predicted and locked_hit_pos is not None and locked_absolute_hit_time is not None:
                use_predicted_command = True

                if latest_ball_msg is not None:
                    dynamic_t_strike = locked_absolute_hit_time - current_time
                else:
                    elapsed_wall = wall_now - wall_time_at_lock
                    estimated_mocap_time = mocap_time_at_lock + elapsed_wall
                    dynamic_t_strike = locked_absolute_hit_time - estimated_mocap_time

                # 新预测可能在 0.60~0.62s 内刚刚锁定；对外发布时上限仍夹到 0.6。
                # 过击球时刻后不切回 default，而是继续递减，最低夹到 -0.9。
                publish_t_strike = float(np.clip(dynamic_t_strike, DEFAULT_T_STRIKE, 0.6))

                if last_flag != locked_p_racket_relative[1]:
                    last_flag = locked_p_racket_relative[1]

                p_cmd_publish = np.array([
                    locked_p_racket_relative[0],
                    np.clip(locked_p_racket_relative[1], -0.69, 0.69),
                    np.clip(locked_p_racket_relative[2], 0.67, 1.15)
                ])

                v_cmd_publish = np.array([
                    np.clip(locked_v_racket_cmd_relative[0], 1.31, 1.99),
                    np.clip(locked_v_racket_cmd_relative[1], -0.09, 0.09),
                    np.clip(locked_v_racket_cmd_relative[2], 0.01, 0.29)
                ])

            cur_msg = [
                float(current_robot_base_pos[0]), float(current_robot_base_pos[1]), float(current_robot_base_pos[2]),
                float(current_robot_base_quat[0]), float(current_robot_base_quat[1]), float(current_robot_base_quat[2]), float(current_robot_base_quat[3]),
                float(publish_t_strike),
                float(p_cmd_publish[0]), float(p_cmd_publish[1]), float(p_cmd_publish[2]),
                float(v_cmd_publish[0]), float(v_cmd_publish[1]), float(v_cmd_publish[2])
            ]

            mode = 'pred' if use_predicted_command else 'default'
            publish_or_print(cur_msg, mode, publish_t_strike, p_cmd_publish, v_cmd_publish, wall_now)

            debug_log.write(
                f"{wall_now:.4f}  ball={rally_count}  "
                f"mode={mode}  "
                f"t_strike={publish_t_strike:.4f}  "
                f"base=[{cur_msg[0]:.3f},{cur_msg[1]:.3f},{cur_msg[2]:.3f}]  "
                f"quat=[{cur_msg[3]:.3f},{cur_msg[4]:.3f},{cur_msg[5]:.3f},{cur_msg[6]:.3f}]  "
                f"p=[{p_cmd_publish[0]:.3f},{p_cmd_publish[1]:.3f},{p_cmd_publish[2]:.3f}]  "
                f"v=[{v_cmd_publish[0]:.3f},{v_cmd_publish[1]:.3f},{v_cmd_publish[2]:.3f}]\n"
            )

            if use_predicted_command and publish_count >= 0:
                publish_count += 1
                if publish_count == 1:
                    rally_count += 1
                    print(f"\n[Ball #{rally_count}] 首次发布成功 | "
                          f"击球点 Y={locked_p_racket_relative[1]:.3f} "
                          f"Z={locked_p_racket_relative[2]:.3f}")

    except KeyboardInterrupt:
        print("\n\n[系统退出] 检测到中止信号 (Ctrl+C)，正在安全关闭。")
    finally:
        debug_log.close()
        if ros_node is not None:
            ros_node.destroy_node()
        if not NO_ROS and rclpy is not None:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
