import glob
import os
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.ticker import MaxNLocator

# =========================
# Cấu hình hiển thị chung (Giảm kích thước chữ để lấy không gian vẽ đồ thị)
# =========================
plt.rcParams.update({
    'axes.titlesize': 9,      # Size chữ tiêu đề đồ thị
    'axes.labelsize': 8,      # Size chữ nhãn trục
    'xtick.labelsize': 7,     # Size chữ số liệu trục X
    'ytick.labelsize': 7,     # Size chữ số liệu trục Y
    'legend.fontsize': 6.5,   # Size chữ chú thích
})

# =========================
# Tìm & đọc file CSV
# =========================
script_dir = os.path.dirname(os.path.abspath(__file__))


def find_csv():
    search_dirs = [
        script_dir,
        os.path.join(script_dir, "data_record"),
        os.path.join(script_dir, "..", "data_record"),
    ]
    candidates = []
    for d in search_dirs:
        if os.path.isdir(d):
            candidates += glob.glob(os.path.join(d, "carla_data_local.csv"))
            candidates += glob.glob(os.path.join(d, "carla_data_*.csv"))
    candidates = sorted(set(candidates), key=os.path.getmtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(
            "Không tìm thấy file carla_data*.csv nào trong: " + ", ".join(search_dirs)
        )
    return candidates[0]


csv_path = "/home/huy/WebRobot/web/recorded_data/carla_data_20260708_162044.csv"

print(f"[ve_do_thi] Đang đọc dữ liệu từ: {csv_path}")
df = pd.read_csv(csv_path)

t = df["timestamp"].to_numpy()

x = df["x"].to_numpy()
y = df["y"].to_numpy()
yaw = df["yaw"].to_numpy()

v_total = df["v_total"].to_numpy()
v_lat = df["v_lat"].to_numpy()
v_long = df["v_long"].to_numpy()
w_z = df["w_z"].to_numpy()

a_lat = df["a_lat"].to_numpy()
a_long = df["a_long"].to_numpy()

throttle_cmd = df["throttle_cmd"].to_numpy()
brake_cmd = df["brake_cmd"].to_numpy()
steer_cmd = df["steer_cmd"].to_numpy()
gear_cmd = df["gear_cmd"].to_numpy()

throttle_real = df["throttle_real"].to_numpy()
brake_real = df["brake_real"].to_numpy()
steer_real = df["steer_real"].to_numpy()
gear_real = df["gear_real"].to_numpy()

is_idle = df["is_idle"].to_numpy()

N = len(t)

# =========================
# Màu & style dùng chung
# =========================
C_THROTTLE = "#2ca02c"
C_BRAKE = "#d62728"
C_STEER = "#1f77b4"
C_GEAR = "#9467bd"
C_YAW = "#1f77b4"
C_WZ = "#ff7f0e"
C_VTOTAL = "#333333"
C_VLONG = "#1f77b4"
C_VLAT = "#d62728"
C_ALONG = "#2ca02c"
C_ALAT = "#d62728"

LW_REAL, LS_REAL = 1.9, "-"
LW_CMD, LS_CMD = 1.3, "--"

MARKER_SIZE = 7
MIN_STOP_DURATION = 1.0

# =========================
# Tìm các đoạn xe dừng
# =========================


def find_stop_segments(idle_mask, times, min_duration):
    segments = []
    n = len(idle_mask)
    i = 0
    while i < n:
        if idle_mask[i] == 1:
            j = i
            while j < n and idle_mask[j] == 1:
                j += 1
            duration = times[j - 1] - times[i]
            if duration >= min_duration:
                segments.append((i, j - 1, duration))
            i = j
        else:
            i += 1
    return segments


stop_segments = find_stop_segments(is_idle, t, MIN_STOP_DURATION)

# =========================
# Figure & lưới (Tăng kích thước Figure)
# =========================
fig, axs = plt.subplots(4, 2, figsize=(17, 20))

ax_xy = axs[0, 0]
ax_vel = axs[0, 1]
ax_acc = axs[1, 0]
ax_yaw = axs[1, 1]
ax_throttle = axs[2, 0]
ax_brake = axs[2, 1]
ax_steer = axs[3, 0]
ax_gear = axs[3, 1]

# =========================
# Ô 1: Quỹ đạo di chuyển
# =========================
ax_xy.plot(x, y, color="#1f77b4", lw=1.8, zorder=2, label="Quỹ đạo")
ax_xy.plot(x[0], y[0], "o", ms=10, mfc="#2ca02c", mec="k", zorder=4, label="Điểm bắt đầu")
ax_xy.plot(x[-1], y[-1], "s", ms=9, mfc="#000000", mec="k", zorder=4, label="Điểm kết thúc")

if stop_segments:
    stop_x = [x[s] for s, e, d in stop_segments]
    stop_y = [y[s] for s, e, d in stop_segments]
    ax_xy.scatter(
        stop_x, stop_y, marker="X", s=140, color="#e85555", edgecolors="k",
        linewidths=0.8, zorder=5, label=f"Điểm dừng (≥{MIN_STOP_DURATION:.0f}s)",
    )
    if len(stop_segments) <= 15:
        for (s, e, d), sx, sy in zip(stop_segments, stop_x, stop_y):
            ax_xy.annotate(
                f"{d:.1f}s", (sx, sy), textcoords="offset points", xytext=(7, 7),
                fontsize=6.5, color="#e85555", fontweight="bold",
            )

ax_xy.set_title("Vehicle Trajectory")
ax_xy.set_xlabel("X (m)")
ax_xy.set_ylabel("Y (m)")
ax_xy.axis("equal")
ax_xy.grid(True)
ax_xy.legend(loc="best")

# =========================
# Ô 2: Vận tốc
# =========================
ax_vel.plot(t, v_total, color=C_VTOTAL, lw=2.0, label="v_total")
ax_vel.plot(t, v_long, color=C_VLONG, lw=1.3, label="v_long")
ax_vel.plot(t, v_lat, color=C_VLAT, lw=1.3, label="v_lat")
ax_vel.set_title("Velocity")
ax_vel.set_xlabel("Time (s)")
ax_vel.set_ylabel("Velocity (m/s)")
ax_vel.grid(True)
ax_vel.legend(loc="upper right")

# =========================
# Ô 3: Gia tốc
# =========================
ax_acc.plot(t, a_long, color=C_ALONG, lw=1.6, label="a_long")
ax_acc.plot(t, a_lat, color=C_ALAT, lw=1.6, label="a_lat")
ax_acc.set_title("Longitudinal / Lateral Acceleration")
ax_acc.set_xlabel("Time (s)")
ax_acc.set_ylabel("Acceleration (m/s²)")
ax_acc.grid(True)
ax_acc.legend(loc="upper right")

# =========================
# Ô 4: Yaw & Yaw rate
# =========================
line_yaw, = ax_yaw.plot(t, yaw, color=C_YAW, lw=1.6, label="yaw")
ax_yaw.set_title("Yaw & Yaw Rate")
ax_yaw.set_xlabel("Time (s)")
ax_yaw.set_ylabel("Yaw (rad)", color=C_YAW)
ax_yaw.tick_params(axis="y", labelcolor=C_YAW)
ax_yaw.grid(True)

ax_wz = ax_yaw.twinx()
line_wz, = ax_wz.plot(t, w_z, color=C_WZ, lw=1.3, label="w_z")
ax_wz.set_ylabel("Yaw rate (rad/s)", color=C_WZ)
ax_wz.tick_params(axis="y", labelcolor=C_WZ)

_combo_lines = [line_yaw, line_wz]
ax_yaw.legend(_combo_lines, [l.get_label() for l in _combo_lines], loc="upper right")

# =========================
# Ô 5-8: Throttle / Brake / Steer / Gear
# =========================
ax_throttle.plot(t, throttle_real, color=C_THROTTLE, lw=LW_REAL, ls=LS_REAL, label="throttle_real")
ax_throttle.plot(t, throttle_cmd, color=C_THROTTLE, lw=LW_CMD, ls=LS_CMD, label="throttle_cmd")
ax_throttle.set_title("Throttle (Ga)")
ax_throttle.set_xlabel("Time (s)")
ax_throttle.set_ylabel("Throttle [0–1]")
ax_throttle.set_ylim(-0.05, 1.05)
ax_throttle.grid(True)
ax_throttle.legend(loc="upper right")

ax_brake.plot(t, brake_real, color=C_BRAKE, lw=LW_REAL, ls=LS_REAL, label="brake_real")
ax_brake.plot(t, brake_cmd, color=C_BRAKE, lw=LW_CMD, ls=LS_CMD, label="brake_cmd")
ax_brake.set_title("Brake (Phanh)")
ax_brake.set_xlabel("Time (s)")
ax_brake.set_ylabel("Brake [0–1]")
ax_brake.set_ylim(-0.05, 1.05)
ax_brake.grid(True)
ax_brake.legend(loc="upper right")

ax_steer.plot(t, steer_real, color=C_STEER, lw=LW_REAL, ls=LS_REAL, label="steer_real")
ax_steer.plot(t, steer_cmd, color=C_STEER, lw=LW_CMD, ls=LS_CMD, label="steer_cmd")
ax_steer.set_title("Steering (Đánh lái)")
ax_steer.set_xlabel("Time (s)")
ax_steer.set_ylabel("Steer [-1..1]")
ax_steer.grid(True)
ax_steer.legend(loc="upper right")

ax_gear.plot(t, gear_real, color=C_GEAR, lw=LW_REAL, ls=LS_REAL, drawstyle="steps-post", label="gear_real")
ax_gear.plot(t, gear_cmd, color=C_GEAR, lw=LW_CMD, ls=LS_CMD, drawstyle="steps-post", label="gear_cmd")
ax_gear.set_title("Gear (Số)")
ax_gear.set_xlabel("Time (s)")
ax_gear.set_ylabel("Gear")
ax_gear.yaxis.set_major_locator(MaxNLocator(integer=True))
ax_gear.grid(True)
ax_gear.legend(loc="upper right")

# ======================================================
# Marker + crosshair cho từng ô
# ======================================================

traj_marker, = ax_xy.plot([], [], "o", ms=9, color="red", mec="k", zorder=6)
traj_vline = ax_xy.axvline(x[0], color="r", linestyle="--", lw=1)
traj_hline = ax_xy.axhline(y[0], color="r", linestyle="--", lw=1)


def _make_time_plot(ax, series_list):
    vline = ax.axvline(t[0], color="gray", linestyle="--", lw=1)
    markers = []
    for data, color in series_list:
        m, = ax.plot([], [], "o", ms=MARKER_SIZE, color=color, mec="k", mew=0.6, zorder=6)
        markers.append(m)
    return {"ax": ax, "vline": vline, "series": series_list, "markers": markers}


TIME_PLOTS = [
    _make_time_plot(ax_vel, [(v_total, C_VTOTAL), (v_long, C_VLONG), (v_lat, C_VLAT)]),
    _make_time_plot(ax_acc, [(a_long, C_ALONG), (a_lat, C_ALAT)]),
    _make_time_plot(ax_throttle, [(throttle_real, C_THROTTLE), (throttle_cmd, C_THROTTLE)]),
    _make_time_plot(ax_brake, [(brake_real, C_BRAKE), (brake_cmd, C_BRAKE)]),
    _make_time_plot(ax_steer, [(steer_real, C_STEER), (steer_cmd, C_STEER)]),
    _make_time_plot(ax_gear, [(gear_real, C_GEAR), (gear_cmd, C_GEAR)]),
]

yaw_vline = ax_yaw.axvline(t[0], color="gray", linestyle="--", lw=1)
yaw_marker, = ax_yaw.plot([], [], "o", ms=MARKER_SIZE, color=C_YAW, mec="k", mew=0.6, zorder=6)
wz_marker, = ax_wz.plot([], [], "o", ms=MARKER_SIZE, color=C_WZ, mec="k", mew=0.6, zorder=6)

TIME_BASED_AXES = (ax_vel, ax_acc, ax_yaw, ax_wz, ax_throttle, ax_brake, ax_steer, ax_gear)

# Giảm kích thước chữ của thanh hiển thị thông tin (suptitle)
info_text = fig.suptitle("", fontsize=8.5, fontweight="bold", y=0.985, linespacing=1.5)

# ======================================================
# Trạng thái frame hiện tại
# ======================================================
current = [0]

# ======================================================
# Hàm update trung tâm
# ======================================================


def update(idx, move_slider=True):
    idx = int(np.clip(idx, 0, N - 1))
    current[0] = idx

    traj_marker.set_data([x[idx]], [y[idx]])
    traj_vline.set_xdata([x[idx], x[idx]])
    traj_hline.set_ydata([y[idx], y[idx]])

    for tp in TIME_PLOTS:
        tp["vline"].set_xdata([t[idx], t[idx]])
        for marker, (data, _color) in zip(tp["markers"], tp["series"]):
            marker.set_data([t[idx]], [data[idx]])

    yaw_vline.set_xdata([t[idx], t[idx]])
    yaw_marker.set_data([t[idx]], [yaw[idx]])
    wz_marker.set_data([t[idx]], [w_z[idx]])

    info_text.set_text(
        f"Frame {idx}/{N - 1}   |   t = {t[idx]:.3f} s   |   "
        f"X = {x[idx]:.2f} m   Y = {y[idx]:.2f} m   yaw = {yaw[idx]:.3f} rad\n"
        f"V_total = {v_total[idx]:.3f}   V_long = {v_long[idx]:.3f}   V_lat = {v_lat[idx]:.3f} m/s   |   "
        f"a_long = {a_long[idx]:.3f}   a_lat = {a_lat[idx]:.3f} m/s²   |   w_z = {w_z[idx]:.3f} rad/s\n"
        f"Throttle {throttle_real[idx]:.2f}/{throttle_cmd[idx]:.2f}   "
        f"Brake {brake_real[idx]:.2f}/{brake_cmd[idx]:.2f}   "
        f"Steer {steer_real[idx]:.2f}/{steer_cmd[idx]:.2f}   "
        f"Gear {gear_real[idx]}/{gear_cmd[idx]}   (thực tế/yêu cầu)"
    )

    if move_slider and abs(slider.val - idx) > 1e-9:
        slider.eventson = False
        slider.set_val(idx)
        slider.eventson = True

    fig.canvas.draw_idle()


# ======================================================
# Click event
# ======================================================


def onclick(event):
    if event.inaxes is ax_xy:
        if event.xdata is None or event.ydata is None:
            return
        dist = np.hypot(x - event.xdata, y - event.ydata)
        idx = int(np.argmin(dist))
    elif event.inaxes in TIME_BASED_AXES:
        if event.xdata is None:
            return
        idx = int(np.argmin(np.abs(t - event.xdata)))
    else:
        return

    update(idx)

    print("------------------------------------------")
    print(f"Index      : {idx}")
    print(f"Time       : {t[idx]:.3f} s")
    print(f"X, Y, yaw  : {x[idx]:.3f} m, {y[idx]:.3f} m, {yaw[idx]:.3f} rad")
    print(f"V total/long/lat : {v_total[idx]:.3f} / {v_long[idx]:.3f} / {v_lat[idx]:.3f} m/s")
    print(f"a_long/a_lat     : {a_long[idx]:.3f} / {a_lat[idx]:.3f} m/s²   w_z: {w_z[idx]:.3f} rad/s")
    print(f"Throttle (real/cmd): {throttle_real[idx]:.3f} / {throttle_cmd[idx]:.3f}")
    print(f"Brake    (real/cmd): {brake_real[idx]:.3f} / {brake_cmd[idx]:.3f}")
    print(f"Steer    (real/cmd): {steer_real[idx]:.3f} / {steer_cmd[idx]:.3f}")
    print(f"Gear     (real/cmd): {gear_real[idx]} / {gear_cmd[idx]}")


fig.canvas.mpl_connect("button_press_event", onclick)

# ======================================================
# Key event
# ======================================================


def on_key(event):
    idx = current[0]
    if event.key == "right":
        idx += 1
    elif event.key == "left":
        idx -= 1
    elif event.key == "up":
        idx += 10
    elif event.key == "down":
        idx -= 10
    else:
        return
    stop_play()
    update(idx)


fig.canvas.mpl_connect("key_press_event", on_key)

# ======================================================
# Layout: Tối ưu hóa khoảng trắng (Tăng diện tích cho đồ thị)
# ======================================================

# Tăng top lên 0.95, giảm bottom xuống 0.07, thu hẹp hspace và wspace
plt.subplots_adjust(top=0.95, bottom=0.07, left=0.05, right=0.97, hspace=0.32, wspace=0.18)

ax_slider = fig.add_axes([0.10, 0.048, 0.58, 0.016])
slider = Slider(ax_slider, "Frame", 0, N - 1, valinit=0, valstep=1)


def on_slider_change(val):
    stop_play()
    update(int(val), move_slider=False)


slider.on_changed(on_slider_change)

# ======================================================
# Nút Play / Pause
# ======================================================

ax_play = fig.add_axes([0.10, 0.008, 0.07, 0.028])
ax_pause = fig.add_axes([0.185, 0.008, 0.07, 0.028])
btn_play = Button(ax_play, "Play")
btn_pause = Button(ax_pause, "Pause")

play_state = {"active": False, "start_wall": 0.0, "start_idx": 0}


def play_tick():
    if not play_state["active"]:
        return
    elapsed = time.time() - play_state["start_wall"]
    target_t = t[play_state["start_idx"]] + elapsed
    idx = int(np.searchsorted(t, target_t, side="right") - 1)
    idx = int(np.clip(idx, 0, N - 1))
    update(idx)
    if idx >= N - 1:
        stop_play()


def start_play(event=None):
    if play_state["active"]:
        return
    play_state["active"] = True
    play_state["start_wall"] = time.time()
    play_state["start_idx"] = current[0]
    timer.start()


def stop_play(event=None):
    play_state["active"] = False
    timer.stop()


btn_play.on_clicked(start_play)
btn_pause.on_clicked(stop_play)

timer = fig.canvas.new_timer(interval=25)
timer.add_callback(play_tick)

# ======================================================
# Nút Xuất lệnh điều khiển
# ======================================================

ax_export = fig.add_axes([0.83, 0.008, 0.15, 0.05])
btn_export = Button(
    ax_export, "Xuất lệnh điều khiển\nmô hình xe",
    color="#dddddd", hovercolor="#dddddd",
)
btn_export.label.set_fontsize(8.5)
btn_export.label.set_color("#888888")
btn_export.ax.set_alpha(0.6)


def export_control_commands(event=None):
    pass


btn_export.on_clicked(export_control_commands)
try:
    btn_export.set_active(False)
except Exception:
    pass

# ======================================================
# Khởi tạo hiển thị
# ======================================================

update(0)

plt.show()