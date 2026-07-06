import os
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# =========================
# Đọc file CSV
# =========================
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "carla_data_local.csv")

df = pd.read_csv(csv_path)

t = df["timestamp"].to_numpy()

x = df["x"].to_numpy()
y = df["y"].to_numpy()

a_long = df["a_long"].to_numpy()
v_long = df["v_long"].to_numpy()
brake = df["brake_real"].to_numpy()

N = len(t)

# =========================
# Tạo figure
# =========================
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

ax_xy = axs[0, 0]
ax_acc = axs[0, 1]
ax_v = axs[1, 0]
ax_brake = axs[1, 1]

# =========================
# Trajectory
# =========================
ax_xy.plot(x, y, 'b', lw=2)

ax_xy.set_title("Vehicle Trajectory")
ax_xy.set_xlabel("X (m)")
ax_xy.set_ylabel("Y (m)")
ax_xy.axis("equal")
ax_xy.grid(True)

# =========================
# Longitudinal Acceleration
# =========================
ax_acc.plot(t, a_long, 'g')

ax_acc.set_title("Longitudinal Acceleration")
ax_acc.set_xlabel("Time (s)")
ax_acc.set_ylabel("Acceleration (m/s²)")
ax_acc.grid(True)

# =========================
# Longitudinal Velocity
# =========================
ax_v.plot(t, v_long, 'r')

ax_v.set_title("Longitudinal Velocity")
ax_v.set_xlabel("Time (s)")
ax_v.set_ylabel("Velocity (m/s)")
ax_v.grid(True)

# =========================
# Brake
# =========================
ax_brake.plot(t, brake, 'm')

ax_brake.set_title("Brake")
ax_brake.set_xlabel("Time (s)")
ax_brake.set_ylabel("Brake")
ax_brake.set_ylim(-0.05, 1.05)
ax_brake.grid(True)

# ======================================================
# Marker + crosshair (gióng giá trị lên cả 2 trục) cho cả 4 ô
# ======================================================

traj_marker, = ax_xy.plot([], [], 'ro', ms=8, zorder=5)
acc_marker, = ax_acc.plot([], [], 'ro', ms=8, zorder=5)
v_marker, = ax_v.plot([], [], 'ro', ms=8, zorder=5)
brake_marker, = ax_brake.plot([], [], 'ro', ms=8, zorder=5)

traj_vline = ax_xy.axvline(x[0], color='r', linestyle='--', lw=1)
traj_hline = ax_xy.axhline(y[0], color='r', linestyle='--', lw=1)

acc_vline = ax_acc.axvline(t[0], color='r', linestyle='--', lw=1)
acc_hline = ax_acc.axhline(a_long[0], color='r', linestyle='--', lw=1)

v_vline = ax_v.axvline(t[0], color='r', linestyle='--', lw=1)
v_hline = ax_v.axhline(v_long[0], color='r', linestyle='--', lw=1)

brake_vline = ax_brake.axvline(t[0], color='r', linestyle='--', lw=1)
brake_hline = ax_brake.axhline(brake[0], color='r', linestyle='--', lw=1)

# Text hiển thị mốc thời gian + các giá trị tại frame hiện tại (luôn cập nhật,
# bất kể điều hướng bằng click / mũi tên / slider / play)
info_text = fig.suptitle("", fontsize=12, fontweight="bold")

# ======================================================
# Trạng thái frame hiện tại (dùng list 1 phần tử để sửa được trong nested function)
# ======================================================
current = [0]

# ======================================================
# Hàm update trung tâm: mọi cách điều hướng (click, mũi tên, slider, play)
# đều gọi qua đây để đồng bộ toàn bộ marker/crosshair/slider/text
# ======================================================


def update(idx, move_slider=True):
    idx = int(np.clip(idx, 0, N - 1))
    current[0] = idx

    traj_marker.set_data([x[idx]], [y[idx]])
    traj_vline.set_xdata([x[idx], x[idx]])
    traj_hline.set_ydata([y[idx], y[idx]])

    acc_marker.set_data([t[idx]], [a_long[idx]])
    acc_vline.set_xdata([t[idx], t[idx]])
    acc_hline.set_ydata([a_long[idx], a_long[idx]])

    v_marker.set_data([t[idx]], [v_long[idx]])
    v_vline.set_xdata([t[idx], t[idx]])
    v_hline.set_ydata([v_long[idx], v_long[idx]])

    brake_marker.set_data([t[idx]], [brake[idx]])
    brake_vline.set_xdata([t[idx], t[idx]])
    brake_hline.set_ydata([brake[idx], brake[idx]])

    info_text.set_text(
        f"Frame {idx}/{N - 1}   |   t = {t[idx]:.3f} s   |   "
        f"X = {x[idx]:.2f} m   Y = {y[idx]:.2f} m   |   "
        f"V_long = {v_long[idx]:.3f} m/s   a_long = {a_long[idx]:.3f} m/s²   "
        f"Brake = {brake[idx]:.2f}"
    )

    if move_slider and abs(slider.val - idx) > 1e-9:
        slider.eventson = False
        slider.set_val(idx)
        slider.eventson = True

    fig.canvas.draw_idle()


# ======================================================
# Click: nhấn vào bất kỳ ô nào trong 4 ô đều nhảy tới frame gần nhất
# (ô Trajectory: gần nhất theo khoảng cách X-Y; 3 ô còn lại: gần nhất theo thời gian)
# ======================================================


def onclick(event):
    if event.inaxes is ax_xy:
        if event.xdata is None or event.ydata is None:
            return
        dist = np.hypot(x - event.xdata, y - event.ydata)
        idx = int(np.argmin(dist))
    elif event.inaxes in (ax_acc, ax_v, ax_brake):
        if event.xdata is None:
            return
        idx = int(np.argmin(np.abs(t - event.xdata)))
    else:
        return

    update(idx)

    print("------------------------------------------")
    print(f"Index      : {idx}")
    print(f"Time       : {t[idx]:.3f} s")
    print(f"X          : {x[idx]:.3f} m")
    print(f"Y          : {y[idx]:.3f} m")
    print(f"V_long     : {v_long[idx]:.3f} m/s")
    print(f"a_long     : {a_long[idx]:.3f} m/s²")
    print(f"Brake      : {brake[idx]:.3f}")


fig.canvas.mpl_connect("button_press_event", onclick)

# ======================================================
# Phím mũi tên: Trái/Phải = từng frame, Lên/Xuống = nhảy nhanh 10 frame
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
# Slider kéo để chọn frame
# ======================================================

plt.subplots_adjust(bottom=0.20)

ax_slider = fig.add_axes([0.15, 0.09, 0.7, 0.03])
slider = Slider(ax_slider, "Frame", 0, N - 1, valinit=0, valstep=1)


def on_slider_change(val):
    # Nếu người dùng đang tự kéo slider thì dừng Play để tránh xung đột vị trí
    # (khi update() tự đặt slider trong lúc play, slider.eventson đã bị tắt nên
    # callback này chỉ chạy khi thực sự là người dùng kéo tay)
    stop_play()
    update(int(val), move_slider=False)


slider.on_changed(on_slider_change)

# ======================================================
# Nút Play / Pause - phát theo đúng thời gian thật của cột timestamp
# ======================================================

ax_play = fig.add_axes([0.15, 0.02, 0.1, 0.045])
ax_pause = fig.add_axes([0.27, 0.02, 0.1, 0.045])
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

# tick nhanh (25ms) để bám sát thời gian thật; tốc độ phát thật sự do chênh lệch
# timestamp quyết định, không phải do interval này
timer = fig.canvas.new_timer(interval=25)
timer.add_callback(play_tick)

# ======================================================
# Khởi tạo hiển thị ở frame đầu tiên rồi mới show
# ======================================================

update(0)

plt.show()