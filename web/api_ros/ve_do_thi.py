import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================
# Đọc file CSV
# =========================
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "carla_data_local.csv")

df = pd.read_csv(csv_path)

t = df["timestamp"].to_numpy()
x = df["x"].to_numpy()
y = df["y"].to_numpy()
yaw = df["yaw"].to_numpy()
vx = df["v_long"].to_numpy()
vy = df["v_lat"].to_numpy()

# =========================
# Tạo figure
# =========================
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

ax_xy = axs[0, 0]
ax_yaw = axs[0, 1]
ax_vx = axs[1, 0]
ax_vy = axs[1, 1]

# =========================
# Vẽ dữ liệu
# =========================
ax_xy.plot(x, y, lw=2)
ax_xy.set_title("Trajectory")
ax_xy.set_xlabel("X (m)")
ax_xy.set_ylabel("Y (m)")
ax_xy.axis("equal")
ax_xy.grid(True)

ax_yaw.plot(t, yaw)
ax_yaw.set_title("Yaw")
ax_yaw.set_xlabel("Time (s)")
ax_yaw.grid(True)

ax_vx.plot(t, vx)
ax_vx.set_title("Longitudinal Velocity")
ax_vx.set_xlabel("Time (s)")
ax_vx.grid(True)

ax_vy.plot(t, vy)
ax_vy.set_title("Lateral Velocity")
ax_vy.set_xlabel("Time (s)")
ax_vy.grid(True)

# =========================
# Marker
# =========================
traj_marker, = ax_xy.plot([], [], 'ro', ms=8)

yaw_marker, = ax_yaw.plot([], [], 'ro', ms=8)
vx_marker, = ax_vx.plot([], [], 'ro', ms=8)
vy_marker, = ax_vy.plot([], [], 'ro', ms=8)

yaw_line = ax_yaw.axvline(0, color='r', ls='--')
vx_line = ax_vx.axvline(0, color='r', ls='--')
vy_line = ax_vy.axvline(0, color='r', ls='--')

# =========================
# Hàm click
# =========================
def onclick(event):

    if event.inaxes != ax_xy:
        return

    click_x = event.xdata
    click_y = event.ydata

    # tìm điểm gần nhất
    dist = np.sqrt((x - click_x)**2 + (y - click_y)**2)
    idx = np.argmin(dist)

    # cập nhật marker
    traj_marker.set_data([x[idx]], [y[idx]])

    yaw_marker.set_data([t[idx]], [yaw[idx]])
    vx_marker.set_data([t[idx]], [vx[idx]])
    vy_marker.set_data([t[idx]], [vy[idx]])

    yaw_line.set_xdata([t[idx], t[idx]])
    vx_line.set_xdata([t[idx], t[idx]])
    vy_line.set_xdata([t[idx], t[idx]])

    print("--------------------------------")
    print(f"Index      : {idx}")
    print(f"Timestamp  : {t[idx]:.3f}")
    print(f"X          : {x[idx]:.3f}")
    print(f"Y          : {y[idx]:.3f}")
    print(f"Yaw        : {yaw[idx]:.3f}")
    print(f"Vx         : {vx[idx]:.3f}")
    print(f"Vy         : {vy[idx]:.3f}")

    fig.canvas.draw_idle()

fig.canvas.mpl_connect('button_press_event', onclick)

plt.tight_layout()
plt.show()