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

a_long = df["a_long"].to_numpy()
v_long = df["v_long"].to_numpy()
brake = df["brake_real"].to_numpy()

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
# Marker
# ======================================================

traj_marker, = ax_xy.plot([], [], 'ro', ms=8)

acc_marker, = ax_acc.plot([], [], 'ro', ms=8)
v_marker, = ax_v.plot([], [], 'ro', ms=8)
brake_marker, = ax_brake.plot([], [], 'ro', ms=8)

acc_line = ax_acc.axvline(0, color='r', linestyle='--')
v_line = ax_v.axvline(0, color='r', linestyle='--')
brake_line = ax_brake.axvline(0, color='r', linestyle='--')


# ======================================================
# Click event
# ======================================================

def onclick(event):

    if event.inaxes != ax_xy:
        return

    click_x = event.xdata
    click_y = event.ydata

    dist = np.hypot(x - click_x, y - click_y)
    idx = np.argmin(dist)

    # Marker trên quỹ đạo
    traj_marker.set_data([x[idx]], [y[idx]])

    # Marker trên các đồ thị
    acc_marker.set_data([t[idx]], [a_long[idx]])
    v_marker.set_data([t[idx]], [v_long[idx]])
    brake_marker.set_data([t[idx]], [brake[idx]])

    # Đường thời gian
    acc_line.set_xdata([t[idx], t[idx]])
    v_line.set_xdata([t[idx], t[idx]])
    brake_line.set_xdata([t[idx], t[idx]])

    print("------------------------------------------")
    print(f"Index      : {idx}")
    print(f"Time       : {t[idx]:.3f} s")
    print(f"X          : {x[idx]:.3f} m")
    print(f"Y          : {y[idx]:.3f} m")
    print(f"V_long     : {v_long[idx]:.3f} m/s")
    print(f"a_long     : {a_long[idx]:.3f} m/s²")
    print(f"Brake      : {brake[idx]:.3f}")

    fig.canvas.draw_idle()


fig.canvas.mpl_connect("button_press_event", onclick)

plt.tight_layout()
plt.show()