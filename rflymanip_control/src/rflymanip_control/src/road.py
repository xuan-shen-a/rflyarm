import numpy as np
import matplotlib.pyplot as plt

def plan_bulb_arc(x0, yc, zc, r, phi0_deg, phi1_deg, N=200):
    phi = np.deg2rad(np.linspace(phi0_deg, phi1_deg, N))

    x = np.full_like(phi, x0)
    y = yc + r * np.cos(phi)
    z = zc + r * np.sin(phi)

    return x, y, z, phi

# 示例参数
x0 = 420.0       # 灯轴方向位置
yc = 0.0         # 灯轴中心 y
zc = 180.0       # 灯轴中心 z
r  = 30.0        # 灯泡半径 mm

x, y, z, phi = plan_bulb_arc(x0, yc, zc, r, phi0_deg=90, phi1_deg=0, N=200)

fig = plt.figure(figsize=(10, 4))

ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(x, y, z, linewidth=2)
ax1.set_xlabel('X (mm)')
ax1.set_ylabel('Y (mm)')
ax1.set_zlabel('Z (mm)')
ax1.set_title('End-Effector Arc Trajectory')

ax2 = fig.add_subplot(122)
ax2.plot(y, z, linewidth=2)
ax2.set_aspect('equal')
ax2.set_xlabel('Y (mm)')
ax2.set_ylabel('Z (mm)')
ax2.set_title('YZ Section')
ax2.grid(True)

plt.tight_layout()
plt.show()