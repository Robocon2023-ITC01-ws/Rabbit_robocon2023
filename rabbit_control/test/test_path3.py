import numpy as np
import matplotlib.pyplot as plt
from bezier_path import calc_4points_bezier_path, calc_bezier_path


def calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset, n_points):

    dist = np.hypot(sx - ex, sy - ey) / offset
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])

    path = calc_bezier_path(control_points, n_points)

    return path, control_points


path1, _ = calc_4points_bezier_path(
    5.2, -5.5, 2.99,
    0.0, 0.0, 1.57,
    1.1, 100
)

x1 = path1[:, 0]    
y1 = path1[:, 1]
yaw1 = np.append(np.arctan2(np.diff(y1), np.diff(x1)), 1.57)

goal1 = np.vstack([x1, y1, yaw1])

path2, _ = calc_4points_bezier_path(
    5.2, -5.5, 0,
    5.2,  -3.5, 3.10,
    5.0, 15
)

x2 = path2[:, 0]
y2 = path2[:, 1]
yaw2 = np.append(np.arctan2(np.diff(y2), np.diff(x2)), 3.14)

goal2 = np.hstack([x2, y2, yaw2])

path3, _ = calc_4points_bezier_path(
    5.2,  -3.5, 3.10,
    3.5, -3.5, 1.57,
    10.0, 20
)

x3 = path3[:, 0]
y3 = path3[:, 1]

yaw3 = np.append(np.arctan2(np.diff(y3), np.diff(x3)), 4.71)

goal3 = np.hstack([x3, y3, yaw3])

path4, _ = calc_4points_bezier_path(
    3.5, -3.5, 1.57,
    3.5,  0.0, 3.14,
    -10.0, 20
)

x4 = path4[:, 0]
y4 = path4[:, 1]
yaw4 = np.append(np.arctan2(np.diff(y4), np.diff(x4)), 3.14)

goal4 = np.hstack([x4, y4, yaw4])



np.savetxt('/home/kenotic/ros2ocp_ws/src/rabbit_control/rabbit_control/path2.csv', goal1, delimiter=",")
#goal1 = np.loadtxt('/home/kenotic/ros2ocp_ws/src/rabbit_control/rabbit_control/path1.csv', delimiter=',', dtype=np.float32)

print(goal1.shape)
x1 = goal1[0, :]
y1 = goal1[1, :]

plt.figure()
plt.plot(x1, y1, label="Goal1")
#plt.plot(x2, y2, label="Goal2")
#plt.plot(x3, y3, label="Goal3")
#plt.plot(x4, y4, label="Goal4")
# plt.xlim(-1, 6)
# plt.ylim(0, 12)
plt.legend()
plt.grid(True)
plt.show()
