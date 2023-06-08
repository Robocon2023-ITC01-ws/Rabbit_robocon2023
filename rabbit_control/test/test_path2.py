import numpy as np

x1 = np.linspace(0.0, -2.0, 100)
y1 = 0.0

goal1 = np.vstack([x1, np.tile(y1, 100), np.tile(0.0, 100)])

x2 = -2.0
y2 = np.linspace(0.0, -2.0, 100)

goal2 = np.vstack([np.tile(x2, 100), y2, np.tile(0.0, 100)])

goal = np.hstack([goal1, goal2])

np.savetxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path7.csv', goal, delimiter=',')
