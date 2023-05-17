import numpy as np

x1 = np.linspace(0.0, -5.0, 100)
y1 = 0.0

goal1 = np.vstack([x1, np.tile(y1, 100), np.tile(0.0, 100)])

x2 = -5.0
y2 = np.linspace(0.0, -5.5, 100)

goal2 = np.vstack([np.tile(x2, 100), y2, np.tile(0.0, 100)])


x3 = np.linspace(-5.0, -3.0, 100)
y3 = -5.5


goal3 = np.vstack([x3, np.tile(y3, 100), np.tile(0.0, 100)])

x4 = -3.0
y4 = np.linspace(-5.5, -3.5, 100)

goal4 = np.vstack([np.tile(x4, 100), y4, np.tile(0.0, 100)])

x5 = np.linspace(-3.0, 0.0, 100)
y5 = -3.5

goal5 = np.vstack([x5, np.tile(y5, 100), np.tile(0.0, 100)])


x6 = 0.0
y6 = np.linspace(-3.5, -4.0, 100)

goal6 = np.vstack([np.tile(x6, 100), y6, np.tile(0.0, 100)])

goal = np.hstack([goal1, goal2, goal3, goal4, goal5, goal6])


np.savetxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path6.csv', goal, delimiter=',')

print(goal)
