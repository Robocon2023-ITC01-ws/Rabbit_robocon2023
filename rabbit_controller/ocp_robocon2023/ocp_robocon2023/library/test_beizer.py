import numpy as np
import matplotlib.pyplot as plt


from bezier_path import calc_4points_bezier_path

start_x1 = 0.0
start_y1 = 0.0
start_yaw1 = -1.57

end_x1 = 5.0
end_y1 = -5.0
end_yaw1 = 0.0
offset1 = 1.2

path1, _ = calc_4points_bezier_path(
    start_x1, start_y1, start_yaw1,
    end_x1, end_y1, end_yaw1,
    offset1
)

path_x1 = path1[:, 0]
path_y1 = path1[:, 1]
path_yaw1 = np.append(np.arctan2(np.diff(path_y1), np.diff(path_x1)), end_yaw1)

goal1 = np.vstack([path_x1, path_y1, np.tile(0.0, 100)])

start_x2 = 5.5
start_y2 = -5.0
start_yaw2 = 0.0

end_x2 = 5.5
end_y2 = -3.0
end_yaw2 = 0.0
offset2 = 20.0

path2, _ = calc_4points_bezier_path(
    start_x2, start_y2, start_yaw2,
    end_x2, end_y2, end_yaw2,
    offset2
)

path_x2 = path2[:, 0]
path_y2 = path2[:, 1]
path_yaw2 = np.append(np.arctan2(np.diff(path_y2), np.diff(path_x2)),  end_yaw2)

goal2 = np.vstack([path_x2, path_y2, np.tile(0.0, 100)])

start_x3 = 5.5
start_y3 = -3.0
start_yaw3 = 0.0

end_x3 = 3.5
end_y3 = -3.0
end_yaw3 = 0.0
offset3 = 20.0

path3, _ = calc_4points_bezier_path(
    start_x3, start_y3, start_yaw3,
    end_x3, end_y3, end_yaw3,
    offset3
)

path_x3 = path3[:, 0]
path_y3 = path3[:, 1]
path_yaw3 = np.append(np.arctan2(np.diff(path_y3), np.diff(path_x3)), end_yaw3)

goal3 = np.vstack([path_x3, path_y3, np.tile(0.0, 100)])

start_x4 = 3.5
start_y4 = -3.0
start_yaw4 = 0.0

end_x4 = 3.5
end_y4 = 0.0
end_yaw4 = 0.0
offset4 = 20.0

path4, _ = calc_4points_bezier_path(
    start_x4, start_y4, start_yaw4,
    end_x4, end_y4, end_yaw4,
    offset4
)

path_x4 = path4[:, 0]
path_y4 = path4[:, 1]
path_yaw4 = np.append(np.arctan2(np.diff(path_y4), np.diff(path_x4)), end_yaw4)

goal4 = np.vstack([path_x4, path_y4, np.tile(0.0, 100)])


start_x5 = 3.5
start_y5 = 0.0
start_yaw5 = 0.0

end_x5 = 4.5
end_y5 = 0.0
end_yaw5 = 0.0
offset5 = 20.0

path5, _ = calc_4points_bezier_path(
    start_x5, start_y5, start_yaw5,
    end_x5, end_y5, end_yaw5,
    offset5
)

path_x5 = path5[:, 0]
path_y5 = path5[:, 1]
path_yaw5 = np.append(np.arctan2(np.diff(path_y4), np.diff(path_x4)), end_yaw4)

goal5 = np.vstack([path_x5, path_y5, np.tile(0.0, 100)])



path6, _ = calc_4points_bezier_path(
    4.5, 0.0, 0.0,
    3.5, 0.0, 0.0,
    20.0
)

path_x6 = path6[:, 0]
path_y6 = path6[:, 1]
path_yaw1 = np.append(np.arctan2(np.diff(path_y1), np.diff(path_x1)), end_yaw1)

goal6 = np.vstack([path_x6, path_y6, np.tile(0.0, 100)])


path7, _ = calc_4points_bezier_path(
    3.5, 0.0, 0.0,
    3.5, -3.0, 0.0,
    50.0
)

path_x7 = path7[:, 0]
path_y7 = path7[:, 1]
path_yaw2 = np.append(np.arctan2(np.diff(path_y7), np.diff(path_x7)),  end_yaw2)

goal7 = np.vstack([path_x7, path_y7, np.tile(0.0, 100)])


path8, _ = calc_4points_bezier_path(
    3.5, -3.0, 0.0,
    5.5, -3.0, 0.0,
    20
)

path_x8 = path8[:, 0]
path_y8 = path8[:, 1]
path_yaw3 = np.append(np.arctan2(np.diff(path_y3), np.diff(path_x3)), end_yaw3)

goal8 = np.vstack([path_x8, path_y8, np.tile(0.0, 100)])


path9, _ = calc_4points_bezier_path(
    5.5, -3.0, 0.0,
    5.5, -5.0, 0.0,
    50.0
)

path_x9 = path9[:, 0]
path_y9 = path9[:, 1]
path_yaw4 = np.append(np.arctan2(np.diff(path_y4), np.diff(path_x4)), end_yaw4)

goal9 = np.vstack([path_x9, path_y9, np.tile(0.0, 100)])



path10, _ = calc_4points_bezier_path(
    5.5, -5.0, 3.15,
    0.0, 0.0, 1.57,
    1.2
)

path_x10 = path10[:, 0]
path_y10 = path10[:, 1]
path_yaw5 = np.append(np.arctan2(np.diff(path_y4), np.diff(path_x4)), end_yaw4)

goal10 = np.vstack([path_x10, path_y10, np.tile(0.0, 100)])

goal = np.hstack([goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8, goal9, goal10])




# goal = np.round(goal, 3)

# 
# np.savetxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path5.csv', goal, delimiter=",")

# goal = np.loadtxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path5.csv', delimiter=',', dtype=np.float32)


print(goal)

# print(path[1, :].shape)
# plt.plot(path_x1, path_y1, color="red", label="Path 1")
# plt.plot(path_x2, path_y2, color="red", label="Path 2")
# plt.plot(path_x3, path_y3, color="red", label="Path 3")
# plt.plot(path_x4, path_y4, color="red", label="Path 4")
plt.plot(goal[0, :], goal[1, :], label="Path")
plt.legend()
plt.grid(True)
plt.show()

# print(goal[1, :])
