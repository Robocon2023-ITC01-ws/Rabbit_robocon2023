import numpy as np
import matplotlib.pyplot as plt


from bezier_path import calc_4points_bezier_path

start_x1 = 0.0
start_y1 = 0.0
start_yaw1 = -1.57

end_x1 = 5.5
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

goal1 = np.vstack([path_x1, path_y1, path_yaw1])

start_x2 = 5.5
start_y2 = -5.0
start_yaw2 = 0.0

end_x2 = 5.5
end_y2 = -3
end_yaw2 = 0.0
offset2 = 12.0

path2, _ = calc_4points_bezier_path(
    start_x2, start_y2, start_yaw2,
    end_x2, end_y2, end_yaw2,
    offset2
)

path_x2 = path2[:, 0]
path_y2 = path2[:, 1]
path_yaw2 = np.append(np.arctan2(np.diff(path_y2), np.diff(path_x2)), end_yaw2)

goal2 = np.vstack([path_x2, path_y2, path_yaw2])

start_x3 = 5.5
start_y3 = -3.0
start_yaw3 = 0.0

end_x3 = 3
end_y3 = -3
end_yaw3 = 0.0
offset3 = 12.0

path3, _ = calc_4points_bezier_path(
    start_x3, start_y3, start_yaw3,
    end_x3, end_y3, end_yaw3,
    offset3
)

path_x3 = path3[:, 0]
path_y3 = path3[:, 1]
path_yaw3 = np.append(np.arctan2(np.diff(path_y3), np.diff(path_x3)), end_yaw3)

start_x4 = 3
start_y4 = -3.0
start_yaw4 = 0.0

end_x4 = 3
end_y4 = 0
end_yaw4 = 0.0
offset4 = 15.0

path4, _ = calc_4points_bezier_path(
    start_x4, start_y4, start_yaw4,
    end_x4, end_y4, end_yaw4,
    offset4
)

path_x4 = path4[:, 0]
path_y4 = path4[:, 1]
path_yaw4 = np.append(np.arctan2(np.diff(path_y4), np.diff(path_x4)), end_yaw4)

# goal = np.concatenate([goal1, goal2], axis=1)

plt.plot(path_x1, path_y1, color="red", label="Path 1")
plt.plot(path_x2, path_y2, color="red", label="Path 2")
plt.plot(path_x3, path_y3, color="red", label="Path 3")
plt.plot(path_x4, path_y4, color="red", label="Path 4")
plt.legend()
plt.grid(True)
plt.show()

# print(goal[1, :])