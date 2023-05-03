import numpy as np
import matplotlib.pyplot as plt

from library.bezier_path import calc_4points_bezier_path


start_x = 0
start_y = 0
start_yaw = 0

end_x = 4
end_y = 0
end_yaw = 0

path, _ = calc_4points_bezier_path(
    start_x, start_y, start_yaw,
    end_x, end_y, end_yaw,
    15
)

path_yaw = np.append(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])), end_yaw)



x = path[:, 0]
y = path[:, 1]

plt.plot(x, y, color="red", label="Path")
plt.plot(np.linspace(0, end_x, 100), path_yaw, color="blue", label="Yaw")
plt.grid(True)
plt.show()
