import numpy as np



theta = 0
u1 = 10
u2 = -10
u3 = 10
u4 = -10


J_for = np.array([
    [-np.sin(theta+np.pi/4), -np.sin(theta+3*np.pi/4), -np.sin(theta+5*np.pi/4), -np.sin(theta+7*np.pi/4)],
    [np.cos(theta+np.pi/4), np.cos(theta+3*np.pi/4), np.cos(theta+5*np.pi/4), np.cos(theta+7*np.pi/4)],
    [1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22)]], dtype=np.float32)

inv_vec = np.array([[u1],[u2],[u3],[u4]], dtype=np.float32)


for_vec = J_for@inv_vec


print(for_vec)
