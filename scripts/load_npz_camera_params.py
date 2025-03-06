import numpy as np

# 假设 npz 文件路径为 "camera_params.npz"
npz_path = "camera_params.npz"
data = np.load(npz_path)

# 输出所有存储的 keys
print("Keys in npz file:", data.files)

# 假定内参矩阵存储在 key 'K'
K = data['K']

# 提取焦距和主点信息
fx = K[0, 0]
fy = K[1, 1]
cx = K[0, 2]
cy = K[1, 2]

print("焦距: fx =", fx, ", fy =", fy)
print("主点: cx =", cx, ", cy =", cy)
