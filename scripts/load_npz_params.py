import numpy as np

# 假设 npz 文件路径为 "params.npz"
npz_path = "params.npz"
data = np.load(npz_path)

# 打印所有参数 key
print("Parameters in the npz file:", data.files)

# 读取并打印每个参数
for key in data.files:
    value = data[key]
    print(f"{key}: {value}")
