import numpy as np

data = np.load('your_file.npz')
# 查看所有数组的 key
print(data.files)
# 例如访问数据
array1 = data['arr_0']
