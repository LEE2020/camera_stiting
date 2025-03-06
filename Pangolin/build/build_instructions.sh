# 创建 build 目录后执行：
cmake .. -DENABLE_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
make -j8   # 根据机器核数调整
