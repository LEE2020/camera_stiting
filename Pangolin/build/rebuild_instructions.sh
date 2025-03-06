rm -rf build
mkdir build && cd build
cmake .. -DENABLE_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
make -j8
