 #include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(pangolin, m) {
    m.doc() = "Pangolin Python bindings";
    // ...在这里添加绑定代码，例如函数、类的绑定...
}