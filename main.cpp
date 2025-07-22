#include <fmt/base.h>
#include <imgui.h>
#include "pid/PID.h"


int main() {

    constexpr pid::CascadePID::Params inParams = {1.0, 0.0, 0.1, 20.0, 20.0};
    constexpr pid::CascadePID::Params outParams = {0.5, 0.05, 0.005, 50.0, 50.0};

    double value = 100.0;

    pid::PID pid(inParams);
    for (int i = 0; i<100; ++i) {
        pid.calc(5.0, value);
        value += pid.getOutput();
        fmt::println("[{1: >2}] offset: {2: >10.{0}f}   value: {3: >10.{0}f}",4, i, pid.getOutput(), value);
    }

    return 0;
}
