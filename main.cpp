#include <fmt/base.h>

#include "pid.h"


int main() {
    double inParams[5] = {1.0, 0.1, 0.01, 100.0, 100.0};
    double outParams[5] = {0.5, 0.05, 0.005, 50.0, 50.0};

    

    pid::CascadePID pid(inParams, outParams);
    pid.calc(10.0, 5.0, 2.0);

    fmt::println("out_put {}", pid.getOutput());

    return 0;
}
