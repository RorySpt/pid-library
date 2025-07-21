//
// Created by tx on 25-7-21.
//

#ifndef PID_H
#define PID_H


#include <algorithm>
#include <concepts>
#include <type_traits>


namespace pid {
    // 模板类使用 T 前缀
    template<typename T = double>
        requires std::is_floating_point_v<T>
    class TPID {
    public:
        struct Params { T kp, ki, kd, maxInt, maxOut; };
    private:
        Params params;
        T error;
        T lastError;
        T integral;
        T output;

    public:
        TPID(Params _params)
            : params(_params),
              error(0), lastError(0), integral(0), output(0) {
        }

        void calc(T ref, T fdb) {
            lastError = error;
            error = ref - fdb;

            T pErr = error * params.kp;
            T dErr = (error - lastError) * params.kd;

            integral += params.ki * error;
            integral = std::clamp(integral, -params.maxInt, params.maxInt);

            T sumErr = pErr + dErr + integral;
            output = std::clamp(sumErr, -params.maxOut, params.maxOut);
        }

        void clear() {
            error = 0;
            lastError = 0;
            integral = 0;
            output = 0;
        }

        T getOutput() const { return output; }
    };

    using PID = TPID<double>;


    // 同样的模板化级联 PID
    template<typename T = double>
        requires std::is_floating_point_v<T>
    class TCascadePID {
    private:
        TPID<T> inner;
        TPID<T> outer;
        T output;

    public:
        using Params = TPID<T>::Params;

        TCascadePID(Params inParams, Params outParams)
            : inner(inParams),
              outer(outParams),
              output(0) {
        }

        void calc(T outRef, T outFdb, T inFdb) {
            outer.calc(outRef, outFdb);
            inner.calc(outer.getOutput(), inFdb);
            output = inner.getOutput();
        }

        void clear() {
            inner.clear();
            outer.clear();
        }

        T getOutput() const { return output; }
    };

    // 为不同浮点类型提供别名
    using CascadePID = TCascadePID<double>;
}


#endif //PID_H
