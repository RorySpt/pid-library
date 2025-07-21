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
    private:
        T kp;
        T ki;
        T kd;
        T maxInt;
        T maxOut;
        T error;
        T lastError;
        T integral;
        T output;

    public:
        TPID(T p, T i, T d, T mi, T mo)
            : kp(p), ki(i), kd(d), maxInt(mi), maxOut(mo),
              error(0), lastError(0), integral(0), output(0) {
        }

        void calc(T ref, T fdb) {
            lastError = error;
            error = ref - fdb;

            T pErr = error * kp;
            T dErr = (error - lastError) * kd;

            integral += ki * error;
            integral = std::clamp(integral, -maxInt, maxInt);

            T sumErr = pErr + dErr + integral;
            output = std::clamp(sumErr, -maxOut, maxOut);
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
        TCascadePID(const T inParams[5], const T outParams[5])
            : inner(inParams[0], inParams[1], inParams[2], inParams[3], inParams[4]),
              outer(outParams[0], outParams[1], outParams[2], outParams[3], outParams[4]),
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
