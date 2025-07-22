//
// Created by tx on 25-7-21.
//

#ifndef PID_H
#define PID_H

#include <algorithm>
#include <concepts>
#include <type_traits>

namespace pid {
    /**
     * @brief 模板化的PID控制器类
     * @tparam T 浮点类型，默认为double
     * @requires std::is_floating_point_v<T> 确保模板参数为浮点类型
     */
    template<typename T = double>
        requires std::is_floating_point_v<T>
    class TPID {
    public:
        /// PID参数结构体
        struct Params {
            T kp;       ///< 比例系数
            T ki;       ///< 积分系数
            T kd;       ///< 微分系数
            T maxInt;   ///< 积分项最大值(抗积分饱和)
            T maxOut;   ///< 输出最大值
        };

    private:
        Params params;  ///< PID参数
        T error;        ///< 当前误差
        T lastError;    ///< 上一次误差
        T integral;     ///< 积分项累计值
        T output;       ///< 控制器输出

    public:
        /**
         * @brief 构造函数
         * @param _params PID参数结构体
         */
        TPID(Params _params)
            : params(_params),
              error(0), lastError(0), integral(0), output(0) {
        }

        /**
         * @brief 计算PID输出
         * @param ref 设定值(Reference)
         * @param fdb 反馈值(Feedback)
         */
        void calc(T ref, T fdb) {
            lastError = error;
            error = ref - fdb;

            // 计算比例项
            T pErr = error * params.kp;
            // 计算微分项(使用后向差分)
            T dErr = (error - lastError) * params.kd;

            // 计算积分项并限制范围
            integral += params.ki * error;
            integral = std::clamp(integral, -params.maxInt, params.maxInt);

            // 计算总输出并限制范围
            T sumErr = pErr + dErr + integral;
            output = std::clamp(sumErr, -params.maxOut, params.maxOut);
        }

        /**
         * @brief 清除PID控制器状态
         */
        void clear() {
            error = 0;
            lastError = 0;
            integral = 0;
            output = 0;
        }

        /**
         * @brief 获取控制器输出
         * @return 当前PID输出值
         */
        T getOutput() const { return output; }
    };

    /// 默认的double类型PID控制器别名
    using PID = TPID<double>;

    /**
     * @brief 模板化的级联PID控制器类
     * @tparam T 浮点类型，默认为double
     * @requires std::is_floating_point_v<T> 确保模板参数为浮点类型
     */
    template<typename T = double>
        requires std::is_floating_point_v<T>
    class TCascadePID {
    private:
        TPID<T> inner;  ///< 内环PID控制器
        TPID<T> outer;  ///< 外环PID控制器
        T output;       ///< 最终输出值

    public:
        using Params = TPID<T>::Params;  ///< 使用TPID的参数类型

        /**
         * @brief 构造函数
         * @param inParams 内环PID参数
         * @param outParams 外环PID参数
         */
        TCascadePID(Params inParams, Params outParams)
            : inner(inParams),
              outer(outParams),
              output(0) {
        }

        /**
         * @brief 计算级联PID输出
         * @param outRef 外环设定值
         * @param outFdb 外环反馈值
         * @param inFdb 内环反馈值
         */
        void calc(T outRef, T outFdb, T inFdb) {
            // 先计算外环输出(作为内环的设定值)
            outer.calc(outRef, outFdb);
            // 再计算内环输出
            inner.calc(outer.getOutput(), inFdb);
            output = inner.getOutput();
        }

        /**
         * @brief 清除级联PID控制器状态
         */
        void clear() {
            inner.clear();
            outer.clear();
        }

        /**
         * @brief 获取控制器输出
         * @return 当前级联PID输出值
         */
        T getOutput() const { return output; }
    };

    /// 默认的double类型级联PID控制器别名
    using CascadePID = TCascadePID<double>;
}

#endif //PID_H
