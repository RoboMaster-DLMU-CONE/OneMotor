#ifndef PID_C_API_H
#define PID_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PID_Handle* PID_Handle_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOutput;
    float Deadband;
    float IntegralLimit;
    float DerivativeFilterRC;
    float OutputFilterRC;
} PID_Params_t;

/**
 * @brief 创建新的PID控制器实例。
 * @param params 指向PID参数的指针。
 * @return 返回PID实例的句柄。
 */
PID_Handle_t PID_Create(const PID_Params_t* params);

/**
 * @brief 销毁PID控制器实例。
 * @param handle PID实例句柄。
 */
void PID_Destroy(PID_Handle_t handle);

/**
 * @brief 计算PID输出。
 * @param handle PID实例句柄。
 * @param ref 设定值。
 * @param measure 测量值。
 * @return 返回计算后的PID输出。
 */
float PID_Compute(PID_Handle_t handle, float ref, float measure);

/**
 * @brief 重置PID内部状态。
 * @param handle PID实例句柄。
 */
void PID_Reset(PID_Handle_t handle);

#ifdef __cplusplus
}
#endif

#endif //PID_C_API_H
