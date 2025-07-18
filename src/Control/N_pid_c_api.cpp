#include "OneMotor/Control/pid_c_api.h"
#include "OneMotor/Control/PID.hpp"
#include <cstdlib>

using C_PID = OneMotor::Control::PIDController<
    OneMotor::Control::Positional,
    float,
    OneMotor::Control::WithDeadband,
    OneMotor::Control::WithIntegralLimit,
    OneMotor::Control::WithDerivativeFilter,
    OneMotor::Control::WithOutputFilter,
    OneMotor::Control::WithOutputLimit>;

struct PID_Handle {
    C_PID* impl;
};

PID_Handle_t PID_Create(const PID_Params_t* params) {
    if (!params) return nullptr;
    auto h = static_cast<PID_Handle*>(std::malloc(sizeof(PID_Handle)));
    if (!h) return nullptr;
    OneMotor::Control::PID_Params<float> p;
    p.Kp = params->Kp;
    p.Ki = params->Ki;
    p.Kd = params->Kd;
    p.MaxOutput = params->MaxOutput;
    p.Deadband = params->Deadband;
    p.IntegralLimit = params->IntegralLimit;
    p.DerivativeFilterRC = params->DerivativeFilterRC;
    p.OutputFilterRC = params->OutputFilterRC;
    h->impl = new C_PID(p);
    return h;
}

void PID_Destroy(PID_Handle_t handle) {
    if (!handle) return;
    delete handle->impl;
    std::free(handle);
}

float PID_Compute(PID_Handle_t handle, float ref, float measure) {
    if (!handle || !handle->impl) return 0.0f;
    return handle->impl->compute(ref, measure);
}

void PID_Reset(PID_Handle_t handle) {
    if (!handle || !handle->impl) return;
    handle->impl->reset();
}
