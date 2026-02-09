#ifndef ONE_MOTOR_IMOTOR_HPP_
#define ONE_MOTOR_IMOTOR_HPP_
/**
 * @file IMotor.hpp
 * @brief 非模板电机接口，允许使用类型擦除的状态访问
 */

#include "Error.hpp"

#include <one/can/CanDriver.hpp>
#include <one/motor/Units.hpp>
#include <one/motor/dji/DjiFrame.hpp>
#include <one/motor/dm/DmFrame.hpp>
#include <tl/expected.hpp>
#include <variant>

namespace one::motor {

/**
 * @typedef AnyStatus
 * @brief 电机状态的变体类型
 *
 * 用于类型擦除的电机状态，可以表示不同类型的电机状态。
 * 当前支持达妙(DM)和大疆(DJI)电机的状态类型。
 */
using AnyStatus = std::variant<dm::DmStatus, dji::MotorStatus>;
using AnyPlainStatus = std::variant<dm::DmStatusPlain, dji::MotorStatusPlain>;

/**
 * @class IMotor
 * @brief 电机接口类
 *
 * 定义了电机的基本操作接口，允许对不同类型电机进行统一操作。
 * 该接口是非模板的，便于使用类型擦除技术。
 */
class IMotor {
  public:
    /**
     * @brief 虚析构函数
     */
    virtual ~IMotor() = default;

    /**
     * @brief 使能电机
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 使能电机，允许电机响应控制指令。
     */
    virtual tl::expected<void, Error> enable() = 0;

    /**
     * @brief 禁用电机
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 禁用电机，停止电机响应控制指令。
     */
    virtual tl::expected<void, Error> disable() = 0;

    /**
     * @brief 设置位置参考值
     * @param ref 位置参考值
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标位置。
     */
    virtual void setPosUnitRef(const units::Angle &ref) noexcept = 0;

    virtual void setPosRef(float ref) noexcept = 0;

    /**
     * @brief 设置角速度参考值
     * @param ref 角速度参考值
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标角速度。
     */
    virtual void setAngUnitRef(const units::AngularVelocity &ref) noexcept = 0;

    virtual void setAngRef(float ref) noexcept = 0;

    /**
     * @brief 设置扭矩参考值
     * @param ref 扭矩参考值
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标扭矩。
     */
    virtual void setTorUnitRef(const units::Torque &ref) noexcept = 0;

    virtual void setTorRef(float ref) noexcept = 0;

    /**
     * @brief 同时设置位置、角速度和扭矩参考值
     * @param pos_ref 位置参考值
     * @param ang_ref 角速度参考值
     * @param tor_ref 扭矩参考值
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 同时设置电机的位置、角速度和扭矩参考值。
     */
    virtual void setUnitRefs(const units::Angle &pos_ref,
                             const units::AngularVelocity &ang_ref,
                             const units::Torque &tor_ref) noexcept = 0;

    virtual void setRefs(float pos_ref, float ang_ref,
                         float tor_ref) noexcept = 0;

    /**
     * @brief 获取电机状态（变体类型）
     * @return 电机状态，成功返回AnyStatus，失败返回Error
     *
     * 获取电机的当前状态，返回一个变体类型，可以表示不同类型的电机状态。
     */
    virtual tl::expected<AnyStatus, Error> getStatusVariant() = 0;

    virtual tl::expected<AnyPlainStatus, Error> getPlainStatusVariant() = 0;

    [[nodiscard]] bool initialized() const { return m_initialized && m_driver; }

  protected:
    virtual tl::expected<void, Error> init(can::CanDriver &driver) {
        if (m_initialized) {
            return tl::make_unexpected(Error{ErrorCode::MotorAlreadyInitialized,
                                             "Motor already initialized"});
        }
        m_driver = &driver;
        m_initialized = true;
        return {};
    };

    [[nodiscard]] can::CanDriver *driver() const { return m_driver; }

    void resetInitialization() noexcept {
        m_initialized = false;
        m_driver = nullptr;
    }

  private:
    bool m_initialized = false;
    can::CanDriver *m_driver = nullptr;
};
} // namespace one::motor

#endif // ONE_MOTOR_IMOTOR_HPP_
