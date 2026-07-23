#pragma once
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class BoatAlloc : public ModuleBase,
                  public ModuleParams,
                  public px4::ScheduledWorkItem
{
public:
    static Descriptor desc;

    BoatAlloc();
    ~BoatAlloc() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
protected:
	void updateParams() override;

private:
    void Run() override;

    uORB::SubscriptionCallbackWorkItem _thrust_sub{this, ORB_ID(vehicle_thrust_setpoint)};
    uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};

    uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
    uORB::Publication<actuator_servos_s> _servos_pub{ORB_ID(actuator_servos)};

    hrt_abstime _last_log{0};   // <-- this line is what's missing
};