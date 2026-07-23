#include "BoatAlloc.hpp"

#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <float.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor BoatAlloc::desc{task_spawn, custom_command, print_usage};

namespace
{
// void forceToThruster(float fx, float fy, float &throttle, float &servo_norm)
// {
// 	const float magnitude = sqrtf(fx * fx + fy * fy);

// 	if (magnitude < 1e-6f) {
// 		throttle = 0.f;
// 		servo_norm = 0.f;
// 		return;
// 	}

// 	float ang = atan2f(fy, fx);   // 0 = forward, +pi/2 = right, -pi/2 = left

// 	if (ang > M_PI_2_F) {
// 		ang -= M_PI_F;
// 		throttle = -magnitude;

// 	} else if (ang < -M_PI_2_F) {
// 		ang += M_PI_F;
// 		throttle = -magnitude;

// 	} else {
// 		throttle = magnitude;
// 	}

// 	servo_norm = math::constrain(ang / M_PI_2_F, -1.f, 1.f);
// }
} // namespace

BoatAlloc::BoatAlloc() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool BoatAlloc::init()
{
	if (!_thrust_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// DEBUG SAFETY NET: also run periodically so Run() executes even when no
	// thrust setpoint is being published. Remove/loosen once things work.
	ScheduleOnInterval(10_ms);   // 20 Hz

	PX4_INFO("BoatAlloc: init OK, callback registered + 20Hz debug schedule");
	return true;
}

void BoatAlloc::updateParams()
{
	ModuleParams::updateParams();
}

void BoatAlloc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// const hrt_abstime now = hrt_absolute_time();
	// const bool thrust_fresh = _thrust_sub.updated();

	// vehicle_thrust_setpoint_s thrust{};
	// vehicle_torque_setpoint_s torque{};
	// _thrust_sub.copy(&thrust);
	// _torque_sub.copy(&torque);

	// const float Fx = thrust.xyz[0];
	// const float Fy = thrust.xyz[1];
	// const float Mz = torque.xyz[2];

	// const float fxL = 0.5f * (Fx + Mz);
	// const float fxR = 0.5f * (Fx - Mz);
	// const float fyL = 0.5f * Fy;
	// const float fyR = 0.5f * Fy;

	// float thrL, servL, thrR, servR;
	// forceToThruster(fxL, fyL, thrL, servL);
	// forceToThruster(fxR, fyR, thrR, servR);

	// const float m = fmaxf(fabsf(thrL), fabsf(thrR));

	// if (m > 1.f) {
	// 	thrL /= m;
	// 	thrR /= m;
	// }

	// actuator_motors_s motors{};
	// motors.reversible_flags = 0x3;
	// motors.control[0] = 0.5;
	// motors.control[1] = -0.5;
	// motors.timestamp = hrt_absolute_time();
	// _motors_pub.publish(motors);

	actuator_servos_s servos{};
	servos.control[0] = -1;
	servos.control[1] = 1;
	servos.timestamp = hrt_absolute_time();
	_servos_pub.publish(servos);
}

int BoatAlloc::task_spawn(int argc, char *argv[])
{
	BoatAlloc *instance = new BoatAlloc();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int BoatAlloc::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

int BoatAlloc::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION("Steerable twin-thruster allocation for surface vessel");
	PRINT_MODULE_USAGE_NAME("boat_alloc", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int boat_alloc_main(int argc, char *argv[])
{
	return BoatAlloc::main(BoatAlloc::desc, argc, argv);
}