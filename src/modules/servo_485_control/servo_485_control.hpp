

/**
 *
 * This module is servo control for demo .
 *
 * @author zcs <zcs1315@buaa.edu.cn>
 */


#include <float.h>
#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/servo_angle.h>
#include <uORB/uORB.h>

#include <lib/hysteresis/hysteresis.h>


static const uint8_t SERVO_NUM = 4;
// static const hrt_abstime SERVO_485_CONTROL_INTERVAL = 20_ms;
// static constexpr uint64_t SERVO_485_CONTROL_INTERVAL{20_ms};
static constexpr uint64_t SERVO_485_CONTROL_INTERVAL{10000};


using namespace time_literals;
using uORB::SubscriptionData;


class Servo485Control: public ModuleBase<Servo485Control>, public ModuleParams
{
public:
	Servo485Control();
	~Servo485Control();

	bool init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Servo485Control *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<servo_angle_s> _servo_angle_pub{ORB_ID(servo_angle)};
	servo_angle_s _s_angle{};


	perf_counter_t	_loop_perf; /**< loop performance counter */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SERVO_CENTRE_1>) _param_servo_centre_1,
		(ParamInt<px4::params::SERVO_CENTRE_2>) _param_servo_centre_2,
		(ParamInt<px4::params::SERVO_CENTRE_3>) _param_servo_centre_3,
		(ParamInt<px4::params::SERVO_CENTRE_4>) _param_servo_centre_4,
		(ParamInt<px4::params::SERVO_CENTRE_5>) _param_servo_centre_5,
		(ParamInt<px4::params::SERVO_SPEED>) _param_servo_speed,

		(ParamFloat<px4::params::SERVO_MAX_ANGLE>) _param_servo_max_angle
		// (ParamInt<px4::params::UUV_STAB_MODE>) _param_stabilization,
		// (ParamInt<px4::params::UUV_SKIP_CTRL>) _param_skip_ctrl
	)

	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(bool force = false);

	int _servo_uart_handle{-1};

	uint32_t _servo_speed{1000};
	static bool is_normal;
	static uint8_t test_ID;
	static int test_angle;


	uint16_t _servo_centre_position[SERVO_NUM];
	uint16_t _servo_position[SERVO_NUM];
	float _servo_angle[SERVO_NUM];
	uint8_t _servo_ID[SERVO_NUM];

	systemlib::Hysteresis _servo_direction_change_hysteresis{false};

	uint16_t _limit_position(int position);
	void _convert_angle_to_position(float servo_angle[], uint16_t servo_centre_position[], uint16_t  servo_position[]);
	void _Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);
	void _send_servos_control_frame(uint8_t ID[], uint16_t servo_speed, uint16_t servo_position[]);
	int set_uart_baudrate(const int fd, unsigned int baud);//自动选取波特率


};
