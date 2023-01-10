
/**
 *
 * This module is servo control for demo .
 *
 * @author zcs <zcs1315@buaa.edu.cn>
 */

#include "servo_485_control.hpp"

/**
 * servo control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int servo_485_control_main(int argc, char *argv[]);

bool Servo485Control::is_normal = true;  //静态变量初始化
uint8_t  Servo485Control::test_ID  = 0;             //设置测试ID
int  Servo485Control::test_angle  = 0;      //设置测试角度

Servo485Control::Servo485Control():
	ModuleParams(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_servo_uart_handle = -1;

	for(int i =0; i<4; i++){
		_servo_centre_position[i] = 2047;     //servo centre pos
		_servo_angle[i] = 0;                          	   //servo  expect  angle. 0表示多旋翼状态，也表示舵机中位
		_servo_position[i] = 2047;                    //servo  set position
		_servo_ID[i]  = i+1;
	}

	_servo_speed = 1000;
	is_normal = true;
	_servo_direction_change_hysteresis.set_hysteresis_time_from(false, 5_s);
	_servo_direction_change_hysteresis.set_hysteresis_time_from(true, 5_s);
}

Servo485Control::~Servo485Control()
{
	perf_free(_loop_perf);
}

bool Servo485Control::init()
{
	// if (!_vehicle_local_position_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }



	return true;
}



void Servo485Control::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// centre position set
		_servo_centre_position[0] =_param_servo_centre_1.get();
		_servo_centre_position[1] =_param_servo_centre_2.get();
		_servo_centre_position[2] =_param_servo_centre_3.get();
		_servo_centre_position[3] =_param_servo_centre_4.get();

		_servo_position[0] =_param_servo_centre_1.get();
		_servo_position[1] =_param_servo_centre_2.get();
		_servo_position[2] =_param_servo_centre_3.get();
		_servo_position[3] =_param_servo_centre_4.get();

		//update servo speed
		_servo_speed =_param_servo_speed.get();
	}
}

uint16_t Servo485Control::_limit_position(int position)
{
	if(position < 0)
		return 0;
	else if(position > 0xFFF)
		return 0xFFF;

	return (uint16_t)position;
}

void Servo485Control::_convert_angle_to_position(float servo_angle[], uint16_t servo_centre_position[], uint16_t  servo_position[])
{
	//限幅及方向，角度
	float servo_angle_temp = 0.0;
	float servo_ratio_set = 11.777778;  // 4096/360

	//舵面转换
	//X型排布
	/*          4                 1
	*             \             /
	*               \         /
	*                 \     /
	*                    X
	*                 /    \
	*               /        \
	*             /             \
	*           3                 2
	*
	*
	*
	*/

	//1号舵
	servo_angle_temp = servo_angle[0];
	if(abs(servo_angle[0]) > _param_servo_max_angle.get() ){
		servo_angle_temp =  _param_servo_max_angle.get()*servo_angle[0]/abs(servo_angle[0]);
	}
	servo_position[0] = _limit_position(servo_centre_position[0] + (int)(servo_angle_temp*servo_ratio_set));
	//2舵
	servo_angle_temp = servo_angle[1];
	if(abs(servo_angle[1]) > _param_servo_max_angle.get() ){
		servo_angle_temp =  _param_servo_max_angle.get()*servo_angle[1]/abs(servo_angle[1]);
	}
	servo_position[1] = _limit_position(servo_centre_position[1] + (int)(servo_angle_temp*servo_ratio_set));
	//3舵
	servo_angle_temp = servo_angle[2];
	if(abs(servo_angle[2]) > _param_servo_max_angle.get() ){
		servo_angle_temp =  _param_servo_max_angle.get()*servo_angle[2]/abs(servo_angle[2]);
	}
	servo_position[2] = _limit_position(servo_centre_position[2] +(int)(servo_angle_temp*servo_ratio_set));
	//4舵
	servo_angle_temp = servo_angle[3];
	if(abs(servo_angle[3]) > _param_servo_max_angle.get() ){
		servo_angle_temp =  _param_servo_max_angle.get()*servo_angle[3]/abs(servo_angle[3]);
	}
	servo_position[3] = _limit_position(servo_centre_position[3] + (int)(servo_angle_temp*servo_ratio_set));

}


void Servo485Control::_Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	*DataH = (Data>>8);
	*DataL = (Data&0xff);
}

void Servo485Control::_send_servos_control_frame(uint8_t ID[], uint16_t servo_speed, uint16_t servo_position[])
{
	uint8_t i, j;
	uint8_t mesLen = ((6+1)*SERVO_NUM+4);
	uint8_t checkSum = 0;
	uint8_t bBuf[7];
	uint8_t buf_pos[6];


	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = 0x83;
	bBuf[5] = 0x2A;
	bBuf[6] = 0x06;
	if (_servo_uart_handle != -1)
		write(_servo_uart_handle, bBuf, 7);

	checkSum = 0xfe + mesLen + 0x83 + 0x2A + 0x06;
	for(i=0; i<SERVO_NUM; i++){

		if (_servo_uart_handle != -1)
			write(_servo_uart_handle,&ID[i], 1);

		_Host2SCS(buf_pos+0, buf_pos+1, servo_position[i]);
		_Host2SCS(buf_pos+2, buf_pos+3, 0);
		_Host2SCS(buf_pos+4, buf_pos+5, servo_speed);

		if (_servo_uart_handle != -1)
			write(_servo_uart_handle,buf_pos, 6);


		checkSum += ID[i];
		for(j=0; j<6; j++){
			checkSum += buf_pos[j];
		}
	}
	checkSum = ~checkSum;
	if (_servo_uart_handle != -1)
		write(_servo_uart_handle,&checkSum, 1);
}

int Servo485Control::set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud) {
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		default:
		    PX4_WARN("ERR: baudrate: %d\n", baud);
		    return -EINVAL;
	}

	struct termios uart_config;


	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);
	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);
	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}


void Servo485Control::run()
{

	/* update parameters from storage */
	parameters_update();

	/**********************************串口初始化**************************************/
	char *uart_name = (char*)"/dev/ttyS3";  //对应端口为UART4

	_servo_uart_handle = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (_servo_uart_handle < 0) {
		PX4_ERR("ERROR opening UART %s, aborting..\n", uart_name);
		return ;
	}
	/* set baudrate */
	if(false == set_uart_baudrate(_servo_uart_handle,115200)){
		PX4_ERR("[FC]set_uart_baudrate is failed\n");
		return ;
    	}
	hrt_abstime servo_485_past = hrt_absolute_time();

	while (!should_exit()) {
		perf_begin(_loop_perf);

		if (hrt_elapsed_time(&servo_485_past)>20_ms)
		{
			// 正常工作模式
			if(is_normal == true){

				// 利用时延函数
				float  angle_1 = 10.0;
				float  angle_2 = -10.0;
				_servo_direction_change_hysteresis.update(hrt_absolute_time());

				if(_servo_direction_change_hysteresis.get_state())
				{
					_servo_angle[0] = angle_1;
					_servo_angle[1] = angle_2;
					_servo_angle[2] = angle_1;
					_servo_angle[3] = angle_2;
					_servo_direction_change_hysteresis.set_state_and_update(false, hrt_absolute_time());
					// if (hrt_elapsed_time(&servo_485_past)>50_ms){
						// PX4_INFO("TRUE");
					// }
				}else{
					_servo_angle[0] = -angle_1;
					_servo_angle[1] = -angle_2;
					_servo_angle[2] = -angle_1;
					_servo_angle[3] = -angle_2;
					// if (hrt_elapsed_time(&servo_485_past)>50_ms){
						// PX4_INFO("FALSE");
					// }
					_servo_direction_change_hysteresis.set_state_and_update(true, hrt_absolute_time());
				}

				//uorb 消息发送
				_s_angle.timestamp = hrt_absolute_time();
				_s_angle.servo_angle[0] = _servo_angle[0];
				_s_angle.servo_angle[1] = _servo_angle[1];
				_s_angle.servo_angle[2] = _servo_angle[2];
				_s_angle.servo_angle[3] = _servo_angle[3];
				_servo_angle_pub.publish(_s_angle);


				//根据角度进行转换并发送数据
				_convert_angle_to_position(_servo_angle, _servo_centre_position, _servo_position);

			}else{
				//测试模式
				_servo_angle[test_ID] = test_angle;
				_convert_angle_to_position(_servo_angle, _servo_centre_position, _servo_position);
			}

			_send_servos_control_frame(_servo_ID,_servo_speed,_servo_position);
			servo_485_past = hrt_absolute_time();
		}
		perf_end(_loop_perf);
		px4_usleep(SERVO_485_CONTROL_INTERVAL);
	}
	close(_servo_uart_handle);

}

int Servo485Control::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("servo_485_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}


Servo485Control *Servo485Control::instantiate(int argc, char *argv[])
{


	Servo485Control *instance = new Servo485Control();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Servo485Control::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "test")) {
		// is_normal = false;
		print_usage("convert to test mode");
		return PX4_OK;
	}

	if (!strcmp(verb, "normal")) {
		// is_normal = true;
		print_usage("convert to normal mode");
		return PX4_OK;
	}


	if (!strcmp(verb, "ID")) {
		if(argc>=2){
			if(argv[1][0]>='0' && argv[1][0]<='9'){
				test_ID = argv[1][0] - '0';
				PX4_WARN("Set Test ID is %d",test_ID);
			}else{
				print_usage("None ID");
			}
		}else{
			print_usage("None ID");
		}

		return PX4_OK;
	}


	if (!strcmp(verb, "ANGLE")) {
		if(argc>=2){
			if(argv[1][0]>='0' && argv[1][0]<='9'){
				test_angle = (argv[1][0] - '0')*10;
				test_angle += argv[1][1] - '0';
				PX4_WARN("Set Test angle is %d",test_angle);
			}else{
				print_usage("None ID");
			}
		}else{
			print_usage("None ID");
		}

		return PX4_OK;
	}


	return print_usage("unknown command");
}

int Servo485Control::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
servo_485_control
### Implementation
...
### Examples
CLI usage example:
$ servo_485_control start
$ servo_485_control status
$ servo_485_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("servo_485_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int Servo485Control::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int servo_485_control_main(int argc, char *argv[])
{
	return Servo485Control::main(argc, argv);
}


