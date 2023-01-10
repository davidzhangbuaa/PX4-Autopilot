px4_add_module(
	MODULE modules__servo_485_control
	MAIN servo_485_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		servo_485_control.cpp
	)
