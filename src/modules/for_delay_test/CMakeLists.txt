px4_add_module(
	MODULE modules__for_delay_test
	MAIN for_delay_test
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS -Os
	SRCS
		for_delay_test_main.cpp
	DEPENDS
		platforms__common
)