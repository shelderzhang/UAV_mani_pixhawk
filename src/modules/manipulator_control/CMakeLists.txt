############################################################################
#
#   Copyright (c) SIA UAV Team. All rights reserved.
#
#
############################################################################
px4_add_module(
	MODULE modules__manipulator_control
	MAIN manipulator_control
	COMPILE_FLAGS -Os
	STACK_MAIN 1500
	SRCS
        manipulator_control_main.cpp
        BlockManipulatorControl.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
