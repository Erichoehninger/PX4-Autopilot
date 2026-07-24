############################################################################
#
#   CMake for TMR CAN Manager Module
#
############################################################################

if(CONFIG_MODULES_TMR_CAN_MANAGER)

px4_add_module(
	MODULE tmr_can_manager
	MAIN tmr_can_manager
	STACK_MAIN 2048

	SRCS
		tmr_can_manager.cpp

	DEPENDS
		px4_work_queue
)

endif()
