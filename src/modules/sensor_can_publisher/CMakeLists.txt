############################################################################
#
#	CMake para o módulo de CAN Publisher (UAVCAN)
#
############################################################################

if(CONFIG_MODULES_SENSOR_CAN_PUBLISHER)

px4_add_module(
	MODULE sensor_can_publisher
	MAIN sensor_can_publisher
	STACK_MAIN 2048

	SRCS
		sensor_can_publisher.cpp

	DEPENDS
		uavcan
)

endif()
