############################################################################
#
#	CMake para o módulo de CAN Publisher
#
# Criado em 16/01/26
############################################################################

if(CONFIG_MODULES_SENSOR_CAN_PUBLISHER)

px4_add_module(
	MODULE sensor_can_publisher
	MAIN sensor_can_publisher
	STACK_MAIN 2048
	SRCS
		sensor_can_publisher.cpp
)

endif()
