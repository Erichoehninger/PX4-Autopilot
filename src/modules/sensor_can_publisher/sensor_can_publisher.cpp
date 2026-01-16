/****************************************************************************
 *
 *   CAN Publisher – módulo de teste
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

extern "C" __EXPORT int sensor_can_publisher_main(int argc, char *argv[]);

static void print_usage()
{
	PX4_INFO("Usage:");
	PX4_INFO("  sensor_can_publisher start");
}

int sensor_can_publisher_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		PX4_INFO("ola, sou o CAN_PUBLISHER e existo");
		return 0;
	}

	print_usage();
	return -1;
}
