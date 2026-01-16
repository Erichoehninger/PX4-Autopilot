/****************************************************************************
 *
 *   uORB self test: publish + subscribe
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/debug_key_value.h>

#include <drivers/drv_hrt.h>

#include <unistd.h>
#include <string.h>

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

		PX4_INFO("uORB self-test iniciado");

		/************ PUBLICADOR ************/
		uORB::Publication<debug_key_value_s> pub{ORB_ID(debug_key_value)};

		/************ SUBSCRIBER ************/
		uORB::Subscription sub{ORB_ID(debug_key_value)};

		debug_key_value_s msg{};
		strncpy(msg.key, "teste", sizeof(msg.key));

		for (int i = 0; i < 20; i++) {

			/************ PUBLICA ************/
			msg.timestamp = hrt_absolute_time();
			msg.value = (float)i;

			pub.publish(msg);

			usleep(10000); // 10 ms

			/************ ESCUTA ************/
			if (sub.updated()) {

				debug_key_value_s rx{};
				sub.copy(&rx);

				PX4_INFO("RECEBIDO: key=%s value=%.2f",
					rx.key,
					(double)rx.value);
			}
		}

		PX4_INFO("uORB self-test finalizado");
		return 0;
	}

	print_usage();
	return -1;
}
