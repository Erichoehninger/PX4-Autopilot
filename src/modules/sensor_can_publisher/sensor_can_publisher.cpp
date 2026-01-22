/****************************************************************************
 *
 *   EKF2 innovation_test_ratio publisher + peer listener + leader election
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>


#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sensor_can_utils.h"
#include "sensor_can_rx.h"

/* ======================== MAIN =========================== */

extern "C" __EXPORT int sensor_can_publisher_main(int argc, char *argv[]);

int sensor_can_publisher_main(int argc, char *argv[])
{
	if (argc < 3 || strcmp(argv[1], "start")) {
		PX4_INFO("Usage: sensor_can_publisher start <num_iteracoes | -1>");
		return -1;
	}

	int max_iter = std::atoi(argv[2]);

	/* ----------- uORB ----------- */

	uORB::Subscription est_sub{ORB_ID(estimator_status)};
	uORB::Subscription veh_sub{ORB_ID(vehicle_status)};
	uORB::Subscription lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription gps_sub{ORB_ID(sensor_gps)};


	estimator_status_s est{};
	vehicle_status_s veh{};
	vehicle_local_position_s lpos{};
	sensor_gps_s gps{};

	/* ----------- PARAM ----------- */

	param_t p_comm_id = param_find("PX4_COMM_ID");
	int32_t my_id = -1;
	param_get(p_comm_id, &my_id);

	PX4_INFO("PX4_COMM_ID = %ld", (long)my_id);

	/* ----------- UDP TX ----------- */

	int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in dest{};
	dest.sin_family = AF_INET;
	dest.sin_addr.s_addr = inet_addr("127.0.0.1");

	/* ----------- RX THREAD ----------- */

	std::thread(udp_rx_thread).detach();

	PX4_INFO("sensor_can_publisher running");

	int count = 0;

	while (true) {

		if (!est_sub.updated()) {
			usleep(2000);
			continue;
		}

		est_sub.copy(&est);

		if (veh_sub.updated()) {
			veh_sub.copy(&veh);
		}

		if (lpos_sub.updated()) {
			lpos_sub.copy(&lpos);
		}

		if (gps_sub.updated()) {
			gps_sub.copy(&gps);
		}

		EkfScore self{};

		self.instance_id = my_id;

		self.vel_test = est.vel_test_ratio;
		self.pos_test = est.pos_test_ratio;
		self.hgt_test = est.hgt_test_ratio;
		self.hdg_test = est.hdg_test_ratio;

		self.pos_var = lpos.eph;
		self.vel_var = lpos.evh;

		self.ekf_flags = est.solution_status_flags;
		self.nav_state = veh.nav_state;
		self.timestamp_utc = gps.time_utc_usec;

		for (int i = 0; i < 3; i++) {
			dest.sin_port = htons(14560 + i);
			sendto(
				tx_sock,
				&self,
				sizeof(self),
				0,
				(sockaddr *)&dest,
				sizeof(dest)
			);
		}

		int32_t leader = elect_leader(self);

		PX4_INFO(
			"\nEU=%ld | LIDER=%ld | timestamp_utc=%llu\n",
			(long)self.instance_id,
			(long)leader,
			(unsigned long long)self.timestamp_utc
		);

		count++;
		if (max_iter >= 0 && count >= max_iter) {
			break;
		}

		usleep(20000); // ~50Hz
	}

	close(tx_sock);
	PX4_INFO("sensor_can_publisher finished");
	return 0;
}
