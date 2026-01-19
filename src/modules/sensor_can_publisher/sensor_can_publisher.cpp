/****************************************************************************
 *
 *   EKF2 innovation_test_ratio listener + UDP publisher
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>

#include <unistd.h>
#include <string.h>
#include <cstdlib>

// Socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct EkfScore {
	float vel;
	float pos;
	float hgt;
	float hdg;
	uint16_t flags;
	uint64_t timestamp;
};

extern "C" __EXPORT int sensor_can_publisher_main(int argc, char *argv[]);

int sensor_can_publisher_main(int argc, char *argv[])
{
	if (argc < 3 || strcmp(argv[1], "start")) {
		PX4_INFO("Usage: sensor_can_publisher start num_iteracoes(-1 == infinito)");
		return -1;
	}

	int max_tests = std::atoi(argv[2]);

	PX4_INFO("Listening to estimator_status (EKF health)");

	uORB::Subscription status_sub{ORB_ID(estimator_status)};

	/* ================= UDP SOCKET SETUP ================= */

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		PX4_ERR("Failed to create UDP socket");
		return -1;
	}

	sockaddr_in dest_addr{};
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(14560); // porta destino
	dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	PX4_INFO("UDP socket ready -> 127.0.0.1:14560");

	/* ==================================================== */

	int count = 0;

	while (true) {

		if (status_sub.updated()) {

			estimator_status_s status{};
			status_sub.copy(&status);

			EkfScore score{};
			score.vel = status.vel_test_ratio;
			score.pos = status.pos_test_ratio;
			score.hgt = status.hgt_test_ratio;
			score.hdg = status.hdg_test_ratio;
			score.flags = status.solution_status_flags;
			score.timestamp = status.timestamp;

			/* ---------- SEND VIA UDP ---------- */

			ssize_t sent = sendto(
				sock,
				&score,
				sizeof(score),
				0,
				(sockaddr *)&dest_addr,
				sizeof(dest_addr)
			);

			if (sent != sizeof(score)) {
				PX4_WARN("UDP send failed (%d)", (int)sent);
			}

			/* --------------------------------- */

			PX4_INFO(
				"EKF test ratios | vel=%.2f pos=%.2f hgt=%.2f hdg=%.2f | ts=%llu",
				(double)score.vel,
				(double)score.pos,
				(double)score.hgt,
				(double)score.hdg,
				(unsigned long long)score.timestamp
			);

			count++;
			if (max_tests >= 0 && count >= max_tests) {
				break;
			}
		}
		//"sleep pode aumentar latência pois coloca o processo na fila de espera
		//e quando bate o tempo de clock, a preempção do Sistema Operacional e troca de contexto
		//pode levar um tempo. TODO: Estudar outra forma de fazer isso, vou verificar meus materias" - Russi 19/01/26
		usleep(20000); // ~50 Hz
	}

	close(sock);

	PX4_INFO("Listener finished");
	return 0;
}
