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

#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <map>
#include <mutex>
#include <thread>

/* ======================== PAYLOAD ======================== */

struct EkfScore {
	int32_t instance_id;

	float vel_test;
	float pos_test;
	float hgt_test;
	float hdg_test;

	float pos_var;
	float vel_var;

	uint16_t ekf_flags;
	uint8_t nav_state;

	uint64_t timestamp;
};

/* ======================== PEERS ========================== */

struct PeerState {
	EkfScore score;
	uint64_t last_rx;
};

static std::map<int32_t, PeerState> peers;
static std::mutex peers_mutex;

/* ======================== UTILS ========================== */

static bool is_valid_peer(const EkfScore &s, uint64_t now)
{
	if (now - s.timestamp > 5000000) {
		return false;
	}

	//if (!(s.ekf_flags & estimator_status_s::ESTIMATOR_STATUS_FLAGS_VALID_POS)) {
	//	return false;
	//}
	//if (s.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
	//	return false;
	//}

	return true;
}

static float compute_score(const EkfScore &s)
{
	return s.instance_id; //teste, a instancia com maior ID deve ser eleita o lider
		//1.0f * s.vel_test +
		//1.0f * s.pos_test +
		//0.5f * s.hgt_test +
		//0.5f * s.hdg_test +
		//0.2f * s.pos_var +
		//0.2f * s.vel_var;
}

static int32_t elect_leader(const EkfScore &self)
{
	uint64_t now = hrt_absolute_time();

	float best_score = compute_score(self);
	int32_t best_id = self.instance_id;

	std::lock_guard<std::mutex> lock(peers_mutex);

	for (const auto &[id, peer] : peers) {

		if (id == self.instance_id) {
			continue;
		}

		if (!is_valid_peer(peer.score, now)) {
			continue;
		}

		float s = compute_score(peer.score);

		if (s < best_score ||
		    (fabsf(s - best_score) < 1e-4f && id < best_id)) {
			best_score = s;
			best_id = id;
		}
	}

	return best_id;
}

/* ======================== RX THREAD ====================== */

static void udp_rx_thread()
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	param_t p_comm_id = param_find("PX4_COMM_ID");
	int32_t my_id = -1;
	param_get(p_comm_id, &my_id);
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(14560 + my_id); //escuta porta 14560+PX_COMM_ID
	addr.sin_addr.s_addr = INADDR_ANY;

	bind(sock, (sockaddr *)&addr, sizeof(addr));
	PX4_INFO("UDP RX LISTENING ON PORT %d", 14560 + my_id);
	PX4_INFO("UDP RX thread active");

	while (true) {
		EkfScore rx{};
		ssize_t n = recv(sock, &rx, sizeof(rx), 0);

		if (n != sizeof(rx)) {
			continue;
		}

		std::lock_guard<std::mutex> lock(peers_mutex);

		peers[rx.instance_id] = {
			.score = rx,
			.last_rx = hrt_absolute_time()
		};
	PX4_INFO(
			"[RX from ID] %ld",
			(long)rx.instance_id
		);
	}
}

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

	estimator_status_s est{};
	vehicle_status_s veh{};
	vehicle_local_position_s lpos{};

	/* ----------- PARAM ----------- */

	param_t p_comm_id = param_find("PX4_COMM_ID");
	int32_t my_id = -1;
	param_get(p_comm_id, &my_id);

	PX4_INFO("PX4_COMM_ID = %ld", (long)my_id);

	/* ----------- UDP TX ----------- */

	int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in dest{};
	dest.sin_family = AF_INET;
	dest.sin_port = htons(14560);
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
		self.timestamp = est.timestamp;

		for(int i=0; i<3; i++) {

		dest.sin_port = htons(14560+i); //envia para portas 14560+0,1,2 (simulando CAN)
		sendto(
			tx_sock,
			&self,
			sizeof(self),
			0,
			(sockaddr *)&dest,
			sizeof(dest)
		);
		}

		//int32_t leader = elect_leader(self);
		elect_leader(self);
		//PX4_INFO(
		//	"EU=%ld | LIDER=%ld | score=%.3f",
		//	(long)self.instance_id,
		//	(long)leader,
		//	(double)compute_score(self)
		//);

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
