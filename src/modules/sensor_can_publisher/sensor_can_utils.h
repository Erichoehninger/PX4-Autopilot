#pragma once

#include <px4_platform_common/posix.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_status.h>
#define EKF_BROADCAST_PORT 14560


#include <map>
#include <mutex>
#include <cmath>

/* ======================== PAYLOAD ======================== */

struct EkfScore {
	int32_t  instance_id;
	uint64_t timestamp_utc;

	float vel_test;
	float pos_test;
	float hgt_test;
	float hdg_test;

	float pos_var;
	float vel_var;

	uint16_t ekf_flags;
	uint8_t  nav_state;
};


/* ======================== PEERS ========================== */
struct LatencyStats {
	uint64_t min_us{UINT64_MAX};
	uint64_t max_us{0};
	uint64_t sum_us{0};
	uint32_t samples{0};
	uint32_t lost_packages{0};

	void update(uint64_t latency_us)
	{
		if (latency_us < min_us) {
			min_us = latency_us;
		}

		if (latency_us > max_us) {
			max_us = latency_us;
		}

		sum_us += latency_us;
		samples++;
	}

	void print(int32_t peer_id) const
	{
		if (samples == 0) {
			PX4_INFO("Peer %ld: no latency samples collected", (long)peer_id);
			return;
		}

		PX4_INFO(
			"Peer %ld latency [us] | min=%" PRIu64 " avg=%" PRIu64 " max=%" PRIu64 " samples=%lu",
			(long)peer_id,
			min_us,
			sum_us / samples,
			max_us,
			(unsigned long)samples
		);
		PX4_INFO(
			"lost packages: %d%% (%u lost out of %u)",
			percentage_lost_pkgs(),
			lost_packages,
			samples + lost_packages
		);
	}
	int percentage_lost_pkgs() const
	{
		uint32_t total = samples + lost_packages;
		if (total == 0) {
			return 0;
		}
		return static_cast<int>((static_cast<float>(lost_packages) / static_cast<float>(total)) * 100.0f);
	}
};
struct PeerState {
	EkfScore score;
	uint64_t last_rx;
	LatencyStats latency;
};



/* ======================== GLOBALS ======================== */

inline std::map<int32_t, PeerState> peers;
inline std::mutex peers_mutex;
bool verbose = false;


/* ======================== UTILS ========================== */

inline bool is_valid_peer(const EkfScore &s, uint64_t now)
{
	//Se o timestamp for zero, ou estamos em SITL ou o GPS não está configurado corretamente
	if (s.timestamp_utc == 0){
		PX4_WARN("timestamp_utc == 0. Analisar Configurações do GPS.");
	}
	//else if (now > s.timestamp_utc && (now - s.timestamp_utc) > 20000) { // > 20 ms
	//	uint64_t latency_us = now - s.timestamp_utc;
//
	//	PX4_WARN(
	//		"Processing latency too high | latency=%" PRIu64 " us",
	//		latency_us
	//	);
//
	//	return false;
	//}

	// filtros opcionais
	// if (!(s.ekf_flags & estimator_status_s::ESTIMATOR_STATUS_FLAGS_VALID_POS)) return false;
	// if (s.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) return false;

	return true;
}

inline float compute_score(const EkfScore &s)
{
	// teste simples: maior ID vence
	return static_cast<float>(s.instance_id);

	// score real (quando quiser ativar)
	/*
	return
		1.0f * s.vel_test +
		1.0f * s.pos_test +
		0.5f * s.hgt_test +
		0.5f * s.hdg_test +
		0.2f * s.pos_var +
		0.2f * s.vel_var;
	*/
}

inline int32_t elect_leader(const EkfScore &self)
{
	timespec ts{};
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	uint64_t now =
			uint64_t(ts.tv_sec) * 1000000ULL +
			uint64_t(ts.tv_nsec) / 1000ULL;
	//uint64_t now = hrt_absolute_time();

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

		if (s > best_score) {
			best_score = s;
			best_id = id;
		}
	}

	return best_id;
}
