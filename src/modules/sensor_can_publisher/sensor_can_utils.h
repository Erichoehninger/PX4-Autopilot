#pragma once

#include <px4_platform_common/posix.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_status.h>


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

struct PeerState {
	EkfScore score;
	uint64_t last_rx;
};

extern std::map<int32_t, PeerState> peers;
extern std::mutex peers_mutex;

/* ======================== UTILS ========================== */

inline bool is_valid_peer(const EkfScore &s, uint64_t now)
{
	if (now - s.timestamp_utc > 5'000'000) {
		return false;
	}

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

		if (s > best_score) {
			best_score = s;
			best_id = id;
		}
	}

	return best_id;
}
