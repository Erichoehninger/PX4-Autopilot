#pragma once

#include <px4_platform_common/posix.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/sem.h>
#include <poll.h>
#include <uORB/uORB.h>

#define EKF_BROADCAST_PORT 14560
#define EKF_SCORE_TIMEOUT 200000 // 200 ms


#include <cmath>


/* ======================== PAYLOAD ======================== */

struct EkfScore {
    int32_t   instance_id;
    uint64_t timestamp_utc;

    float vel_test;
    float pos_test;
    float hgt_test;
    float hdg_test;

    float pos_var;
    float vel_var;

    uint16_t ekf_flags;
    uint8_t  nav_state;

    int32_t leader_id;
};


/* ======================== PEERS ========================== */
struct LatencyStats {
	uint64_t min_us{UINT64_MAX};
	uint64_t max_us{0};
	uint64_t sum_us{0};
	uint32_t samples{0};
	uint32_t lost_packages{0};
	constexpr static int MAX_SAMPLES = 1000;

	void update(uint64_t latency_us)
	{
		if (samples >= MAX_SAMPLES) {
			// reset suave
			samples = 0;
			sum_us = 0;
			min_us = UINT64_MAX;
			max_us = 0;
		}

		min_us = math::min(min_us, latency_us);
		max_us = math::max(max_us, latency_us);
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
			"lost packages: %d%% (%lu lost out of %lu)",
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

static constexpr int MAX_PEERS = 3;
PeerState peers[MAX_PEERS];
int32_t peer_ids[MAX_PEERS] = {-1, -1, -1}; // -1 = slot livre
sem_t peers_sem;


bool verbose = true;


/* ======================== UTILS ========================== */

inline bool is_valid_peer(const EkfScore &s, uint64_t now)
{
	//Se o timestamp for zero, ou estamos em SITL ou o GPS não está configurado corretamente
	if (s.timestamp_utc == 0){
		//PX4_WARN("timestamp_utc == 0. Analisar Configurações do GPS.");
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
	float score = 0.0f;

	if (PX4_ISFINITE(s.pos_test)) {
		score += 1.0f * s.pos_test;
	}

	if (PX4_ISFINITE(s.hgt_test)) {
		score += 0.5f * s.hgt_test;
	}

	if (PX4_ISFINITE(s.hdg_test)) {
		score += 0.5f * s.hdg_test;
	}

	if (PX4_ISFINITE(s.pos_var)) {
		score += 0.2f * s.pos_var;
	}

	if (PX4_ISFINITE(s.vel_var)) {
		score += 0.2f * s.vel_var;
	}



	return score;
}

inline int32_t elect_leader(const EkfScore &self, int32_t current_leader_id = -1)
{
	constexpr float START_SWITCH  = 0.25f; // precisa melhorar pelo menos isso para trocar
	constexpr float CANCEL_SWITCH = 0.15f; // depois de trocar, só volta se perder essa margem

	static bool switch_armed = false;

	float leader_score = INFINITY;
	float best_score   = compute_score(self);
	int32_t best_id    = self.instance_id;
	uint8_t active_peers = 0;
	//----------------------------------------------------------
	// Descobre o melhor score e o score do líder atual
	//----------------------------------------------------------

	if (self.instance_id == current_leader_id) {
		leader_score = best_score;
	}

	for (int i = 0; i < MAX_PEERS; i++) {

		const int32_t id = peer_ids[i];
		active_peers = 0;
		if (id < 0 || id == self.instance_id) {
			continue;
		}
		else{
			active_peers++;
		}

		const float s = compute_score(peers[i].score);

		if (id == current_leader_id) {
			leader_score = s;
		}

		// menor score vence
		if (s < best_score ||
		   (fabsf(s - best_score) < 1e-6f && id > best_id)) {
			best_score = s;
			best_id = id;
		}
	}

	//----------------------------------------------------------
	// Primeira eleição
	//----------------------------------------------------------

	if (current_leader_id < 0 && active_peers > 0) {
		switch_armed = false;
		return best_id;
	}

	//----------------------------------------------------------
	// Se o líder desapareceu
	//----------------------------------------------------------

	if (!PX4_ISFINITE(leader_score)) {
		switch_armed = false;
		return best_id;
	}

	float improvement = leader_score - best_score;

	//----------------------------------------------------------
	// Schmitt Trigger
	//----------------------------------------------------------

	// Ainda não habilitou a troca
	if (!switch_armed) {

		// Só arma quando a vantagem é suficientemente grande
		if (improvement >= START_SWITCH) {
			switch_armed = true;
		}

		return current_leader_id;
	}

	// Já estava armado.
	// Se perdeu muita vantagem, desarma.
	if (improvement < CANCEL_SWITCH) {
		switch_armed = false;
		return current_leader_id;
	}

	// Continua armado e o melhor candidato continua melhor.
	switch_armed = false;
	return best_id;
}

inline int find_or_allocate_peer(int32_t peer_id)
{
	// 1) já existe?
	for (int i = 0; i < MAX_PEERS; i++) {
		if (peer_ids[i] == peer_id) {
			return i;
		}
	}

	// 2) procurar slot livre
	for (int i = 0; i < MAX_PEERS; i++) {
		if (peer_ids[i] < 0) {
			peer_ids[i] = peer_id;
			peers[i] = PeerState{}; // zera estado
			return i;
		}
	}

	// 3) tabela cheia
	return -1;
}

static EkfScore ekfScoreFromUorb(const ekf_score_s &u)
{
    EkfScore s{};

    s.instance_id = u.instance_id;

    s.vel_test = u.vel_test;
    s.pos_test = u.pos_test;
    s.hgt_test = u.hgt_test;
    s.hdg_test = u.hdg_test;

    s.pos_var = u.pos_var;
    s.vel_var = u.vel_var;

    s.ekf_flags = u.ekf_flags;
    s.nav_state = u.nav_state;
    s.timestamp_utc = u.timestamp_utc;

    s.leader_id = u.leader_id;      // <-- novo

    return s;
}

static int listen_ekf_score_instance(int instance)
{
	int fd = orb_subscribe_multi(ORB_ID(ekf_score), instance);

	if (fd < 0) {
		PX4_ERR("Failed to subscribe to ekf_score instance %d", instance);
		return -1;
	}

	PX4_INFO("Listening to ekf_score instance %d (Ctrl+C to stop)", instance);

	pollfd fds{};
	fds.fd = fd;
	fds.events = POLLIN;
	int count = 0; //numero de mensagens recebidas
	while (count <1) {
		int ret = poll(&fds, 1, 1000);

		if (ret < 0) {
			PX4_ERR("poll error");
			break;
		}

		if (ret == 0) {
			// timeout, igual listener (silencioso)
			continue;
		}

		if (fds.revents & POLLIN) {
			ekf_score_s msg{};

			orb_copy(ORB_ID(ekf_score), fd, &msg);

			PX4_INFO(
			"ekf_score[%d]: ts=%" PRIu64
			" inst_id=%ld leader=%ld vel=%.2f pos=%.2f hgt=%.2f hdg=%.2f",
			instance,
			msg.timestamp,
			(long)msg.instance_id,
			(long)msg.leader_id,
			(double)msg.vel_test,
			(double)msg.pos_test,
			(double)msg.hgt_test,
			(double)msg.hdg_test
			);
			count++;
		}
	}

	orb_unsubscribe(fd);
	return 0;
}
