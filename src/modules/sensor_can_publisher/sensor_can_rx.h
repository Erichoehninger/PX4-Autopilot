#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/ekf_score.h>

#include "sensor_can_utils.h"

class SensorCanRx : public px4::ScheduledWorkItem
{
public:
	SensorCanRx(int32_t my_id) :
		ScheduledWorkItem("sensor_can_rx", px4::wq_configurations::test1),
		_my_id(my_id),
		_ekf_score_sub(this, ORB_ID(ekf_score)) // 🔥 callback
	{
	}

	int init()
	{
		PX4_INFO("SensorCanRx started (my_id=%ld)", (long)_my_id);

		// registra callback → Run() será chamado quando chegar msg
		if (!_ekf_score_sub.registerCallback()) {
			PX4_ERR("Failed to register ekf_score callback");
			return -1;
		}

		return 0;
	}

	~SensorCanRx() override
	{
		_ekf_score_sub.unregisterCallback();
	}

	void Run() override
	{
		ekf_score_s rx{};
		const uint64_t now = hrt_absolute_time();

		// drena a fila (igual listener)
		while (_ekf_score_sub.update(&rx)) {

			PX4_INFO(
				"RX ekf_score: inst_id=%ld vel=%.2f",
				(long)rx.instance_id,
				(double)rx.vel_test
			);

			// ignora a própria mensagem
			if (rx.instance_id == _my_id) {
				continue;
			}

			uint64_t latency = 0;
			if (rx.timestamp_utc > 0) {
				latency = now - rx.timestamp_utc;
			}

			px4_sem_wait(&peers_sem);

			int idx = find_or_allocate_peer(rx.instance_id);
			if (idx >= 0) {
				PeerState &peer = peers[idx];

				peer.score   = ekfScoreFromUorb(rx);
				peer.last_rx = now;

				if (latency > 0 && latency < 100000) {
					peer.latency.update(latency);
				} else {
					peer.latency.lost_packages++;
				}
			}

			px4_sem_post(&peers_sem);
		}
	}

private:
	int32_t _my_id{-1};

	// 🔥 acorda o Run() quando chega dado (igual poll+listener)
	uORB::SubscriptionCallbackWorkItem _ekf_score_sub;
};
