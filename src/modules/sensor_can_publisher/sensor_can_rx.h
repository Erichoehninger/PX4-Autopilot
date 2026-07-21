#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/ekf_score.h>

#include "sensor_can_utils.h"

class SensorCanRx : public px4::ScheduledWorkItem
{
public:
	static constexpr int MAX_EKF_INSTANCES = 3;

	SensorCanRx(int32_t my_id) :
		ScheduledWorkItem("sensor_can_rx", px4::wq_configurations::test1),
		_my_id(my_id)
	{
	}

	int init()
	{
		PX4_INFO("SensorCanRx started (my_id=%ld)", (long)_my_id);

		const unsigned count = orb_group_count(ORB_ID(ekf_score));

		for (unsigned i = 0; i < count && i < MAX_EKF_INSTANCES; i++) {
			_subs[i] = orb_subscribe_multi(ORB_ID(ekf_score), i);

			if (_subs[i] < 0) {
				PX4_WARN("Failed to subscribe ekf_score inst %u", i);
			} else {
				PX4_INFO("Subscribed ekf_score instance %u", i);
			}
		}

		ScheduleOnInterval(20000); // 50 Hz
		return 0;
	}

	~SensorCanRx() override
	{
		for (int i = 0; i < MAX_EKF_INSTANCES; i++) {
			if (_subs[i] >= 0) {
				orb_unsubscribe(_subs[i]);
				_subs[i] = -1;
			}
		}
	}

	void Run() override
	{
		const uint64_t now = hrt_absolute_time();

		for (int i = 0; i < MAX_EKF_INSTANCES; i++) {

			if (_subs[i] < 0) {
				continue;
			}

			// poll não bloqueante
			px4_pollfd_struct_t fds{};
			fds.fd     = _subs[i];
			fds.events = POLLIN;

			const int ret = px4_poll(&fds, 1, 0);

			if (ret <= 0) {
				continue;
			}

			ekf_score_s rx{};

			// drena fila da instância
			while (orb_copy(ORB_ID(ekf_score), _subs[i], &rx) == PX4_OK) {

				PX4_INFO(
					"RX ekf_score inst=%d instance_id=%ld",
					i, (long)rx.instance_id
				);

				// ignora mensagem própria
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
	}

private:
	int32_t _my_id{-1};
	int _subs[MAX_EKF_INSTANCES]{0, 1, 2}; // deixar o slot 2 vazio por agora já que os testes estão utilzando apenas 2 Pixhawks por agora (05/02/26)
};
