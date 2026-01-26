#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "sensor_can_utils.h"

class SensorCanRx :
	public px4::ScheduledWorkItem
{
public:
	SensorCanRx(int32_t my_id) :
		ScheduledWorkItem("sensor_can_rx", px4::wq_configurations::test1),
		_my_id(my_id)
	{
		init_socket();
		ScheduleOnInterval(1000); // 1 kHz
	}

	~SensorCanRx() override
	{
		if (_sock >= 0) {
			close(_sock);
		}
	}

	void Run() override
	{
		pollfd fds{};
		fds.fd = _sock;
		fds.events = POLLIN;
        static uint64_t last_log = 0;
        uint64_t now = hrt_absolute_time();

        if (now - last_log > 10000000) { //log a cada 100s
            PX4_INFO("RX alive");
            last_log = now;
        }

		if (poll(&fds, 1, 0) <= 0) {
			return;
		}

        constexpr int MAX_PKTS_PER_RUN = 5;
        int processed = 0;
		EkfScore rx{};
		while (processed < MAX_PKTS_PER_RUN) { //drena o pipe
            ssize_t n = recv(_sock, &rx, sizeof(rx), MSG_DONTWAIT);
            if (n <= 0) {
                break;
            }
            // processa só o MAIS RECENTE
            if (n != sizeof(rx) || rx.instance_id == _my_id) {
                processed++;
                continue;
		    }

            //uint64_t now = hrt_absolute_time();
            //PX4_INFO("NOW: %" PRIu64 "", now);
            timespec ts{};
            px4_clock_gettime(CLOCK_REALTIME, &ts);

            uint64_t tempo_real_us =
                uint64_t(ts.tv_sec) * 1000000ULL +
                uint64_t(ts.tv_nsec) / 1000ULL;



            uint64_t latency = tempo_real_us - rx.timestamp_utc;
            {
                std::lock_guard<std::mutex> lock(peers_mutex);
                PeerState &peer = peers[rx.instance_id];

                peer.score = rx;
                peer.last_rx = tempo_real_us;

                if (rx.timestamp_utc > 0 && latency < 100000) {
                    peer.latency.update(latency);
                } else if (latency > 100000) {
                    peer.latency.lost_packages++;
                }
            }
            processed++;
        }



	}

private:
	void init_socket()
	{
		_sock = socket(AF_INET, SOCK_DGRAM, 0);

		int reuse = 1;
		setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        int rcvbuf = 256 * 1024; // 256 KB
        setsockopt(_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));


		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(EKF_BROADCAST_PORT);
		addr.sin_addr.s_addr = INADDR_ANY;

		if (bind(_sock, (sockaddr *)&addr, sizeof(addr)) < 0) {
			PX4_ERR("RX bind failed");
			close(_sock);
			_sock = -1;
			return;
		}

		PX4_INFO("RX listening on UDP %d", EKF_BROADCAST_PORT);
	}

	int _sock{-1};
	int32_t _my_id{-1};
};


