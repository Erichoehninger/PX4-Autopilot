#pragma once

#include "sensor_can_utils.h"

#include <px4_platform_common/log.h>
#include <parameters/param.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

/* ======================== GLOBALS ======================== */

inline std::map<int32_t, PeerState> peers;
inline std::mutex peers_mutex;



/* ======================== RX THREAD ====================== */

inline void udp_rx_thread()
{
    param_t p_comm_id = param_find("PX4_COMM_ID");
    int32_t my_id = -1;
    param_get(p_comm_id, &my_id);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    // Define um timeout de 5 segundos no socket
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        PX4_ERR("Failed to set socket timeout");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14560 + my_id);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr *)&addr, sizeof(addr)) < 0) {
        PX4_ERR("RX bind failed");
        close(sock);
        return;
    }

    PX4_INFO("UDP RX listening on port %d (Auto-close after 5s idle)", 14560 + my_id);

    while (true) {
        EkfScore rx{};
        ssize_t n = recv(sock, &rx, sizeof(rx), 0);

        if (n < 0) {
            // No PX4/NuttX/Linux, EAGAIN e EWOULDBLOCK geralmente são o mesmo valor.
            // Verificamos apenas EAGAIN para evitar o erro de compilação logical-op.
            if (errno == EAGAIN) {
                PX4_WARN("UDP RX timeout: No data for 5 seconds. Unbinding socket...");
                break;
            }

            // Caso seja um erro de interrupção de sistema, podemos tentar ler novamente
            if (errno == EINTR) {
                continue;
            }

            PX4_ERR("UDP RX error: %d", errno);
            break;
        }

        if (n == sizeof(rx)) {
		timespec ts{};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		uint64_t now =
				uint64_t(ts.tv_sec) * 1000000ULL +
				uint64_t(ts.tv_nsec) / 1000ULL;

		std::lock_guard<std::mutex> lock(peers_mutex);
		PeerState &peer = peers[rx.instance_id];

		peer.score = rx;
		peer.last_rx = now;

		if (rx.timestamp_utc > 0 && now > rx.timestamp_utc) {
			uint64_t latency_us = now - rx.timestamp_utc;
			peer.latency.update(latency_us);
		}

		PX4_INFO(
			"Received from ID: %ld | latency=%" PRIu64 " us",
			(long)rx.instance_id,
			(now > rx.timestamp_utc) ? (now - rx.timestamp_utc) : 0
		);
	}
    }
    	{
		std::lock_guard<std::mutex> lock(peers_mutex);

		for (auto &it : peers) {
			it.second.latency.print(it.first);
		}
	}
    PX4_INFO("Closing socket on port %d", 14560 + my_id);
    close(sock);
}
