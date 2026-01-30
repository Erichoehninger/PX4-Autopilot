/****************************************************************************
 *
 *   EKF2 innovation_test_ratio publisher + peer listener + leader election
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/ekf_score.h>


#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <inttypes.h>
#include <time.h>
#include <poll.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sensor_can_utils.h"
#include "sensor_can_rx.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>




/* forward declaration */
class SensorCanRx;

class SensorCanPublisher :
	public ModuleBase<SensorCanPublisher>,
	public px4::ScheduledWorkItem
{

public:
	SensorCanPublisher(int max_iter) :
		ScheduledWorkItem(MODULE_NAME,px4::wq_configurations::test1),_max_iter(max_iter){}
	int init()
	{
		param_t p_comm_id = param_find("PX4_COMM_ID");

		if (p_comm_id == PARAM_INVALID) {
			PX4_ERR("PX4_COMM_ID param not found");
			return -1;
		}

		param_get(p_comm_id, &_my_id);

		if (_my_id < 0) {
			PX4_ERR("PX4_COMM_ID not set or invalid");
			return -1;
		}

		//char guid[PX4_GUID_FORMAT_SIZE];
		//if (board_get_px4_guid_formated(guid, sizeof(guid)) != 0) {
		//	PX4_ERR("Failed to get board GUID");
		//	return -1;
		//}
		//PX4_INFO("Board GUID: %s", guid);
		//PX4_INFO("PX4_COMM_ID = %ld", (long)_my_id);

		_tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (_tx_sock < 0) {
			PX4_ERR("socket creation failed");
			return -1;
		}

		int enable = 1;
		setsockopt(_tx_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));

		_dest.sin_family = AF_INET;
		_dest.sin_port = htons(EKF_BROADCAST_PORT);
		_dest.sin_addr.s_addr = inet_addr("255.255.255.255");

		px4_sem_init(&peers_sem, 1, 1);

		_rx = new SensorCanRx(_my_id);
		if (!_rx) {
			PX4_ERR("Failed to create RX");
			return -1;
		}

		PX4_INFO("sensor_can_publisher running");
		ScheduleOnInterval(20000); // já agenda aqui 👍

		return 0;
	}

	~SensorCanPublisher()
	{
		local_stop();

		if (_tx_sock >= 0) {
			close(_tx_sock);
		}


	}


	void Run()
	{
		if (should_exit()) {
			PX4_INFO("Stopping work item");
			ScheduleClear();
			return;
		}



		/* ---------- uORB ---------- */

		if (_est_sub.updated()) {
			_est_sub.copy(&est);
		}

		if (_veh_sub.updated()) {
			_veh_sub.copy(&veh);
		}

		if (_lpos_sub.updated()) {
			_lpos_sub.copy(&lpos);
		}

		if (_gps_sub.updated()) {
			_gps_sub.copy(&gps);
		}

		/* ---------- Timestamp ---------- */

		timespec ts{};
		px4_clock_gettime(CLOCK_REALTIME, &ts);

		uint64_t tempo_real_us =
			uint64_t(ts.tv_sec) * 1000000ULL +
			uint64_t(ts.tv_nsec) / 1000ULL;

		/* ---------- Monta mensagem ---------- */

		EkfScore self{};
		self.instance_id = _my_id;

		self.vel_test = est.vel_test_ratio;
		self.pos_test = est.pos_test_ratio;
		self.hgt_test = est.hgt_test_ratio;
		self.hdg_test = est.hdg_test_ratio;

		self.pos_var = lpos.eph;
		self.vel_var = lpos.evh;

		self.ekf_flags = est.solution_status_flags;
		self.nav_state = veh.nav_state;
		self.timestamp_utc = tempo_real_us;

		sendto(
			_tx_sock,
			&self,
			sizeof(self),
			0,
			(sockaddr *)&_dest,
			sizeof(_dest)
			);

		publish_ekf_score_uorb(self);
		/* ---------- Leader election ---------- */


		leader_id = elect_leader(self);

		if (verbose) {
			PX4_INFO("EU=%ld | LIDER=%ld",
				(long)self.instance_id,
				(long)leader_id
			);
		}

		/* ---------- Controle de iteração ---------- */

		if (_max_iter >= 0 && ++_count >= _max_iter) {
			//request_stop();
			local_stop();
			return;
		}

		/* ---------- Reagenda ---------- */
		ScheduleOnInterval(20000); // ~50Hz
	}




	void local_stop()
	{
		request_stop();

		if (_rx) {
			delete _rx;
			_rx = nullptr;
		}

		px4_sem_destroy(&peers_sem);
		ScheduleClear();
	}


	int32_t get_my_id() const {
		return _my_id;
	}

	int32_t get_leader_id() const {
		return leader_id;
	}

		void publish_ekf_score_uorb(const EkfScore &self)
	{
		ekf_score_s msg{};
		msg.timestamp = hrt_absolute_time();

		msg.instance_id = self.instance_id;

		msg.vel_test = self.vel_test;
		msg.pos_test = self.pos_test;
		msg.hgt_test = self.hgt_test;
		msg.hdg_test = self.hdg_test;

		msg.pos_var = self.pos_var;
		msg.vel_var = self.vel_var;

		msg.ekf_flags = self.ekf_flags;
		msg.nav_state = self.nav_state;
		msg.timestamp_utc = self.timestamp_utc;

		_ekf_score_pub.publish(msg);
	}

private:
	int _max_iter{-1};
	int _count{0};

	int32_t _my_id{-1};
	int _tx_sock{-1};
	sockaddr_in _dest{};

	SensorCanRx *_rx{nullptr};


	/* uORB */
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};
	uORB::Subscription _veh_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Publication<ekf_score_s> _ekf_score_pub{ORB_ID(ekf_score)};


	/* cache */
	estimator_status_s est{};
	vehicle_status_s veh{};
	vehicle_local_position_s lpos{};
	sensor_gps_s gps{};

	int32_t leader_id{-1};





};
static SensorCanPublisher *g_instance{nullptr};

extern "C" __EXPORT int sensor_can_publisher_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: sensor_can_publisher {start|stop|status} ...");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		if (argc < 3) {
			PX4_ERR("missing max_iter");
			return -1;
		}

		int max_iter = std::atoi(argv[2]);

		for (int i = 3; i < argc; i++) {
			if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) {
				verbose = true;
			}
		}

		if (!g_instance) {
			g_instance = new SensorCanPublisher(max_iter);

			if (!g_instance) {
				PX4_ERR("alloc failed");
				return -1;
			}

			if (g_instance->init() != 0) {
				delete g_instance;
				g_instance = nullptr;
				return -1;
			}
			}


		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (g_instance) {
			PX4_INFO("Stopping sensor_can_publisher");

			g_instance->local_stop();
			delete g_instance;
			g_instance = nullptr;
		} else {
			PX4_WARN("sensor_can_publisher not running");
		}

		return 0;
	}


	if (!strcmp(argv[1], "status")) {
		if (g_instance) {
			PX4_INFO("sensor_can_publisher is running");
			px4_sem_wait(&peers_sem);
				for (int i = 0; i < MAX_PEERS; i++) {
					int32_t id = peer_ids[i];
					if (id < 0) {
						continue;
					}
					peers[i].latency.print(id);
				}
			px4_sem_post(&peers_sem);

			PX4_INFO("Líder Atual: %ld", (long)g_instance->get_leader_id());
		} else {
			PX4_INFO("sensor_can_publisher is stopped");

		}
		return 0;
	}

	PX4_ERR("Unknown command");
	return -1;
}

