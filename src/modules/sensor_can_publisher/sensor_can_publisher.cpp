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
#include <uORB/topics/leader_publishable_info.h>
#include <uORB/topics/vehicle_odometry.h>
//#include <uORB/topics/estimator_global_position.h>



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
#include <perf/perf_counter.h>




/* forward declaration */
class SensorCanRx;

class SensorCanPublisher :
	public ModuleBase<SensorCanPublisher>,
	public px4::ScheduledWorkItem
{

public:
	SensorCanPublisher(int max_iter) :
		ScheduledWorkItem(MODULE_NAME,px4::wq_configurations::hp_default),_max_iter(max_iter){}
	int init()
	{
		_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": loop");

		param_t p_comm_id = param_find("SYS_PX4_COMM_ID");

		if (p_comm_id == PARAM_INVALID) {
			PX4_ERR("SYS_PX4_COMM_ID param not found");
			return -1;
		}

		param_get(p_comm_id, &_my_id);

		if (_my_id < 0) {
			PX4_ERR("SYS_PX4_COMM_ID not set or invalid");
			return -1;
		}

		char px4guid_fmt_buffer[PX4_GUID_FORMAT_SIZE];

		board_get_px4_guid_formated(px4guid_fmt_buffer, sizeof(px4guid_fmt_buffer));
		//PX4_INFO_RAW("PX4GUID: %s\n", px4guid_fmt_buffer);
		// 1. Encontrar o comprimento da string
		int len = strlen(px4guid_fmt_buffer);

		if (len >= 6) {
			// 2. Apontar para o início dos últimos 6 dígitos
			char *last_six_str = &px4guid_fmt_buffer[len - 6];

			// 3. Converter de Hexadecimal (base 16) para Inteiro
			int last_six_int = (int)strtol(last_six_str, NULL, 16);

			//PX4_INFO("Last 6 hex as INT: %d", last_six_int);
			param_set(p_comm_id, &last_six_int);
			_my_id = last_six_int%256;
		}
		else {PX4_ERR("GUID string too short");}
		//char guid[PX4_GUID_FORMAT_SIZE];
		//if (board_get_px4_guid_formated(guid, sizeof(guid)) != 0) {
		//	PX4_ERR("Failed to get board GUID");
		//	return -1;
		//}
		//PX4_INFO("Board GUID: %s", guid);
		//PX4_INFO("PX4_COMM_ID = %ld", (long)_my_id);



		px4_sem_init(&peers_sem, 1, 1);

		//_rx = new SensorCanRx(_my_id);
		//_rx->init();
		//if (!_rx) {
		//	PX4_ERR("Failed to create RX");
		//	return -1;
		//}

		PX4_INFO("sensor_can_publisher running");
		//ScheduleOnInterval(20000); // já agenda aqui 👍
		//ScheduleNow();
		ScheduleOnInterval(20000);


		return 0;
	}

	~SensorCanPublisher()
	{
		local_stop();
		perf_free(_loop_perf);


	}

	int print_status()
	{
	//px4_sem_wait(&_peers_sem);

	for (int i = 0; i < MAX_PEERS; i++) {
		int32_t id = peer_ids[i];
		peers[i].latency.print(id);
	}
	return 0;
	//px4_sem_post(&_peers_sem);
	}



	void Run()
	{
		perf_begin(_loop_perf);
		if (should_exit()) {
			PX4_INFO("Stopping work item");
			ScheduleClear();
			return;
		}
		EkfScore self{};


		update_uorb_subs();
		montar_mensage(self);
		publish_ekf_score_uorb(self);
		update_local_peers();
		remove_stale_peers();
		leader_id = elect_leader(self);

		if (leader_id == _my_id) {
			handle_leader_duties();
			}


		if (_max_iter >= 0 && ++_count >= _max_iter) {
			//request_stop();
			local_stop();
			return;
		}

		/* ---------- Reagenda ---------- */
		//ScheduleOnInterval(20000); // ~50Hz
		//ScheduleDelayed(20000);
		perf_end(_loop_perf);

	}




	void local_stop()
	{
		request_stop();

		ScheduleClear();
		usleep(20000);
		px4_sem_destroy(&peers_sem);

		//if (_rx) {
		//	delete _rx;
		//	_rx = nullptr;
		//}
	}

	void update_uorb_subs(){

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
		if(_leader_publishable_info_sub.updated()) {
			_leader_publishable_info_sub.copy(&leader_info);
		}
		if(_veh_odm.updated()) {
			_veh_odm.copy(&odm);
		}
	}

	void montar_mensage(EkfScore &self){
		self.instance_id = _my_id;

		self.vel_test = est.vel_test_ratio;
		self.pos_test = est.pos_test_ratio;
		self.hgt_test = est.hgt_test_ratio;
		self.hdg_test = est.hdg_test_ratio;


		self.pos_var = lpos.eph;
		self.vel_var = lpos.evh;

		self.ekf_flags = est.solution_status_flags;
		self.nav_state = veh.nav_state;
		self.timestamp_utc = hrt_absolute_time();
	}

	void update_local_peers(){
		for (int i = 0; i < MAX_EKF_INSTANCES; i++) {
			if(_leader_publishable_info_subs[i].updated())
			{
				_leader_publishable_info_subs[i].copy(&leader_info);
			}
			if (_ekf_score_subs[i].updated())
			{

				if (_ekf_score_subs[i].copy(&rx)) {

					// ignora mensagem própria
					if (rx.instance_id == _my_id) {
						continue;
					}
					uint64_t rx_time = hrt_absolute_time();
					uint64_t latency = rx_time - rx.timestamp;
					//px4_sem_wait(&peers_sem);
					int idx = find_or_allocate_peer(rx.instance_id);
					if (idx >= 0) {
						PeerState &peer = peers[idx];
						peer.score   = ekfScoreFromUorb(rx);
						peer.last_rx = rx_time;

						if (latency < 10000000) {
							peer.latency.update(latency);
						} else {
							peer.latency.lost_packages++;
						}
					}
					//px4_sem_post(&peers_sem);
				}
			}

		}
	}

	void handle_leader_duties(){
		msg_lpi.instance_id = _my_id;
		msg_lpi.timestamp = hrt_absolute_time();
		msg_lpi.timestamp_sample = odm.timestamp_sample;
		msg_lpi.pose_frame     = odm.pose_frame;
		msg_lpi.velocity_frame = odm.velocity_frame;
		for (int i = 0; i < 3; i++) {
			msg_lpi.position[i] = odm.position[i];
		}
		for (int i = 0; i < 4; i++) {
			msg_lpi.q[i] = odm.q[i];
		}
		for (int i = 0; i < 3; i++) {
			msg_lpi.velocity[i]         = odm.velocity[i];
			msg_lpi.angular_velocity[i] = odm.angular_velocity[i];
		}
		// variances
		for (int i = 0; i < 3; i++) {
			msg_lpi.position_variance[i]    = odm.position_variance[i];
			msg_lpi.orientation_variance[i] = odm.orientation_variance[i];
			msg_lpi.velocity_variance[i]    = odm.velocity_variance[i];
		}
		msg_lpi.reset_counter = odm.reset_counter;
		msg_lpi.quality       = 1;//odm.quality;
		_leader_publishable_info_pub.publish(msg_lpi);
	}

	int32_t get_my_id() const {
		return _my_id;
	}

	int32_t get_leader_id() const {
		return leader_id;
	}

	void publish_ekf_score_uorb(const EkfScore &self)
	{

		msg_ekfs.timestamp = hrt_absolute_time();

		msg_ekfs.instance_id = self.instance_id;

		msg_ekfs.vel_test = self.vel_test;
		msg_ekfs.pos_test = self.pos_test;
		msg_ekfs.hgt_test = self.hgt_test;
		msg_ekfs.hdg_test = self.hdg_test;

		msg_ekfs.pos_var = self.pos_var;
		msg_ekfs.vel_var = self.vel_var;

		msg_ekfs.ekf_flags = self.ekf_flags;
		msg_ekfs.nav_state = self.nav_state;
		msg_ekfs.timestamp_utc = self.timestamp_utc;

		_ekf_score_pub.publish(msg_ekfs);
	}

	void remove_stale_peers()
	{
	uint64_t now = hrt_absolute_time();

	for (int i = 0; i < MAX_PEERS; i++) {

		if (peer_ids[i] < 0) {
		continue;
		}

		uint64_t age = now - peers[i].last_rx;

		if (age > PEER_TIMEOUT_US) {

		PX4_WARN("Peer %" PRId32 " timed out (%.2f ms)",
			peer_ids[i],
			(double)age / 1000.0);

		peer_ids[i] = -1;
		peers[i] = PeerState{}; //reseta estado

		}
	}
	}

private:
	int _max_iter{-1};
	int _count{0};

	int32_t _my_id{-1};
	static constexpr uint64_t PEER_TIMEOUT_US = 50000; // 50 ms


	SensorCanRx *_rx{nullptr};
	ekf_score_s rx{};
	leader_publishable_info_s msg_lpi{};
	ekf_score_s msg_ekfs{};
	/* uORB */
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};
	uORB::Subscription _veh_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _veh_odm{ORB_ID(vehicle_odometry)};
	//uORB::Subscription _global_pos_sub{ORB_ID(estimator_global_position)};


	uORB::Publication<ekf_score_s> _ekf_score_pub{ORB_ID(ekf_score)};
	uORB::Publication<leader_publishable_info_s> _leader_publishable_info_pub{ORB_ID(leader_publishable_info)};
	uORB::Subscription _leader_publishable_info_sub{ORB_ID(leader_publishable_info)};

	/* cache */
	estimator_status_s est{};
	vehicle_status_s veh{};
	vehicle_local_position_s lpos{};
	sensor_gps_s gps{};
	vehicle_odometry_s odm{};
	//estimator_global_position_s global_pos{};
	leader_publishable_info_s leader_info{};

	int32_t leader_id{-1};
	perf_counter_t _loop_perf{nullptr};

	static constexpr int MAX_EKF_INSTANCES = 3;

	uORB::Subscription _ekf_score_subs[MAX_EKF_INSTANCES] = {
	uORB::Subscription(ORB_ID(ekf_score), 0),
	uORB::Subscription(ORB_ID(ekf_score), 1),
	uORB::Subscription(ORB_ID(ekf_score), 2),
	};
	uORB::Subscription _leader_publishable_info_subs[MAX_EKF_INSTANCES] = {
	uORB::Subscription(ORB_ID(leader_publishable_info), 0),
	uORB::Subscription(ORB_ID(leader_publishable_info), 1),
	uORB::Subscription(ORB_ID(leader_publishable_info), 2),
	};


};







static SensorCanPublisher *g_instance{nullptr};

extern "C" __EXPORT int sensor_can_publisher_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: sensor_can_publisher {start|stop|status|listen <instance>|}");
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
			delete g_instance; //delete já invoca o destrutor.
			g_instance = nullptr;
		} else {
			PX4_WARN("sensor_can_publisher not running");
		}

		return 0;
	}


	if (!strcmp(argv[1], "status")) {
		if (g_instance) {
			PX4_INFO("sensor_can_publisher is running");
			g_instance->print_status();
			PX4_INFO("Eu: %ld | Lider Atual: %ld", (long)g_instance->get_my_id(), (long)g_instance->get_leader_id());
		} else {
			PX4_INFO("sensor_can_publisher is stopped");

		}
		return 0;
	}

	if (!strcmp(argv[1], "listen")) {
		if (argc < 3) {
			PX4_ERR("Usage: sensor_can_publisher listen <instance>");
			return -1;
		}

		int instance = std::atoi(argv[2]);

		return listen_ekf_score_instance(instance);
	}


	PX4_ERR("Unknown command");
	return -1;
}
