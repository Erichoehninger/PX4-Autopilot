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
#include <errno.h>
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
#include <uORB/topics/actuator_test.h>
//#include <uORB/topics/estimator_global_position.h>



#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <inttypes.h>
#include <time.h>
#include <poll.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sensor_can_utils.h"
#include "sensor_can_rx.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>
#include <perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>

orb_advert_t _mavlink_log_pub{nullptr};



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

		if (_my_id < 0 || _my_id > 255) {
			PX4_ERR("SYS_PX4_COMM_ID not set or invalid");
			PX4_WARN("Attempting to set SYS_PX4_COMM_ID from board GUID");
			char px4guid_fmt_buffer[PX4_GUID_FORMAT_SIZE];

			board_get_px4_guid_formated(px4guid_fmt_buffer, sizeof(px4guid_fmt_buffer));
			//PX4_INFO_RAW("PX4GUID: %s\n", px4guid_fmt_buffer);
			// 1. Encontrar o comprimento da string
			int len = strlen(px4guid_fmt_buffer);


				// 2. Apontar para o início dos últimos 6 dígitos
				char *last_six_str = &px4guid_fmt_buffer[len - 6];

				// 3. Converter de Hexadecimal (base 16) para Inteiro
				int last_six_int = (int)strtol(last_six_str, NULL, 16);

				//PX4_INFO("Last 6 hex as INT: %d", last_six_int);
				param_set(p_comm_id, &last_six_int);
				_my_id = last_six_int%256;
				PX4_WARN("SYS_PX4_COMM_ID set to %d from board GUID", static_cast<int>(_my_id));


		}


		//char guid[PX4_GUID_FORMAT_SIZE];
		//if (board_get_px4_guid_formated(guid, sizeof(guid)) != 0) {
		//	PX4_ERR("Failed to get board GUID");
		//	return -1;
		//}
		//PX4_INFO("Board GUID: %s", guid);
		//PX4_INFO("PX4_COMM_ID = %ld", (long)_my_id);



		px4_sem_init(&peers_sem, 1, 1);

		PX4_INFO("sensor_can_publisher running");
		//ScheduleOnInterval(20000); // já agenda aqui 👍
		//ScheduleNow();
		ScheduleOnInterval(10000); //diminui pra 10ms pra pegar mais rápido o líder

		return 0;
	}

	void udp_test()
	{
	ekf_score_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.instance_id = _my_id;
	msg.leader_id = _my_id;
	PX4_INFO("TEST: this=%p sock=%d", this, _sock);

	int type;
	socklen_t len = sizeof(type);

	int ret = getsockopt(
	_sock,
	SOL_SOCKET,
	SO_TYPE,
	&type,
	&len
	);

	PX4_INFO("getsockopt ret=%d errno=%d type=%d",
		ret,
		errno,
		type);

	PX4_INFO("Sending UDP test packet...");
	PX4_INFO("Socket FD: %d", _sock);
	if (_sock < 0) {
	PX4_ERR("INVALID SOCKET %d", _sock);
	return;
	}
	 ret = sendto(_sock,
				&msg,
				sizeof(msg),
				0,
				(sockaddr *)&_broadcast_addr,
				sizeof(_broadcast_addr));

	if (ret < 0) {
		PX4_ERR("sendto failed errno=%d", errno);

	} else {
		PX4_INFO("sendto OK (%d bytes)", (int)ret);
	}

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
		float score = compute_score(peers[i].score);
		PX4_INFO("Score: %.3f", (double)score);
	}
	return 0;
	//px4_sem_post(&_peers_sem);
	}

	void init_udp_socket()
	{
		PX4_INFO("INIT UDP SOCKET CALLED");
		if (_socket_initialized) {
			return;
		}

		_sock = socket(AF_INET, SOCK_DGRAM, 0);

		if (_sock < 0) {
			PX4_ERR("socket() failed errno=%d", errno);
			return;
		}

		int broadcast = 1;

		setsockopt(
			_sock,
			SOL_SOCKET,
			SO_BROADCAST,
			&broadcast,
			sizeof(broadcast)
		);


		memset(&_broadcast_addr, 0, sizeof(_broadcast_addr));

		_broadcast_addr.sin_family = AF_INET;
		_broadcast_addr.sin_port = htons(14560);
		_broadcast_addr.sin_addr.s_addr = inet_addr("192.168.0.255");

		memset(&_local_addr, 0, sizeof(_local_addr));

		_local_addr.sin_family = AF_INET;
		_local_addr.sin_port = htons(14560);
		_local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

		if (bind(_sock,
		(struct sockaddr *)&_local_addr,
		sizeof(_local_addr)) < 0) {

		PX4_ERR("bind failed errno=%d", errno);
		close(_sock);
		_sock = -1;
		return;
		}

		PX4_INFO("RUN UDP socket initialized fd=%d", _sock);

		_socket_initialized = true;
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

		init_udp_socket();
		if (!_socket_initialized) {
			return;
		}

		update_uorb_subs();
		montar_mensage(self);

		receive_ekf_score_udp();
		//update_local_peers();
		remove_stale_peers();
		update_leader_local(self);   // era: leader_id = elect_leader(self, leader_id);
		publish_ekf_score_uorb(self);

		_my_curr_score = self;
		if (leader_id == self.instance_id) {
			handle_leader_duties();
			}
		PX4_INFO("NEW_LEADER_ID: %d", static_cast<int>(leader_id));




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

	void receive_ekf_score_udp()
	{
	while (true) {

		ekf_score_s packet{};

		sockaddr_in src_addr{};
		socklen_t addrlen = sizeof(src_addr);

		ssize_t ret = recvfrom(_sock,
				&packet,
				sizeof(packet),
				0,
				(sockaddr *)&src_addr,
				&addrlen);



		if (ret < 0) {

		// não há mais pacotes
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			break;
		}

		_udp_rx_errors++;
		PX4_WARN("recvfrom failed (%d)", errno);
		break;
		}

		if ((size_t)ret != sizeof(packet)) {
		_udp_bad_size++;
		continue;
		}

		if (packet.instance_id == _my_id) {
		_udp_self_packets++;
		continue;
		}

		_udp_rx_count++;
		_last_rx_time = hrt_absolute_time();
		_last_rx_from = packet.instance_id;
		uint64_t latency = _last_rx_time - packet.timestamp;

		int idx = find_or_allocate_peer(packet.instance_id);

		if (idx < 0) {
		continue;
		}

		PeerState &peer = peers[idx];

		peer.score = ekfScoreFromUorb(packet);
		peer.last_rx = _last_rx_time;
		peer.miss_count = 0;

		if (latency < 10000000) {
		peer.latency.update(latency);

		} else {
		peer.latency.lost_packages++;
		}

		maybe_update_leader(packet.instance_id,
				peer.score);
	}
	}

	void send_ekf_score_udp(const ekf_score_s &msg)
	{
	if (_sock < 0) {
		PX4_WARN("socket invalid");
		return;
	}

	ssize_t ret = sendto(_sock,
				&msg,
				sizeof(msg),
				0,
				(sockaddr *)&_broadcast_addr,
				sizeof(_broadcast_addr));

	if (ret < 0) {
		PX4_WARN("sendto errno=%d", errno);

	} else {
		PX4_INFO("sendto %d bytes", (int)ret);

		if ((size_t)ret == sizeof(msg)) {
		_udp_tx_count++;
		_last_tx_time = hrt_absolute_time();
		}
	}
	}

	void print_udp_status()
	{
	PX4_INFO("--------------- UDP ----------------");
	PX4_INFO("TX packets      : %lu", _udp_tx_count);
	PX4_INFO("RX packets      : %lu", _udp_rx_count);
	PX4_INFO("RX errors       : %lu", _udp_rx_errors);
	PX4_INFO("Bad size        : %lu", _udp_bad_size);
	PX4_INFO("Self packets    : %lu", _udp_self_packets);
	PX4_INFO("Leader          : %ld", leader_id);

	if (_last_rx_time != 0) {
		PX4_INFO("Last RX %.2f ms ago",
			(double)(hrt_absolute_time() - _last_rx_time)/1000.0);
	} else {
		PX4_INFO("Never received");
	}
	}

	void local_stop()
	{
	request_stop();

	if (_sock >= 0) {
		close(_sock);
		_sock = -1;
		_socket_initialized = false;
	}

	ScheduleClear();
	usleep(20000);
	px4_sem_destroy(&peers_sem);
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
		self.timestamp_utc = static_cast<uint64_t>(leader_id);//hrt_absolute_time();
	}

	/*void update_local_peers(){
		for (int i = 0; i < MAX_EKF_INSTANCES; i++) {
			if(_leader_publishable_info_subs[i].updated())
			{
				_leader_publishable_info_subs[i].copy(&leader_info);
			}
			if (_ekf_score_subs[i].updated())
			{
				if (_ekf_score_subs[i].copy(&rx)) {

					if (rx.instance_id == _my_id) {
						continue;
					}
					uint64_t rx_time = hrt_absolute_time();
					uint64_t latency = rx_time - rx.timestamp;
					int idx = find_or_allocate_peer(rx.instance_id);
					if (idx >= 0) {
					PeerState &peer = peers[idx];
					peer.score      = ekfScoreFromUorb(rx);
					peer.last_rx    = rx_time;
					peer.miss_count = 0;              // NOVO

					if (latency < 10000000) {
						peer.latency.update(latency);
					} else {
						peer.latency.lost_packages++;
					}

					maybe_update_leader(rx.instance_id, peer.score);
					}
				}
			}
		}
	}*/

	// Só troca de líder quando chega um score NOVO melhor que o do líder atual.
	// Se o score novo for do próprio líder, atualiza o valor de referência mesmo
	// que tenha piorado (senão o líder fica "travado" com um score desatualizado).
	void maybe_update_leader(int32_t candidate_id, const EkfScore &candidate_score)
	{
		float candidate_val = compute_score(candidate_score);

		if (leader_id < 0) {
			leader_id = candidate_id;
			leader_score = candidate_val;
			return;
		}

		if (candidate_id == leader_id) {
			leader_score = candidate_val;
			return;
		}

		if (candidate_val < leader_score) {
			leader_id = candidate_id;
			leader_score = candidate_val;
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

	float get_my_curr_score() const {

		return compute_score(_my_curr_score);

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
		msg_ekfs.timestamp_utc = static_cast<uint64_t>(leader_id); //self.timestamp_utc;

		msg_ekfs.leader_id = leader_id;

		send_ekf_score_udp(msg_ekfs);
		_ekf_score_pub.publish(msg_ekfs);
	}

	void remove_stale_peers()
	{
		uint64_t now = hrt_absolute_time();
		static constexpr uint8_t MAX_MISSES = 100; // ajuste depois de calibrar

		for (int i = 0; i < MAX_PEERS; i++) {

			if (peer_ids[i] < 0) {
				continue;
			}

			uint64_t age = now - peers[i].last_rx;

			if (age > PEER_TIMEOUT_US) {

				peers[i].miss_count++;

				if (peers[i].miss_count < MAX_MISSES) {
					continue; // ainda dentro da tolerância
				}

				PX4_WARN("Peer %" PRId32 " timed out (%.2f ms, %d misses)",
					peer_ids[i],
					(double)age / 1000.0,
					peers[i].miss_count);

				if (peer_ids[i] == leader_id) {
					_leader_lost = true;
				}

				peer_ids[i] = -1;
				peers[i] = PeerState{};
			}
		}
	}

	void elect_leader_from_scratch(const EkfScore &self)
	{
		float best_score = compute_score(self);
		int32_t best_id = self.instance_id;

		for (int i = 0; i < MAX_PEERS; i++) {
			int32_t id = peer_ids[i];
			if (id < 0) {
				continue;
			}
			float score = compute_score(peers[i].score);
			if (score < best_score) {
				best_score = score;
				best_id = id;
			}
		}

		leader_id = best_id;
		leader_score = best_score;
	}

	void update_leader_local(const EkfScore &self)
	{
		if (leader_id < 0 || _leader_lost) {
			if (active_peer_count() > 0) {
				elect_leader_from_scratch(self);
			}
			_leader_lost = false;
			return;
		}


		float self_score = compute_score(self);

		if (self.instance_id == leader_id) {
			leader_score = self_score;
		} else if (self_score < leader_score) {
			leader_id = self.instance_id;
			leader_score = self_score;
		}
	}

	void test_esc(float value, float duration_s)
{
	actuator_test_s msg{};
	msg.timestamp = hrt_absolute_time();

	// motor 1 (pode parametrizar depois)
	msg.function = actuator_test_s::FUNCTION_MOTOR1;
	// motor 2 ESC ATUAL TA COM ID = 2
	msg.function = actuator_test_s::FUNCTION_MOTOR1 +1;

	msg.value = value;

	if (duration_s <= 0.f) {
		msg.action = actuator_test_s::ACTION_RELEASE_CONTROL;
		msg.timeout_ms = 0;

	} else {
		msg.action = actuator_test_s::ACTION_DO_CONTROL;
		msg.timeout_ms = (int)(duration_s * 1000.f);

		// limite de segurança igual ao PX4
		if (msg.timeout_ms > 3000) {
			msg.timeout_ms = 3000;
		}
	}

	PX4_INFO("TEST_ESC -> value: %.2f | duration: %.2f s", (double)value, (double)duration_s);

	_actuator_test_pub.publish(msg);
}

private:
	int _max_iter{-1};
	int _count{0};

	int32_t _my_id{-1};
	static constexpr uint64_t PEER_TIMEOUT_US = 200000; // 100 ms


	/* ETHERNET */
	int _sock{-1};
	sockaddr_in _local_addr{};
	sockaddr_in _broadcast_addr{};
	bool _socket_initialized{false};
	uint32_t _udp_tx_count{0};
	uint32_t _udp_rx_count{0};
	uint32_t _udp_rx_errors{0};
	uint32_t _udp_bad_size{0};
	uint32_t _udp_self_packets{0};

	int _last_rx_time{0};
	int _last_tx_time{0};

	uint8_t _last_rx_from{255};


	/* uORB */
	SensorCanRx *_rx{nullptr};
	ekf_score_s rx{};
	leader_publishable_info_s msg_lpi{};
	ekf_score_s msg_ekfs{};
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};
	uORB::Subscription _veh_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _veh_odm{ORB_ID(vehicle_odometry)};
	//uORB::Subscription _global_pos_sub{ORB_ID(estimator_global_position)};


	uORB::Publication<ekf_score_s> _ekf_score_pub{ORB_ID(ekf_score)};
	uORB::Publication<leader_publishable_info_s> _leader_publishable_info_pub{ORB_ID(leader_publishable_info)};
	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};
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
	int32_t previous_leader_id{-1};
	perf_counter_t _loop_perf{nullptr};
	EkfScore _my_curr_score{};
	float leader_score{FLT_MAX};
	bool _leader_lost{false};
	static constexpr int MAX_EKF_INSTANCES = 4;



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
	if (!strcmp(argv[1], "udp_test")) {

		if (!g_instance) {
			PX4_ERR("module not running");
			return -1;
		}

		g_instance->udp_test();
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
			g_instance->print_udp_status();
			PX4_INFO("Eu: %ld| Meu Score: %.3f | Lider Atual: %ld", (long)g_instance->get_my_id(), (double)g_instance->get_my_curr_score(), (long)g_instance->get_leader_id());
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

	if (!strcmp(argv[1], "test_esc")) {

		if (!g_instance) {
			PX4_ERR("module not running");
			return -1;
		}

		if (argc < 4) {
			PX4_ERR("Usage: sensor_can_publisher test_esc <value -1..1> <time_s>");
			return -1;
		}

		float value = atof(argv[2]);
		float time_s = atof(argv[3]);

		// clamp por segurança
		if (value > 1.0f) value = 1.0f;
		if (value < -1.0f) value = -1.0f;

		if (time_s < 0.f) time_s = 0.f;

		g_instance->test_esc(value, time_s);

		return 0;
	}


	PX4_ERR("Unknown command");
	return -1;
}
