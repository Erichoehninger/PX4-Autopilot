/****************************************************************************
 *
 *   EKF2 innovation_test_ratio publisher + peer listener + leader election
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>
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
#include <uORB/topics/actuator_test.h>
//#include <uORB/topics/estimator_global_position.h>



#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <inttypes.h>
#include <time.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sensor_can_utils.h"
#include "sensor_can_rx.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>
#include <perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>

#define UDP_PORT 14560
#define MAX_RX_HISTORY 10
#define MAX_MSG_SIZE 128

struct EthernetMessage
{
	uint32_t sender_id;
	uint32_t counter;
	uint64_t timestamp;
	char data[MAX_MSG_SIZE];
};
orb_advert_t _mavlink_log_pub{nullptr};



/* forward declaration */
class SensorCanRx;

class SensorCanPublisher :
	public ModuleBase<SensorCanPublisher>
{

public:
	SensorCanPublisher(int max_iter) :
		_max_iter(max_iter){}
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
		px4_sem_init(&_manual_sem, 1, 1);

		//_rx = new SensorCanRx(_my_id);
		//_rx->init();
		//if (!_rx) {
		//	PX4_ERR("Failed to create RX");
		//	return -1;
		//}

		// NOTA: o socket e' criado dentro de run_loop(), que roda inteiramente
		// dentro da task dedicada deste modulo (spawnada via px4_task_spawn_cmd).
		// Antes isso rodava na work queue compartilhada hp_default, que nao tem
		// memoria reservada suficiente pra abrir um socket (ENOMEM) - por isso
		// o modulo virou uma task propria em vez de um work item.

		PX4_INFO("sensor_can_publisher init ok");

		return 0;
	}



	~SensorCanPublisher()
	{
		local_stop();
		perf_free(_loop_perf);


	}

	int setup_socket()
{
	_sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (_sock < 0) {
		PX4_ERR("socket failed, errno=%d", errno);
		return -1;
	}

	int enable = 1;

	setsockopt(_sock,
			   SOL_SOCKET,
			   SO_BROADCAST,
			   &enable,
			   sizeof(enable));

	struct sockaddr_in local{};

	local.sin_family = AF_INET;
	local.sin_port = htons(UDP_PORT);
	local.sin_addr.s_addr = INADDR_ANY;

	if (bind(_sock,
			 (sockaddr *)&local,
			 sizeof(local)) < 0) {
		PX4_ERR("bind failed, errno=%d", errno);
		return -1;
	}

	int flags = fcntl(_sock, F_GETFL, 0);
	fcntl(_sock, F_SETFL, flags | O_NONBLOCK);

	memset(&_broadcast_addr, 0, sizeof(_broadcast_addr));

	_broadcast_addr.sin_family = AF_INET;
	_broadcast_addr.sin_port = htons(UDP_PORT);
	_broadcast_addr.sin_addr.s_addr = inet_addr("192.168.0.255");
	PX4_INFO("Broadcast=%s",
         inet_ntoa(_broadcast_addr.sin_addr));
	PX4_INFO("Socket ready, fd=%d", _sock);
	send_broadcast("sensor_can_publisher started");

	return 0;
}

void send_broadcast(const char *msg)
{
	if (_sock < 0) {
		PX4_WARN("send_broadcast chamado sem socket valido (fd=%d)", _sock);
		return;
	}

	EthernetMessage packet{};

	packet.sender_id = _my_id;
	packet.counter = _tx_count;
	packet.timestamp = hrt_absolute_time();

	strncpy(packet.data,
			msg,
			MAX_MSG_SIZE - 1);

	int ret = sendto(_sock,
		   &packet,
		   sizeof(packet),
		   0,
		   (sockaddr *)&_broadcast_addr,
		   sizeof(_broadcast_addr));

	PX4_INFO("sock=%d sendto=%d errno=%d", _sock, ret, errno);

	// So conta como TX real se o sendto realmente teve sucesso.
	if (ret >= 0) {
		_tx_count++;
	}
}

void receive_messages()
{
	if (_sock < 0) {
		return;
	}

	while (true) {

		EthernetMessage packet{};

		sockaddr_in src{};
		socklen_t len = sizeof(src);

		int ret = recvfrom(_sock,
						   &packet,
						   sizeof(packet),
						   0,
						   (sockaddr *)&src,
						   &len);

		if (ret < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_INFO("recv errno=%d", errno);
			}
			break;
		}

		PX4_INFO("RX from %s id=%u cnt=%u msg=%s",
         inet_ntoa(src.sin_addr),
         (unsigned)packet.sender_id,
         (unsigned)packet.counter,
         packet.data);

		_rx_count++;

		_history[_history_index] = packet;

		_history_index++;

		if (_history_index >= MAX_RX_HISTORY) {
			_history_index = 0;
		}
	}
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



	void run_loop()
	{
		// Socket criado aqui, ja dentro da task dedicada deste modulo.
		// Diferente da work queue compartilhada (hp_default), essa task tem
		// memoria propria reservada, entao o socket() nao falha com ENOMEM.
		if (setup_socket() != 0) {
			PX4_ERR("Socket init failed inside run_loop()");
			return;
		}

		_socket_ready = true;

		while (!should_exit()) {
			perf_begin(_loop_perf);

			// Drena qualquer mensagem manual pendente (comando "send" via CLI),
			// enviando ela a partir daqui, no contexto correto do socket.
			if (_manual_pending) {
				char local_copy[MAX_MSG_SIZE];
				px4_sem_wait(&_manual_sem);
				strncpy(local_copy, _manual_msg, MAX_MSG_SIZE - 1);
				local_copy[MAX_MSG_SIZE - 1] = '\0';
				_manual_pending = false;
				px4_sem_post(&_manual_sem);

				send_broadcast(local_copy);
			}

			send_broadcast("heartbeat");

			receive_messages();

			EkfScore self{};

			update_uorb_subs();
			montar_mensage(self);

			update_local_peers();
			remove_stale_peers();
			update_leader_with_self(self);   // era: leader_id = elect_leader(self, leader_id);
			publish_ekf_score_uorb(self);
			_my_curr_score = self;
			PX4_INFO("NEW_LEADER_ID: %d", static_cast<int>(leader_id));

			if (_max_iter >= 0 && ++_count >= _max_iter) {
				perf_end(_loop_perf);
				break;
			}

			perf_end(_loop_perf);

			usleep(10000); // 10ms, equivalente ao antigo ScheduleOnInterval
		}

		PX4_INFO("run_loop encerrando");

		if (_sock >= 0) {
			close(_sock);
			_sock = -1;
		}
	}





	void local_stop()
	{
		request_stop();

		// da' tempo pro run_loop() (rodando na task dedicada) notar
		// should_exit() e sair do while de forma limpa, fechando o socket
		// dele mesmo antes de a gente destruir o objeto.
		usleep(50000);

		px4_sem_destroy(&peers_sem);
		px4_sem_destroy(&_manual_sem);

		if (_sock >= 0) {
			close(_sock);
			_sock = -1;
		}

		//if (_rx) {
		//	delete _rx;
		//	_rx = nullptr;
		//}
	}

	int send_manual(const char *msg)
	{
		// IMPORTANTE: este metodo e' chamado a partir da task do NSH shell
		// (comando "sensor_can_publisher send ..."), que e' uma task
		// diferente da task dedicada do modulo onde o _sock foi criado.
		// Nao da' pra chamar sendto() direto aqui - o fd seria invalido
		// nesse contexto (mesmo erro EBADF que tinhamos antes). Em vez
		// disso, so' deixamos a mensagem "pendente" e quem manda de
		// verdade e' o run_loop(), rodando no contexto correto, na
		// proxima iteracao (ate 10ms de atraso).
		px4_sem_wait(&_manual_sem);
		strncpy(_manual_msg, msg, MAX_MSG_SIZE - 1);
		_manual_msg[MAX_MSG_SIZE - 1] = '\0';
		_manual_pending = true;
		px4_sem_post(&_manual_sem);
		return 0;
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

	void update_local_peers(){
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
						peer.score   = ekfScoreFromUorb(rx);
						peer.last_rx = rx_time;

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
	}

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

	void update_leader_with_self(const EkfScore &self)
	{
		if (leader_id < 0 || _leader_lost) {
			elect_leader_from_scratch(self);
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
int print_messages()
{
	PX4_INFO("TX=%u RX=%u",
         static_cast<unsigned>(_tx_count),
         static_cast<unsigned>(_rx_count));

	for (int i = 0; i < MAX_RX_HISTORY; i++) {

		const EthernetMessage &m = _history[i];

		if (m.timestamp == 0) {
			continue;
		}

		PX4_INFO("[%d] id=%u cnt=%u msg=%s",
				 i,
				 static_cast<unsigned>(m.sender_id),
				 static_cast<unsigned>(m.counter),
				 m.data);
	}

	return 0;
}

private:
	int _max_iter{-1};
	int _count{0};

	int32_t _my_id{-1};
	static constexpr uint64_t PEER_TIMEOUT_US = 100000; // 100 ms


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
	bool _socket_ready{false};

	/* mensagem manual pendente: escrita pela task do CLI (send_manual),
	 * consumida e enviada pela task da work queue (Run) */
	px4_sem_t _manual_sem;
	bool _manual_pending{false};
	char _manual_msg[MAX_MSG_SIZE]{};

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

	int _sock{-1};
	uint32_t _tx_count{0};
	uint32_t _rx_count{0};
	EthernetMessage _history[MAX_RX_HISTORY]{};
	int _history_index{0};
	struct sockaddr_in _broadcast_addr{};


};







static SensorCanPublisher *g_instance{nullptr};

/* Trampolim executado dentro da task dedicada spawnada por px4_task_spawn_cmd.
 * g_instance ja foi alocado e inicializado (init()) pela task que processou
 * o comando "start" antes de spawnar essa task - so' falta rodar o loop
 * principal (que abre o socket e fica nele) no contexto certo. */
static int sensor_can_publisher_task_main(int argc, char *argv[])
{
	if (g_instance) {
		g_instance->run_loop();
	}

	return 0;
}

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

			int task_id = px4_task_spawn_cmd("sensor_can_publisher",
							  SCHED_DEFAULT,
							  SCHED_PRIORITY_DEFAULT,
							  2048,
							  (px4_main_t)&sensor_can_publisher_task_main,
							  nullptr);

			if (task_id < 0) {
				PX4_ERR("task spawn failed");
				delete g_instance;
				g_instance = nullptr;
				return -1;
			}
		}


		return 0;
	}

	if (!strcmp(argv[1], "send")) {

	if (!g_instance) {
		PX4_ERR("module not running");
		return -1;
	}

	if (argc < 3) {
		PX4_ERR("Usage: sensor_can_publisher send <msg>");
		return -1;
	}

	return g_instance->send_manual(argv[2]);
}

if (!strcmp(argv[1], "messages")) {

	if (!g_instance) {
		PX4_ERR("module not running");
		return -1;
	}

	return g_instance->print_messages();
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
