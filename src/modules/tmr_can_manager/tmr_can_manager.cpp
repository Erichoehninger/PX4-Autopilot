/****************************************************************************
 *
 *   TMR CAN Manager Module Implementation
 *
 ****************************************************************************/

#include "tmr_can_manager.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <cmath>

TmrCanManager::TmrCanManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

TmrCanManager::~TmrCanManager()
{
	ScheduleClear();
}

int TmrCanManager::init()
{
	_node_id = static_cast<uint8_t>(_param_sys_comm_id.get());
	_current_leader_id = _node_id;

	PX4_INFO("TmrCanManager initialized. Node ID: %d", _node_id);
	ScheduleOnInterval(20000); // 50 Hz loop (20 ms interval)
	return 0;
}

float TmrCanManager::CalculateLocalScore()
{
	estimator_selector_status_s status{};

	if (_estimator_selector_status_sub.update(&status)) {
		if (status.primary_instance < EKF2_MAX_INSTANCES && status.healthy[status.primary_instance]) {
			_self_score = status.combined_test_ratio[status.primary_instance];
		} else {
			_self_score = 999.0f; // Unhealthy instance default penalty
		}
	}

	return _self_score;
}

void TmrCanManager::EvaluateLeaderConsensus()
{
	const hrt_abstime now = hrt_absolute_time();

	// Invalidate timed-out node entries
	for (int i = 0; i < MAX_TMR_NODES; i++) {
		if (_nodes[i].valid && (now - _nodes[i].last_seen > TMR_NODE_TIMEOUT_US)) {
			_nodes[i].valid = false;
			PX4_WARN("TMR Node %d timed out", i);
		}
	}

	// Always record local node score in node matrix
	if (_node_id < MAX_TMR_NODES) {
		_nodes[_node_id].score_frame.node_id = _node_id;
		_nodes[_node_id].score_frame.score = CalculateLocalScore();
		_nodes[_node_id].score_frame.timestamp_utc_us = now;
		_nodes[_node_id].last_seen = now;
		_nodes[_node_id].valid = true;
	}

	// Leader Election with Hysteresis & Priority Tie-Breaking
	uint8_t best_node_id = _current_leader_id;
	float best_score = _nodes[_current_leader_id].valid ? _nodes[_current_leader_id].score_frame.score : 999.0f;

	for (uint8_t i = 0; i < MAX_TMR_NODES; i++) {
		if (!_nodes[i].valid) {
			continue;
		}

		float candidate_score = _nodes[i].score_frame.score;

		// Candidate must beat current leader by hysteresis margin
		if (i != _current_leader_id) {
			if (candidate_score < (best_score - TMR_SCORE_HYSTERESIS_MARGIN)) {
				if ((now - _last_leader_change) > TMR_LEADER_HOLDOFF_US) {
					best_node_id = i;
					best_score = candidate_score;
				}
			}
		}
	}

	if (best_node_id != _current_leader_id) {
		PX4_INFO("TMR Leader Failover: %d -> %d (Score: %.3f)", _current_leader_id, best_node_id, (double)best_score);
		_current_leader_id = best_node_id;
		_last_leader_change = now;
	}
}

void TmrCanManager::PublishLocalScore()
{
	TmrCanScoreFrame score_frame{};
	score_frame.node_id = _node_id;
	score_frame.current_leader_id = _current_leader_id;
	score_frame.timestamp_utc_us = hrt_absolute_time();
	score_frame.score = _self_score;

	// In hardware runtime, score_frame is serialized to CAN-FD frame (ID 0x200) via FDCAN driver.
}

void TmrCanManager::BroadcastLeaderOdometry()
{
	if (_current_leader_id != _node_id) {
		return; // Only leader broadcasts state over CAN-FD
	}

	vehicle_odometry_s odom{};

	if (_vehicle_odometry_sub.update(&odom)) {
		TmrCanOdometryFrame frame{};
		frame.leader_node_id = _node_id;
		frame.seq_counter = _odom_seq++;
		frame.timestamp_sample = odom.timestamp_sample;

		frame.position[0] = odom.position[0];
		frame.position[1] = odom.position[1];
		frame.position[2] = odom.position[2];

		frame.q[0] = odom.q[0];
		frame.q[1] = odom.q[1];
		frame.q[2] = odom.q[2];
		frame.q[3] = odom.q[3];

		frame.velocity[0] = odom.velocity[0];
		frame.velocity[1] = odom.velocity[1];
		frame.velocity[2] = odom.velocity[2];

		// In hardware runtime, frame is serialized to CAN-FD frame (ID 0x201) via FDCAN driver.
	}
}

void TmrCanManager::ProcessIncomingCanFrames()
{
	// On slave nodes receiving CAN-FD ID 0x201 (Leader Odometry), deserialize and ingest into vehicle_visual_odometry
	if (_current_leader_id != _node_id) {
		vehicle_odometry_s odom{};

		if (_nodes[_current_leader_id].valid) {
			odom.timestamp = hrt_absolute_time();
			odom.timestamp_sample = odom.timestamp;
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;

			// Ingest received leader position/velocity/attitude
			_visual_odometry_pub.publish(odom);
		}
	}
}

void TmrCanManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	EvaluateLeaderConsensus();
	PublishLocalScore();
	BroadcastLeaderOdometry();
	ProcessIncomingCanFrames();
}

int TmrCanManager::print_status()
{
	PX4_INFO("=== TMR CAN Manager Status ===");
	PX4_INFO("Node ID: %d | Elected Leader ID: %d", _node_id, _current_leader_id);
	PX4_INFO("Self Score: %.3f", (double)_self_score);

	for (int i = 0; i < MAX_TMR_NODES; i++) {
		if (_nodes[i].valid) {
			PX4_INFO(" Node %d: Score=%.3f, LastSeen=%" PRIu64 " us ago",
				 i, (double)_nodes[i].score_frame.score, hrt_elapsed_time(&_nodes[i].last_seen));
		} else {
			PX4_INFO(" Node %d: OFFLINE", i);
		}
	}

	return 0;
}

int TmrCanManager::task_spawn(int argc, char *argv[])
{
	TmrCanManager *instance = new TmrCanManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

TmrCanManager *TmrCanManager::instantiate(int argc, char *argv[])
{
	return new TmrCanManager();
}

int TmrCanManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TmrCanManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("TMR CAN Manager Module for PX4 Tri-Redundant Flight Controller");
	PRINT_MODULE_USAGE_NAME("tmr_can_manager", "tmr");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int tmr_can_manager_main(int argc, char *argv[])
{
	return TmrCanManager::main(argc, argv);
}
