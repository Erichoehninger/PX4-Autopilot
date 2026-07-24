/****************************************************************************
 *
 *   TMR CAN Manager Module Header
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/ekf_score.h>

#include "tmr_can_protocol.h"

struct TmrNodeState {
	TmrCanScoreFrame score_frame{};
	hrt_abstime      last_seen{0};
	bool             valid{false};
};

class TmrCanManager :
	public ModuleBase<TmrCanManager>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	TmrCanManager();
	~TmrCanManager() override;

	static int task_spawn(int argc, char *argv[]);
	static TmrCanManager *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	void Run() override;

	int print_status() override;

private:
	static constexpr int MAX_TMR_NODES = 3;

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _ekf_score_sub{ORB_ID(ekf_score)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};

	uORB::Publication<vehicle_odometry_s> _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_PX4_COMM_ID>) _param_sys_comm_id
	)

	uint8_t      _node_id{0};
	uint8_t      _current_leader_id{0};
	hrt_abstime  _last_leader_change{0};
	float        _self_score{1.0f};

	TmrNodeState _nodes[MAX_TMR_NODES]{};
	uint8_t      _odom_seq{0};

	void EvaluateLeaderConsensus();
	float CalculateLocalScore();
	void PublishLocalScore();
	void BroadcastLeaderOdometry();
	void ProcessIncomingCanFrames();
};
