#include "leader_publishable_info.hpp"

#include <uORB/topics/leader_publishable_info.h>
#include <drivers/drv_hrt.h>

/* ========================== RX (CAN -> uORB) ========================== */

const char *const UavcanLeaderPublishableInfoBridge::NAME =
	"leader_publishable_info";

UavcanLeaderPublishableInfoBridge::UavcanLeaderPublishableInfoBridge(
	uavcan::INode &node,
	NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase(
		"uavcan_leader_publishable_info",
		ORB_ID(leader_publishable_info),
		node_info_publisher),
	_sub(node)
{
}

int UavcanLeaderPublishableInfoBridge::init()
{
	return _sub.start(
		LeaderPublishableInfoCbBinder(
			this,
			&UavcanLeaderPublishableInfoBridge::leader_publishable_info_cb));
}

void UavcanLeaderPublishableInfoBridge::leader_publishable_info_cb(
	const uavcan::ReceivedDataStructure<
		globaldrones::LeaderPublishableInfo> &msg)
{
	leader_publishable_info_s s{};
	s.instance_id = msg.instance_id;
	s.timestamp = hrt_absolute_time();
	s.timestamp_sample = msg.timestamp_sample;
	s.pose_frame     = msg.pose_frame;
	s.velocity_frame = msg.velocity_frame;
	for (int i = 0; i < 3; i++) {
		s.position[i] = msg.position[i];
	}
	for (int i = 0; i < 4; i++) {
		s.q[i] = msg.q[i];
	}
	for (int i = 0; i < 3; i++) {
		s.velocity[i]         = msg.velocity[i];
		s.angular_velocity[i] = msg.angular_velocity[i];
	}

	// variances
	for (int i = 0; i < 3; i++) {
		s.position_variance[i]    = msg.position_variance[i];
		s.orientation_variance[i] = msg.orientation_variance[i];
		s.velocity_variance[i]    = msg.velocity_variance[i];
	}

	s.reset_counter = msg.reset_counter;
	s.quality       = msg.quality;

	_leader_publishable_info_pub.publish(s);
}

/* ========================== TX (uORB -> CAN) ========================== */

const char *const UavcanLeaderPublishableInfoTxBridge::NAME =
	"leader_publishable_info_tx";

UavcanLeaderPublishableInfoTxBridge::UavcanLeaderPublishableInfoTxBridge(
	uavcan::INode &node,
	NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase(
		"uavcan_leader_publishable_info_tx",
		ORB_ID(leader_publishable_info),
		node_info_publisher),
	_pub(node)
{
}

int UavcanLeaderPublishableInfoTxBridge::init()
{
	return _pub.init();
}

void UavcanLeaderPublishableInfoTxBridge::update()
{
	leader_publishable_info_s s{};

	if (!_sub_leader_publishable_info.update(&s)) {
		return;
	}

	globaldrones::LeaderPublishableInfo msg{};

	msg.instance_id = s.instance_id;

	msg.timestamp = s.timestamp;
	msg.timestamp_sample = s.timestamp_sample;
	msg.pose_frame = s.pose_frame;
	msg.velocity_frame = s.velocity_frame;
	for (int i = 0; i < 3; i++) {
		msg.position[i] = s.position[i];
	}
	for (int i = 0; i < 4; i++) {
		msg.q[i] = s.q[i];
	}
	for (int i = 0; i < 3; i++) {
		msg.velocity[i]         = s.velocity[i];
		msg.angular_velocity[i] = s.angular_velocity[i];
	}

	// variances
	for (int i = 0; i < 3; i++) {
		msg.position_variance[i]    = s.position_variance[i];
		msg.orientation_variance[i] = s.orientation_variance[i];
		msg.velocity_variance[i]    = s.velocity_variance[i];
	}

	msg.reset_counter = s.reset_counter;
	msg.quality       = s.quality;
	int res = _pub.broadcast(msg);

	//PX4_WARN("LeaderPublishableInfo CAN publish res: %d", res);

	if (res < 0) {
		PX4_WARN("LeaderPublishableInfo CAN publish failed: %d", res);
	}
}
