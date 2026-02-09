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
	s.timestamp = hrt_absolute_time();

	s.instance_id = msg.instance_id;

	s.vel_test = msg.vel_test;
	s.pos_test = msg.pos_test;
	s.hgt_test = msg.hgt_test;
	s.hdg_test = msg.hdg_test;

	s.pos_var = msg.pos_var;
	s.vel_var = msg.vel_var;

	s.ekf_flags = msg.ekf_flags;
	s.nav_state = msg.nav_state;

	s.timestamp_utc = msg.timestamp_utc;

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

	msg.vel_test = s.vel_test;
	msg.pos_test = s.pos_test;
	msg.hgt_test = s.hgt_test;
	msg.hdg_test = s.hdg_test;

	msg.pos_var = s.pos_var;
	msg.vel_var = s.vel_var;

	msg.ekf_flags = s.ekf_flags;
	msg.nav_state = s.nav_state;

	msg.timestamp_utc = s.timestamp_utc;

	int res = _pub.broadcast(msg);

	PX4_WARN("LeaderPublishableInfo CAN publish res: %d", res);

	if (res < 0) {
		PX4_WARN("LeaderPublishableInfo CAN publish failed: %d", res);
	}
}
