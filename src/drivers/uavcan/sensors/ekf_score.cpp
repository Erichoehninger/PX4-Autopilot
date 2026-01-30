#include "ekf_score.hpp"
#include <uORB/topics/ekf_score.h>

#include <drivers/drv_hrt.h>

const char *const UavcanEkfScoreBridge::NAME = "ekf_score";

UavcanEkfScoreBridge::UavcanEkfScoreBridge(uavcan::INode &node,
					   NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_ekf_score", ORB_ID(ekf_score), node_info_publisher),
	_sub(node)
{
}


int UavcanEkfScoreBridge::init()
{
	return _sub.start(EkfScoreCbBinder(this, &UavcanEkfScoreBridge::ekf_score_cb));
}



void UavcanEkfScoreBridge::ekf_score_cb(
	const uavcan::ReceivedDataStructure<globaldrones::EkfScore> &msg)
{
	ekf_score_s s{};
	s.timestamp = hrt_absolute_time();

	s.instance_id = msg.instance_id;
	s.vel_test   = msg.vel_test;
	s.pos_test   = msg.pos_test;
	s.hgt_test   = msg.hgt_test;
	s.hdg_test   = msg.hdg_test;
	s.pos_var    = msg.pos_var;
	s.vel_var    = msg.vel_var;
	s.ekf_flags  = msg.ekf_flags;
	s.nav_state  = msg.nav_state;
	s.timestamp_utc = msg.timestamp_utc;

	_ekf_score_pub.publish(s);
}

