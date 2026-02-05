#pragma once

#include "sensor_bridge.hpp"
#include <uavcan/uavcan.hpp>
#include <globaldrones/EkfScore.hpp>
#include <uORB/topics/ekf_score.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>


class UavcanEkfScoreBridge : public UavcanSensorBridgeBase //RX || CAN->uORB
{
public:
	static const char *const NAME;

	UavcanEkfScoreBridge(uavcan::INode &node,
			     NodeInfoPublisher *node_info_publisher);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	void ekf_score_cb(
		const uavcan::ReceivedDataStructure<globaldrones::EkfScore> &msg
	);

	typedef uavcan::MethodBinder<
		UavcanEkfScoreBridge *,
		void (UavcanEkfScoreBridge::*)(
			const uavcan::ReceivedDataStructure<globaldrones::EkfScore> &
		)
	> EkfScoreCbBinder;
	uORB::PublicationMulti<ekf_score_s> _ekf_score_pub{ORB_ID(ekf_score)}; // PUBLISHER DE UORB
	uavcan::Subscriber<globaldrones::EkfScore, EkfScoreCbBinder> _sub; // SUBSCRIBER UAVCAN
};

class UavcanEkfScoreTxBridge : public UavcanSensorBridgeBase //TX || uORB->CAN
{
public:
	static const char *const NAME;

	UavcanEkfScoreTxBridge(uavcan::INode &node,
			       NodeInfoPublisher *node_info_publisher);

	int init() override;
	void update();
	const char *get_name() const override { return NAME; }

private:
	uORB::Subscription _sub_ekf_score{ORB_ID(ekf_score)}; // SUBSCRIBER DE UORB
	uavcan::Publisher<globaldrones::EkfScore> _pub; // PUBLISHER DE UAVCAN
};
