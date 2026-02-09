#pragma once

#include "sensor_bridge.hpp"

#include <uavcan/uavcan.hpp>

#include <globaldrones/LeaderPublishableInfo.hpp>

#include <uORB/topics/leader_publishable_info.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

/**
 * RX || CAN -> uORB
 */
class UavcanLeaderPublishableInfoBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanLeaderPublishableInfoBridge(
		uavcan::INode &node,
		NodeInfoPublisher *node_info_publisher);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	void leader_publishable_info_cb(
		const uavcan::ReceivedDataStructure<
			globaldrones::LeaderPublishableInfo> &msg
	);

	typedef uavcan::MethodBinder<
		UavcanLeaderPublishableInfoBridge *,
		void (UavcanLeaderPublishableInfoBridge::*)(
			const uavcan::ReceivedDataStructure<
				globaldrones::LeaderPublishableInfo> &
		)
	> LeaderPublishableInfoCbBinder;

	uORB::PublicationMulti<leader_publishable_info_s>
		_leader_publishable_info_pub{ORB_ID(leader_publishable_info)};

	uavcan::Subscriber<
		globaldrones::LeaderPublishableInfo,
		LeaderPublishableInfoCbBinder
	> _sub;
};

/**
 * TX || uORB -> CAN
 */
class UavcanLeaderPublishableInfoTxBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanLeaderPublishableInfoTxBridge(
		uavcan::INode &node,
		NodeInfoPublisher *node_info_publisher);

	int init() override;
	void update();

	const char *get_name() const override { return NAME; }

private:
	uORB::Subscription
		_sub_leader_publishable_info{ORB_ID(leader_publishable_info)};

	uavcan::Publisher<globaldrones::LeaderPublishableInfo> _pub;
};
