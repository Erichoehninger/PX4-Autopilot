/****************************************************************************
 *
 *   TMR CAN Protocol Definitions for PX4 TMR Topology
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

#pragma pack(push, 1)

/**
 * @brief CAN-FD Score Broadcast Message (16 bytes)
 * Broadcasted by every node at ~50 Hz.
 */
struct TmrCanScoreFrame {
	uint8_t  node_id;           // Node Identifier (0, 1, 2)
	uint8_t  current_leader_id; // Node ID of current recognized leader
	uint16_t ekf_flags;         // EKF status flags
	uint64_t timestamp_utc_us;  // GPS/System synchronized timestamp [us]
	float    score;             // Combined EKF quality score (lower is better)
};

/**
 * @brief CAN-FD Leader Odometry Broadcast Message (56 bytes)
 * Broadcasted ONLY by the elected Leader node at ~50 Hz.
 */
struct TmrCanOdometryFrame {
	uint8_t  leader_node_id;    // Node ID of the transmitting leader
	uint8_t  seq_counter;       // Monotonic sequence counter
	uint16_t flags;             // Status flags
	uint64_t timestamp_sample;  // Sample timestamp [us]
	float    position[3];       // Position X, Y, Z [m] (Local frame)
	float    q[4];              // Quaternion attitude (w, x, y, z)
	float    velocity[3];       // Velocity X, Y, Z [m/s]
};

#pragma pack(pop)

static_assert(sizeof(TmrCanScoreFrame) == 16, "TmrCanScoreFrame must be 16 bytes for 1 CAN-FD frame");
static_assert(sizeof(TmrCanOdometryFrame) == 56, "TmrCanOdometryFrame must be 56 bytes for 1 CAN-FD frame");

static constexpr uint32_t TMR_CAN_SCORE_MSG_ID = 0x200;
static constexpr uint32_t TMR_CAN_ODOM_MSG_ID  = 0x201;

static constexpr float TMR_SCORE_HYSTERESIS_MARGIN = 0.05f;
static constexpr uint64_t TMR_LEADER_HOLDOFF_US     = 200000; // 200 ms
static constexpr uint64_t TMR_NODE_TIMEOUT_US       = 100000; // 100 ms
