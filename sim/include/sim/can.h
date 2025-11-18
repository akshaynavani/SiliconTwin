/*
 * CAN 2.0A bus emulation.
 *
 * Models the parts of CAN that firmware actually has to cope with:
 *   - bitwise arbitration (lowest identifier wins when several nodes have a
 *     frame pending at the same time),
 *   - frame duration derived from the bitrate, including average bit stuffing,
 *   - per-node hardware acceptance filters (id/mask) and a bounded RX FIFO,
 *   - fault confinement: TEC/REC error counters with error-active,
 *     error-passive and bus-off states, automatic retransmission and the
 *     128 x 11 recessive bit bus-off recovery sequence,
 *   - fault injection through a per-frame bus error probability.
 */
#ifndef SIM_CAN_H
#define SIM_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "sim/clock.h"
#include "sim/rng.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_MAX_NODES 4
#define CAN_TX_QUEUE_DEPTH 8
#define CAN_RX_FIFO_DEPTH 16
#define CAN_NUM_FILTERS 4
#define CAN_LOG_SIZE 512
#define CAN_ID_MASK 0x7FFu

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum {
    CAN_STATE_ERROR_ACTIVE = 0,
    CAN_STATE_ERROR_PASSIVE = 1,
    CAN_STATE_BUS_OFF = 2
} can_state_t;

typedef struct {
    uint16_t id;
    uint16_t mask;
    bool enabled;
} can_filter_t;

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_errors;
    uint32_t tx_dropped;   /* send() rejected: queue full or bus-off */
    uint32_t arbitration_lost;
    uint32_t rx_ok;
    uint32_t rx_filtered;
    uint32_t rx_overruns;
    uint32_t bus_off_events;
} can_node_stats_t;

typedef struct {
    char name[16];
    bool attached;
    bool enabled;

    can_frame_t tx_queue[CAN_TX_QUEUE_DEPTH];
    uint8_t tx_head, tx_count;
    can_frame_t rx_fifo[CAN_RX_FIFO_DEPTH];
    uint8_t rx_head, rx_count;

    can_filter_t filters[CAN_NUM_FILTERS];

    uint16_t tec;
    uint16_t rec;
    can_state_t state;
    bool recovery_requested;
    sim_time_us recovery_at;

    can_node_stats_t stats;
} can_node_t;

typedef struct {
    sim_time_us timestamp_us;
    uint8_t source;
    can_frame_t frame;
} can_log_entry_t;

typedef struct {
    uint32_t bitrate;
    can_node_t nodes[CAN_MAX_NODES];
    uint8_t num_nodes;

    sim_time_us busy_until;
    double error_rate;
    sim_rng_t *rng;

    can_log_entry_t log[CAN_LOG_SIZE];
    uint32_t log_head;
    uint32_t log_total;

    uint32_t frames_ok;
    uint32_t error_frames;
} can_bus_t;

void can_bus_init(can_bus_t *bus, uint32_t bitrate, sim_rng_t *rng);
/* Attaches a node and returns its index, or -1 if the bus is full. */
int can_bus_attach(can_bus_t *bus, const char *name);
void can_bus_step(can_bus_t *bus, sim_time_us now);
void can_bus_set_error_rate(can_bus_t *bus, double probability);
sim_time_us can_frame_duration_us(const can_bus_t *bus, uint8_t dlc);

/* Node operations (firmware HAL or test bench). */
void can_node_reset(can_bus_t *bus, uint8_t node);
void can_node_enable(can_bus_t *bus, uint8_t node, bool enabled);
bool can_node_set_filter(can_bus_t *bus, uint8_t node, uint8_t index, uint16_t id, uint16_t mask);
bool can_node_send(can_bus_t *bus, uint8_t node, const can_frame_t *frame);
bool can_node_receive(can_bus_t *bus, uint8_t node, can_frame_t *frame);
can_state_t can_node_state(const can_bus_t *bus, uint8_t node);
void can_node_request_recovery(can_bus_t *bus, uint8_t node, sim_time_us now);
/* Fault injection: drive the node straight into bus-off. */
void can_node_force_bus_off(can_bus_t *bus, uint8_t node);

/* Iterate the transmitted-frame log, oldest first. Returns entry count. */
uint32_t can_bus_log_count(const can_bus_t *bus);
const can_log_entry_t *can_bus_log_at(const can_bus_t *bus, uint32_t i);

const char *can_state_name(can_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CAN_H */
