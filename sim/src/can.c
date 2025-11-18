#include "sim/can.h"

#include <stdio.h>
#include <string.h>

/* Error counter thresholds from ISO 11898-1 fault confinement. */
#define CAN_PASSIVE_THRESHOLD 128u
#define CAN_BUS_OFF_THRESHOLD 256u
#define CAN_RECOVERY_BITS (128u * 11u)

static bool valid_node(const can_bus_t *bus, uint8_t node)
{
    return node < bus->num_nodes && bus->nodes[node].attached;
}

static void update_state(can_node_t *n)
{
    if (n->state == CAN_STATE_BUS_OFF) {
        return; /* only the recovery sequence leaves bus-off */
    }
    if (n->tec >= CAN_BUS_OFF_THRESHOLD) {
        n->state = CAN_STATE_BUS_OFF;
        n->stats.bus_off_events++;
        n->tx_count = 0; /* pending transmissions are aborted */
        n->recovery_requested = false;
    } else if (n->tec >= CAN_PASSIVE_THRESHOLD || n->rec >= CAN_PASSIVE_THRESHOLD) {
        n->state = CAN_STATE_ERROR_PASSIVE;
    } else {
        n->state = CAN_STATE_ERROR_ACTIVE;
    }
}

static bool filters_accept(const can_node_t *n, uint16_t id)
{
    bool any_enabled = false;
    for (int i = 0; i < CAN_NUM_FILTERS; i++) {
        const can_filter_t *f = &n->filters[i];
        if (!f->enabled) {
            continue;
        }
        any_enabled = true;
        if ((id & f->mask) == (f->id & f->mask)) {
            return true;
        }
    }
    /* With no filters configured the controller accepts everything. */
    return !any_enabled;
}

static void log_frame(can_bus_t *bus, sim_time_us now, uint8_t src, const can_frame_t *f)
{
    can_log_entry_t *e = &bus->log[bus->log_head];
    e->timestamp_us = now;
    e->source = src;
    e->frame = *f;
    bus->log_head = (bus->log_head + 1u) % CAN_LOG_SIZE;
    bus->log_total++;
}

void can_bus_init(can_bus_t *bus, uint32_t bitrate, sim_rng_t *rng)
{
    memset(bus, 0, sizeof(*bus));
    bus->bitrate = bitrate ? bitrate : 500000;
    bus->rng = rng;
}

int can_bus_attach(can_bus_t *bus, const char *name)
{
    if (bus->num_nodes >= CAN_MAX_NODES) {
        return -1;
    }
    uint8_t idx = bus->num_nodes++;
    can_node_t *n = &bus->nodes[idx];
    memset(n, 0, sizeof(*n));
    snprintf(n->name, sizeof(n->name), "%s", name ? name : "node");
    n->attached = true;
    n->enabled = true;
    return idx;
}

void can_bus_set_error_rate(can_bus_t *bus, double probability)
{
    bus->error_rate = probability;
}

sim_time_us can_frame_duration_us(const can_bus_t *bus, uint8_t dlc)
{
    /* Standard frame: 44 overhead bits + 3 intermission + payload, plus an
     * average of one stuff bit per five bits over the stuffable region. */
    uint32_t payload = 8u * dlc;
    uint32_t bits = 47u + payload + (34u + payload) / 5u;
    sim_time_us us = ((sim_time_us)bits * 1000000ULL + bus->bitrate - 1) / bus->bitrate;
    return us ? us : 1;
}

static void deliver(can_bus_t *bus, uint8_t src, const can_frame_t *f)
{
    for (uint8_t i = 0; i < bus->num_nodes; i++) {
        can_node_t *n = &bus->nodes[i];
        if (i == src || !n->attached || !n->enabled || n->state == CAN_STATE_BUS_OFF) {
            continue;
        }
        /* Successful reception decrements REC (re-entering the active range). */
        if (n->rec > 0) {
            n->rec = (n->rec > 127u) ? 120u : (uint16_t)(n->rec - 1u);
        }
        update_state(n);
        if (!filters_accept(n, f->id)) {
            n->stats.rx_filtered++;
            continue;
        }
        if (n->rx_count >= CAN_RX_FIFO_DEPTH) {
            n->stats.rx_overruns++;
            continue;
        }
        uint8_t slot = (uint8_t)((n->rx_head + n->rx_count) % CAN_RX_FIFO_DEPTH);
        n->rx_fifo[slot] = *f;
        n->rx_count++;
        n->stats.rx_ok++;
    }
}

static uint8_t count_listeners(const can_bus_t *bus, uint8_t src)
{
    uint8_t c = 0;
    for (uint8_t i = 0; i < bus->num_nodes; i++) {
        const can_node_t *n = &bus->nodes[i];
        if (i != src && n->attached && n->enabled && n->state != CAN_STATE_BUS_OFF) {
            c++;
        }
    }
    return c;
}

void can_bus_step(can_bus_t *bus, sim_time_us now)
{
    sim_time_us bit_us = (1000000ULL + bus->bitrate - 1) / bus->bitrate;

    /* Bus-off recovery runs regardless of bus activity in this model. */
    for (uint8_t i = 0; i < bus->num_nodes; i++) {
        can_node_t *n = &bus->nodes[i];
        if (n->state == CAN_STATE_BUS_OFF && n->recovery_requested && now >= n->recovery_at) {
            n->tec = 0;
            n->rec = 0;
            n->state = CAN_STATE_ERROR_ACTIVE;
            n->recovery_requested = false;
        }
    }

    if (now < bus->busy_until) {
        return;
    }

    /* Arbitration: every node with a pending frame competes; lowest ID wins. */
    int winner = -1;
    uint16_t best_id = 0xFFFF;
    uint8_t contenders = 0;
    for (uint8_t i = 0; i < bus->num_nodes; i++) {
        can_node_t *n = &bus->nodes[i];
        if (!n->attached || !n->enabled || n->state == CAN_STATE_BUS_OFF || n->tx_count == 0) {
            continue;
        }
        contenders++;
        uint16_t id = n->tx_queue[n->tx_head].id;
        if (id < best_id) {
            best_id = id;
            winner = i;
        }
    }
    if (winner < 0) {
        return;
    }
    if (contenders > 1) {
        for (uint8_t i = 0; i < bus->num_nodes; i++) {
            can_node_t *n = &bus->nodes[i];
            if ((int)i != winner && n->attached && n->enabled && n->state != CAN_STATE_BUS_OFF &&
                n->tx_count > 0) {
                n->stats.arbitration_lost++;
            }
        }
    }

    can_node_t *tx = &bus->nodes[winner];
    can_frame_t frame = tx->tx_queue[tx->tx_head];
    sim_time_us duration = can_frame_duration_us(bus, frame.dlc);

    bool ack_missing = count_listeners(bus, (uint8_t)winner) == 0;
    bool bus_error = bus->rng && sim_rng_chance(bus->rng, bus->error_rate);

    if (ack_missing || bus_error) {
        /* Error frame aborts the transmission partway; the frame stays queued
         * for automatic retransmission. */
        bus->error_frames++;
        tx->stats.tx_errors++;
        /* Exception: an error-passive transmitter does not increment TEC on
         * an ACK error, so a lone node parks in error-passive. */
        if (!(ack_missing && tx->state == CAN_STATE_ERROR_PASSIVE)) {
            tx->tec = (uint16_t)(tx->tec + 8u);
        }
        if (bus_error) {
            for (uint8_t i = 0; i < bus->num_nodes; i++) {
                can_node_t *n = &bus->nodes[i];
                if ((int)i != winner && n->attached && n->enabled && n->state != CAN_STATE_BUS_OFF) {
                    n->rec = (uint16_t)(n->rec + 1u);
                    update_state(n);
                }
            }
        }
        update_state(tx);
        bus->busy_until = now + duration / 2 + 20u * bit_us;
        return;
    }

    /* Success. */
    tx->tx_head = (uint8_t)((tx->tx_head + 1u) % CAN_TX_QUEUE_DEPTH);
    tx->tx_count--;
    tx->stats.tx_ok++;
    if (tx->tec > 0) {
        tx->tec--;
    }
    update_state(tx);
    bus->frames_ok++;
    log_frame(bus, now, (uint8_t)winner, &frame);
    deliver(bus, (uint8_t)winner, &frame);
    bus->busy_until = now + duration;
}

void can_node_reset(can_bus_t *bus, uint8_t node)
{
    if (!valid_node(bus, node)) {
        return;
    }
    can_node_t *n = &bus->nodes[node];
    n->tx_head = n->tx_count = 0;
    n->rx_head = n->rx_count = 0;
    memset(n->filters, 0, sizeof(n->filters));
    n->tec = 0;
    n->rec = 0;
    n->state = CAN_STATE_ERROR_ACTIVE;
    n->recovery_requested = false;
    n->enabled = false;
}

void can_node_enable(can_bus_t *bus, uint8_t node, bool enabled)
{
    if (valid_node(bus, node)) {
        bus->nodes[node].enabled = enabled;
    }
}

bool can_node_set_filter(can_bus_t *bus, uint8_t node, uint8_t index, uint16_t id, uint16_t mask)
{
    if (!valid_node(bus, node) || index >= CAN_NUM_FILTERS) {
        return false;
    }
    can_filter_t *f = &bus->nodes[node].filters[index];
    f->id = (uint16_t)(id & CAN_ID_MASK);
    f->mask = (uint16_t)(mask & CAN_ID_MASK);
    f->enabled = true;
    return true;
}

bool can_node_send(can_bus_t *bus, uint8_t node, const can_frame_t *frame)
{
    if (!valid_node(bus, node) || frame->dlc > 8) {
        return false;
    }
    can_node_t *n = &bus->nodes[node];
    if (!n->enabled || n->state == CAN_STATE_BUS_OFF || n->tx_count >= CAN_TX_QUEUE_DEPTH) {
        n->stats.tx_dropped++;
        return false;
    }
    uint8_t slot = (uint8_t)((n->tx_head + n->tx_count) % CAN_TX_QUEUE_DEPTH);
    n->tx_queue[slot] = *frame;
    n->tx_queue[slot].id &= CAN_ID_MASK;
    n->tx_count++;
    return true;
}

bool can_node_receive(can_bus_t *bus, uint8_t node, can_frame_t *frame)
{
    if (!valid_node(bus, node)) {
        return false;
    }
    can_node_t *n = &bus->nodes[node];
    if (n->rx_count == 0) {
        return false;
    }
    *frame = n->rx_fifo[n->rx_head];
    n->rx_head = (uint8_t)((n->rx_head + 1u) % CAN_RX_FIFO_DEPTH);
    n->rx_count--;
    return true;
}

can_state_t can_node_state(const can_bus_t *bus, uint8_t node)
{
    if (!valid_node(bus, node)) {
        return CAN_STATE_BUS_OFF;
    }
    return bus->nodes[node].state;
}

void can_node_request_recovery(can_bus_t *bus, uint8_t node, sim_time_us now)
{
    if (!valid_node(bus, node)) {
        return;
    }
    can_node_t *n = &bus->nodes[node];
    if (n->state != CAN_STATE_BUS_OFF || n->recovery_requested) {
        return;
    }
    sim_time_us bit_us = (1000000ULL + bus->bitrate - 1) / bus->bitrate;
    n->recovery_requested = true;
    n->recovery_at = now + (sim_time_us)CAN_RECOVERY_BITS * bit_us;
}

void can_node_force_bus_off(can_bus_t *bus, uint8_t node)
{
    if (!valid_node(bus, node)) {
        return;
    }
    can_node_t *n = &bus->nodes[node];
    n->tec = CAN_BUS_OFF_THRESHOLD;
    update_state(n);
}

uint32_t can_bus_log_count(const can_bus_t *bus)
{
    return bus->log_total < CAN_LOG_SIZE ? bus->log_total : CAN_LOG_SIZE;
}

const can_log_entry_t *can_bus_log_at(const can_bus_t *bus, uint32_t i)
{
    uint32_t count = can_bus_log_count(bus);
    if (i >= count) {
        return NULL;
    }
    uint32_t oldest = (bus->log_total < CAN_LOG_SIZE) ? 0u : bus->log_head;
    return &bus->log[(oldest + i) % CAN_LOG_SIZE];
}

const char *can_state_name(can_state_t s)
{
    switch (s) {
    case CAN_STATE_ERROR_ACTIVE: return "error_active";
    case CAN_STATE_ERROR_PASSIVE: return "error_passive";
    case CAN_STATE_BUS_OFF: return "bus_off";
    }
    return "unknown";
}
