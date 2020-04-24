#ifndef TETRA_SLOTTER_H
#define TETRA_SLOTTER_H

#include "timing.h"

struct timing_state;

struct slotter_state {
	//struct timing_state *timing;

	uint64_t prev_burst_time; // for test
	// TODO
};

/* Allocate and initialize a timing state.
 * Parameters TODO. */
struct slotter_state *slotter_init();

/* Process a received burst in a given slot. */
int slotter_rx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot);

/* Produce a burst to be transmitted in a given slot.
 * Return value is the number of bits in the burst,
 * -1 if there is no burst to transmit in that slot.
 */
int slotter_tx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot);

#endif
