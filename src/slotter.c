#include "slotter.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

struct timing_state *timing1;

struct slotter_state *slotter_init() {
	struct slotter_state *s;
	s = calloc(1, sizeof(*s));

	// TODO

	return s;
}

int slotter_rx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot)
{
	printf("RX slot: %2u %2u %2u, diff %10ld ns\n", slot->tn, slot->fn, slot->mn, slot->diff);

	/* Just a test: if no bursts have been received in a while,
	 * synchronize timing to the first received burst. */
	if (slot->time - s->prev_burst_time > 1000000000UL) {
		printf("Resynchronizing\n");
		struct timing_slot sync_slot = {
			.time = slot->time,
			.diff = 0, // not used
			.tn = 1,
			.fn = 1,
			.mn = 1
		};
		timing_resync(timing1, &sync_slot);
	}

	s->prev_burst_time = slot->time;
	return 0;
}


int slotter_tx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot)
{
	//printf("TX slot: %2u %2u %2u\n", slot->tn, slot->fn, slot->mn);

	// TODO

	return -1;
}
