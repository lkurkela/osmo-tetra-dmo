#ifndef OSMOTETRA_TIMING_H
#define OSMOTETRA_TIMING_H

#include <stdint.h>

struct timing_state {
	// TODO
};

/* Allocate and initialize a timing state.
 * Parameters TODO. */
struct timing_state *timing_init();

/* Process a received burst.
 * bits is an array of values 0 and 1.
 * len is the number of array members.
 * ts is a timestamp of the received burst in nanoseconds.
 * Return value is 0 on success. */
int timing_rx_burst(struct timing_state *s, const uint8_t *bits, int len, uint64_t ts);

/* Produce a burst to be transmitted in near future.
 * Return value is the number of bits in the burst,
 * -1 if there is no burst to transmit at the moment.
 * Timestamp of the burst is returned in *ts.
 * The current time of the modulator is given in *ts. */
int timing_tx_burst(struct timing_state *s, uint8_t *bits, int maxlen, uint64_t *ts);

#endif
