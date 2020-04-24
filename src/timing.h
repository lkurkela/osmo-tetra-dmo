#ifndef OSMOTETRA_TIMING_H
#define OSMOTETRA_TIMING_H

#include <stdint.h>

#define TIMING_SLOTS 4320

struct timing_state {
	// Parameter: length of a slot
	uint64_t slot_time;
	// Parameter: how long beforehand a burst is produced
	int64_t ahead_time;

	// Next transmission slot
	unsigned tx_slot; // Combined slot number, counting from 0 to 4319
	uint64_t tx_time;
};

struct timing_slot {
	uint64_t time;    // Timestamp
	int64_t  diff;    // For RX: Time difference from expected timestamp
	unsigned char tn; // Timeslot Number (1 to 4)
	unsigned char fn; // TDMA Frame Number (1 to 18)
	unsigned char mn; // TDMA Multiframe Number (1 to 60)
	unsigned char hn; // TDMA Hyperframe Number?
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
