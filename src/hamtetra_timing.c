/* Convert between burst timestamps and timeslot numbers.
 * Handle timing of producing transmit bursts.
 *
 * In terms of TETRA specification, a part of the L1 (physical layer)
 * happens here. Rest of L1 happens in the modem, which synchronizes
 * to individual received bursts based on their training sequences
 * and then delivers them here.
 */

#include "hamtetra_timing.h"
#include "hamtetra_slotter.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* DMO EN 300 396-2 - 9.4.3.3.2 Inter-slot frequency correction bits */
static const uint8_t g_bits[40] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};

/* Extra symbols added before transmission.
 * A future version of modem could send bits at half amplitude when bit 4 is set
 * and reset its phase when bit 5 is set. */
#define RAMPUP_SYMS 3
static const uint8_t rampup_bits[RAMPUP_SYMS * 2] = { 0x30,0x31, 0,1, 1,1 };
// Extra symbols added after transmission
#define RAMPDN_SYMS 1
static const uint8_t rampdn_bits[RAMPDN_SYMS * 2] = { 0x10,0x10 };


struct timing_state *timing_init()
{
	struct timing_state *s;
	s = calloc(1, sizeof(*s));

	s->sym_time = 1e9 / 18000.0 + 0.5;
	s->slot_time = 1e9 * 255.0 / 18000.0 + 0.5;
	s->use_interslot_bits = 1;
	s->use_calibration = 1;

	return s;
}


int timing_rx_burst(struct timing_state *s, const uint8_t *bits, int len, uint64_t ts)
{
	//printf("RX %20lu %20lu\n", ts, s->tx_time);
	/* If the timestamp is sufficiently close to one of previous own
	 * transmitted bursts, assume it's an "echo" and don't pass it to L2.
	 * Use it, however, to calibrate the timestamp difference between
	 * received and transmitted bursts.
	 *
	 * Some extra checking might be useful for reliable operation:
	 * if the training sequence happens to occur elsewhere in a burst,
	 * the modem may deliver a mis-synchronized burst which will spoil
	 * the calibration. This is not handled yet. */
	const int64_t reject_margin = 2000000;
	unsigned i;
	for (i = 0; i < TIMING_TX_TIMES; i++) {
		int64_t td = ts - s->tx_times[i];
		if (td > -reject_margin && td < reject_margin) {
			if (s->use_calibration)
				s->cal_time = -td;
			printf("Rejected echo, timediff %10ld ns\n", (long)td);
			return 0;
		}
	}

	// Apply calibration value before further processing the timestamp
	ts += s->cal_time;

	// Find the nearest timeslot by comparing to the transmit counters
	int64_t td_tx = ts - s->tx_time;
	float slotdiff = (float)td_tx / (float)s->slot_time;
	int intdiff = roundf(slotdiff);
	int rx_slot = intdiff + s->tx_slot;
	// Modulo in C...
	rx_slot = ((rx_slot % TIMING_SLOTS) + TIMING_SLOTS) % TIMING_SLOTS;

	struct timing_slot tslot = {
		.time = ts,
		.diff = td_tx - (int64_t)intdiff * s->slot_time,
		.tn =  (rx_slot % 4)       + 1,
		.fn = ((rx_slot / 4) % 18) + 1,
		.mn =  (rx_slot / (4*18))  + 1,
		.hn = 0
	};

	slotter_rx_burst(s->slotter, bits, len, &tslot);

	return 0;
}


int timing_tx_burst(struct timing_state *s, uint8_t *bits, int maxlen, uint64_t *ts)
{
	if (maxlen < 510)
		return -1;

	/* Extra margin to ensure that inter-slot bits can still be
	 * added to the end of the previous slot */
	const int64_t time_margin = s->sym_time * 4;

	int retlen = -1;
	uint64_t tnow = *ts;
	if (s->tx_time == 0) {
		// Initialize timing on first tick
		s->tx_time = tnow + s->slot_time*2;
	}

	const uint64_t tx_time = s->tx_time;
	int64_t tdiff = tnow - tx_time;
	if (tdiff >= -time_margin) {
		const unsigned slot = s->tx_slot;

		struct timing_slot tslot = {
			.time = tx_time,
			.diff = 0,
			.tn =  (slot % 4)       + 1,
			.fn = ((slot / 4) % 18) + 1,
			.mn =  (slot / (4*18))  + 1,
			.hn = 0
		};

		retlen = slotter_tx_burst(s->slotter, bits, maxlen, &tslot);
		int offset_syms = 0; // Burst offset from beginning of a slot
		unsigned char is_dmo = 0;
		if (retlen < 0) {
			/* No transmission.
			 * If DMO burst was previously transmitted, add ramp-down symbols
			 * right after the previous burst. */
			if (s->prev_dmo) {
				offset_syms = -3;
				memcpy(bits, rampdn_bits, RAMPDN_SYMS * 2);
				retlen = RAMPDN_SYMS * 2;
			}
		} else if (retlen == 470) {
			// DMO burst
			is_dmo = 1;
			if (s->use_interslot_bits && s->prev_dmo) {
				/* A DMO burst was transmitted in the previous slot.
				 * Add 40 interslot frequency correction bits in the beginning
				 * and place the timestamp 3 symbols before the slot boundary,
				 * so it starts right after the previous burst. */
				offset_syms = -3;
				memmove(bits + 40, bits, retlen);
				memcpy(bits, g_bits, 40);
				retlen += 40;
			} else {
				/* DMO burst starts 34 bits (17 symbols) after slot boundary.
				 * Add extra ramp-up symbols before it. */
				offset_syms = 17 - RAMPUP_SYMS;
				memmove(bits + RAMPUP_SYMS * 2, bits, retlen);
				memcpy(bits, rampup_bits, RAMPUP_SYMS * 2);
				retlen += RAMPUP_SYMS * 2;
			}
		} else if (retlen == 510) {
			/* Burst that fills the whole slot.
			 * Keep the timestamp on slot boundary. */
			offset_syms = 0;
		} else {
			fprintf(stderr, "Warning: unexpected burst length %d\n", retlen);
		}
		if (retlen >= 0)
			s->tx_times[(s->tx_n++) % TIMING_TX_TIMES] = tx_time;
		*ts = tx_time + s->sym_time * offset_syms;
		s->prev_dmo = is_dmo;

		// Go to the next slot
		s->tx_time = tx_time + s->slot_time;
		s->tx_slot = (slot + 1) % TIMING_SLOTS;
	}
	return retlen;
}


int timing_resync(struct timing_state *s, struct timing_slot *slot)
{
	unsigned next_slot =
		18 * 4 * (slot->mn - 1) +
		     4 * (slot->fn - 1) +
		         (slot->tn - 1);
	uint64_t next_time = slot->time;

	/* Find the first slot that doesn't cause tx_time to jump backwards */
	const uint64_t tx_time = s->tx_time;
	unsigned i = 0;
	do {
		// Go to the next slot
		next_time += s->slot_time;
		next_slot = (next_slot + 1) % TIMING_SLOTS;
		if (++i >= 100) {
			printf("Something is wrong with timestamps and things may be in a weird state\n");
			break;
		}
	} while ((int64_t)(next_time - tx_time) < 0);
	printf("Resynchronizing, skipped %u\n", i);

	s->tx_time = next_time;
	s->tx_slot = next_slot;
	s->prev_dmo = 0;
	return 0;
}
