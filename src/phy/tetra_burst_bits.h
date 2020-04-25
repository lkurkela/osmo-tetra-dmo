#ifndef TETRA_BURST_BITS_H
#define TETRA_BURST_BITS_H
#include <stdint.h>

/* Structs to represent bit positions in bursts coming out of the
 * synchronizer in the modem. */

struct dmo_normal_bits {
	uint8_t begin[14]; // Unused bits at beginning
	uint8_t preamble[12];
	uint8_t phase_adj[2];
	uint8_t block1[216];
	uint8_t train[22];
	uint8_t block2[216];
	uint8_t tail[2];
	uint8_t end[26]; // Unused bits at end
};

struct dmo_sync_bits {
	uint8_t begin[14]; // Unused bits at beginning
	uint8_t preamble[12];
	uint8_t phase_adj[2];
	uint8_t freq_corr[80];
	uint8_t block1[120];
	uint8_t train[38];
	uint8_t block2[216];
	uint8_t tail[2];
	uint8_t end[26]; // Unused bits at end
};

#endif
