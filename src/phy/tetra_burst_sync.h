#ifndef TETRA_BURST_SYNC_H
#define TETRA_BURST_SYNC_H

#include <stdint.h>

enum rx_state {
	RX_S_UNLOCKED,		/* we're completely unlocked */
	RX_S_KNOW_FSTART,	/* we know the next frame start */
	RX_S_LOCKED,		/* fully locked */
};

enum modem_burst_sync_state {
	RX_UNSYNC,
	RX_SYNC
};

struct tetra_rx_state {
	enum rx_state state;
	unsigned int bits_in_buf;		/* how many bits are currently in bitbuf */
	uint8_t bitbuf[4096];
	unsigned int bitbuf_start_bitnum;	/* bit number at first element in bitbuf */
	unsigned int next_frame_start_bitnum;	/* frame start expected at this bitnum */

	uint64_t modem_burst_rx_timestamp; /* modem timestamp (ns) of burst end */
	uint64_t modem_prev_burst_rx_timestamp; /* previous burst end (ns) */
	enum modem_burst_sync_state modem_sync_state;
	uint64_t host_burst_rx_timestamp;

	void *burst_cb_priv;
};


/* input a raw bitstream into the tetra burst synchronizaer */
int tetra_burst_sync_in(struct tetra_rx_state *trs, uint8_t *bits, unsigned int len);

#endif /* TETRA_BURST_SYNC_H */
