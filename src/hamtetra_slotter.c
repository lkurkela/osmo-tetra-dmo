/* Process and produce bursts based on their timeslot number */

#include "hamtetra_slotter.h"
#include "phy/tetra_burst.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

extern struct tetra_phy_state t_phy_state;

struct slotter_state *slotter_init()
{
	struct slotter_state *s;
	s = calloc(1, sizeof(*s));

	s->tms = calloc(1, sizeof(*s->tms));
	tetra_mac_state_init(s->tms);
	s->tms->infra_mode = TETRA_INFRA_DMO;
	init_more_tetra_dmo_rep_stuff(s->tms);

	return s;
}

int slotter_rx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot)
{
	printf("RX slot: %2u %2u %2u, diff %10ld ns\n", slot->tn, slot->fn, slot->mn, slot->diff);

	// TODO: add different operating modes here

	/* Just a test: if no bursts have been received in a while,
	 * synchronize timing to the first received burst. */
	if (slot->time - s->prev_burst_time > 1000000000UL) {
		struct timing_slot sync_slot = {
			.time = slot->time,
			.diff = 0, // not used
			.tn = 1,
			.fn = 1,
			.mn = 1
		};
		timing_resync(s->timing, &sync_slot);
	}

	/* MAC functions read timeslot number from the global variable t_phy_state.
	 * TODO: change struct timing_slot to include struct tdma_time
	 * to simplify this part. */
	t_phy_state.time.tn = slot->tn;
	t_phy_state.time.fn = slot->fn;
	t_phy_state.time.mn = slot->mn;

	enum tetra_train_seq ts = tetra_check_train(bits, len);
	tetra_burst_dmo_rx_cb2(bits, len, ts, s->tms);

	s->prev_burst_time = slot->time;
	return 0;
}


// TODO: move this to headers
int build_pdu_dpress_sync(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out);


int slotter_tx_burst(struct slotter_state *s, uint8_t *bits, int maxlen, struct timing_slot *slot)
{
	int len = -1;
	//printf("TX slot: %2u %2u %2u\n", slot->tn, slot->fn, slot->mn);

	// TODO: add different operating modes here

	// trying to glue it to tetra-dmo-rep here
	int send_count = s->send_count;
	if (slot->fn == 1 && slot->tn == 1)
		send_count = 8;
	if (--send_count >= 0) {
		uint8_t countdown = (send_count-1) / 4;
		len = build_pdu_dpress_sync(slot->fn, slot->tn, countdown, bits);
	}
	s->send_count = send_count;

	assert(len <= maxlen);
	return len;
}
