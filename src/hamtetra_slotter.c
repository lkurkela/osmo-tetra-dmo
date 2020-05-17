/* Process and produce bursts based on their timeslot number.
 *
 * This is where the L2 protocols, i.e. lower-MAC and upper-MAC run.
 * The calls from hamtetra_timing.c to here roughly correspond
 * to the DP-SAP interface in TETRA specifications.
 */

#include "hamtetra_pdu_generator.h"
#include "hamtetra_mac.h"
#include "hamtetra_slotter.h"
#include "hamtetra_config.h"
#include "phy/tetra_burst.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

extern struct tetra_phy_state t_phy_state;
FILE *debuglog;

struct slotter_state *slotter_init()
{
	struct slotter_state *s;
	s = calloc(1, sizeof(*s));

	s->tms = calloc(1, sizeof(*s->tms));
	tetra_mac_state_init(s->tms);
	s->tms->infra_mode = TETRA_INFRA_DMO;

	struct tetra_tdma_time init_time = {
			.hn = 1,
			.sn = 1,
			.tn = 1,
			.fn = 1,
			.mn = 1,
			.link = DM_LINK_MASTER
		};
	t_phy_state.time = init_time;

	// initialize fragmentation slots
	memset((void *)&fragslots,0,sizeof(struct fragslot)*FRAGSLOT_NR_SLOTS);
	char desc[]="slot \0";
	for (int k=0;k<FRAGSLOT_NR_SLOTS;k++) {
		desc[4]='0'+k;
		fragslots[k].msgb=msgb_alloc(8192, desc);
		msgb_reset(fragslots[k].msgb);
	}

	s->tms->channel_state=DM_CHANNEL_S_DMREP_IDLE_UNKNOWN;

	#ifdef DEBUG_BURSTLOG
	debuglog = fopen(DEBUG_BURSTLOG, "ab");
	#endif

	return s;
}

int slotter_rx_burst(struct slotter_state *s, const uint8_t *bits, int len, struct timing_slot *slot)
{
	printf("RX slot: %2u %2u %2u, diff %10ld ns\n", slot->tn, slot->fn, slot->mn, slot->diff);

	// TODO: add different operating modes here

	/* Just a test: if no bursts have been received in a while,
	 * synchronize timing to the first received burst. 
	if (slot->time - s->prev_burst_time > 1000000000UL) {
		struct timing_slot sync_slot = {
			.time = slot->time,
			.diff = 0, // not used
			.tn = 1,
			.fn = 1,
			.mn = 1
		};
		timing_resync(s->timing, &sync_slot);
	} */

	/* MAC functions read timeslot number from the global variable t_phy_state.
	 * TODO: change struct timing_slot to include struct tdma_time
	 * to simplify this part. */
	t_phy_state.time.tn = slot->tn;
	t_phy_state.time.fn = slot->fn;
	t_phy_state.time.mn = slot->mn;

	slot->changed = 0;
	s->tms->slot = slot;
	enum tetra_train_seq ts = tetra_check_train(bits, len);
	tetra_burst_dmo_rx_cb2(bits, len, ts, s->tms);

	// if lower mac resyncronizes the frame timings based on SCH/S burst
	if (slot->changed>0) {
		struct timing_slot sync_slot = {
			.time = slot->time,
			.diff = 0, // not used
			.tn = slot->tn,
			.fn = slot->fn,
			.mn = slot->mn
		};
		timing_resync(s->timing, &sync_slot);

	}

	#ifdef DEBUG_BURSTLOG
	fprintf(debuglog,"RX;%u;%02u;%u;%ld;%s\n", slot->mn, slot->fn, slot->tn, slot->diff, osmo_ubit_dump(bits,len));
	#endif

	s->prev_burst_time = slot->time;
	return 0;
}


int slotter_tx_burst(struct slotter_state *s, uint8_t *bits, int maxlen, struct timing_slot *slot)
{
	int len = -1;
	//printf("TX slot: %2u %2u %2u\n", slot->tn, slot->fn, slot->mn);

	// TODO: add different operating modes here

	// trying to glue it to tetra-dmo-rep here
	len = mac_request_tx_buffer_content(bits, slot);

	#ifdef DEBUG_BURSTLOG
	if (len>-1) {
		fprintf(debuglog,"TX;%u;%02u;%u;%ld;%s\n", slot->mn, slot->fn, slot->tn, slot->diff, osmo_ubit_dump(bits,len));
	}
	#endif

	assert(len <= maxlen);
	return len;
}
