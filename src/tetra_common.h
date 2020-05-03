#ifndef TETRA_COMMON_H
#define TETRA_COMMON_H

#include <stdint.h>
#include "tetra_mac_pdu.h"
#include <osmocom/core/linuxlist.h>

#ifdef DEBUG
#define DEBUGP(x, args...)	printf(x, ## args)
#else
#define DEBUGP(x, args...)	do { } while(0)
#endif

#define TETRA_SYM_PER_TS	255
#define TETRA_BITS_PER_TS	(TETRA_SYM_PER_TS*2)

enum tetra_infrastructure_mode {
	TETRA_INFRA_DMO,
	TETRA_INFRA_TMO
};

/* Chapter 22.2.x */
enum tetra_log_chan {
	TETRA_LC_UNKNOWN,
	/* TMA SAP */
	TETRA_LC_SCH_F,
	TETRA_LC_SCH_HD,
	TETRA_LC_SCH_HU,
	TETRA_LC_STCH,
	TETRA_LC_SCH_P8_F,
	TETRA_LC_SCH_P8_HD,
	TETRA_LC_SCH_P8_HU,

	TETRA_LC_AACH,
	TETRA_LC_TCH,
	TETRA_LC_BSCH,
	TETRA_LC_BNCH,

	TETRA_LC_SCH_S,
	TETRA_LC_SCH_H

	/* FIXME: QAM */
};

enum dm_channel_state {
	DM_CHANNEL_S_MS_IDLE_UNKNOWN,
	DM_CHANNEL_S_MS_IDLE_FREE,
	DM_CHANNEL_S_MS_IDLE_OCCUPIED,
	DM_CHANNEL_S_MS_IDLE_RESERVED,
	DM_CHANNEL_S_MASTER_OCCUPIED,
	DM_CHANNEL_S_MASTER_RESERVED,
	DM_CHANNEL_S_SLAVE_OCCUPIED,
	DM_CHANNEL_S_SLAVE_RESERVED,
	DM_CHANNEL_S_DMREP_IDLE_UNKNOWN,
	DM_CHANNEL_S_DMREP_IDLE_FREE,
	DM_CHANNEL_S_DMREP_IDLE_OCCUPIED,
	DM_CHANNEL_S_DMREP_IDLE_RESERVED,
	DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED,
	DM_CHANNEL_S_DMREP_ACTIVE_RESERVED,
};

uint32_t bits_to_uint(const uint8_t *bits, unsigned int len);

#define FRAGSLOT_MSGB_SIZE 8192
#define FRAGSLOT_NR_SLOTS 5
struct fragslot {
	int active;
	int fragtimer;	
	struct msgb *msgb;
	int length;
	int fragments;
	int encryption;
};

struct fragslot fragslots[FRAGSLOT_NR_SLOTS]; /* slots are 1-4 but sometimes  slot==0 */

#include "tetra_tdma.h"
struct tetra_phy_state {
	struct tetra_tdma_time time;
	void (*time_adjust_cb_func)(uint8_t, uint8_t); 
};
extern struct tetra_phy_state t_phy_state;

struct tetra_mac_state {
	struct llist_head voice_channels;
	struct {
		int is_traffic;
	} cur_burst;
	struct tetra_si_decoded last_sid;

	char *dumpdir;	/* Where to save traffic channel dump */
	int ssi;	/* SSI */
	int tsn;	/* Timeslot number */
	enum tetra_infrastructure_mode infra_mode;
	enum dm_channel_state channel_state;
	uint64_t channel_state_last_chg;
};

void tetra_mac_state_init(struct tetra_mac_state *tms);

#define TETRA_CRC_OK	0x1d0f

uint32_t tetra_dl_carrier_hz(uint8_t band, uint16_t carrier, uint8_t offset);
uint32_t tetra_ul_carrier_hz(uint8_t band, uint16_t carrier, uint8_t offset,
			     uint8_t duplex, uint8_t reverse);

const char *tetra_get_lchan_name(enum tetra_log_chan lchan);
const char *tetra_get_sap_name(uint8_t sap);
#endif
