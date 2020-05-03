#ifndef HAMTETRA_MAC_H
#define HAMTETRA_MAC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "tetra_prim.h"
#include "tetra_common.h"
#include "hamtetra_timing.h"
#include "phy/tetra_burst.h"

static const int8_t presence_signal_multiframe_count[8] = {
	[0] = 0,
	[1] = 2,
	[2] = 5,
	[3] = 10,
    [4] = 15,
    [5] = 20,
    [6] = 30,
    [7] = 60
};


void mac_hamtetra_init();

int mac_request_tx_buffer_content(uint8_t *bits, struct timing_slot *slot);
void mac_dp_sap_udata_ind_filter(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv, struct timing_slot *slot);

void dp_sap_udata_req(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, struct tetra_tdma_time tdma_time, struct tetra_mac_state *tms_req);

#endif