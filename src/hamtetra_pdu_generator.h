#ifndef HAMTETRA_PDU_GENERATOR_H
#define HAMTETRA_PDU_GENERATOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>

#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_scramb.h>
#include <lower_mac/tetra_rm3014.h>
#include <lower_mac/viterbi.h>

#include "tetra_common.h"
#include "tetra_dll_pdu.h"
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_sync.h>

#include "hamtetra_config.h"

#define swap16(x) ((x)<<8)|((x)>>8)

int build_pdu_dpress_sync(uint8_t fn, uint8_t tn, enum tdma_master_slave_link_flag dir, uint8_t frame_countdown, uint8_t channel_state, uint8_t *out);
int build_pdu_dpress_sync_gate(uint8_t fn, uint8_t tn, enum tdma_master_slave_link_flag dir, uint8_t frame_countdown, uint8_t *out);

int build_pdu_dmac_sync_schs(struct tetra_dmo_pdu_dmac_sync *dmac_sync, uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out);
int build_pdu_dmac_sync_schh(struct tetra_dmo_pdu_dmac_sync *dmac_sync, uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out);

#endif