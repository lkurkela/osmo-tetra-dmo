/* TETRA lower MAC layer main routine, between TP-SAP and TMV-SAP */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/limits.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>

#include <tetra_common.h>
#include <tetra_tdma.h>
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_sync.h>
#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_scramb.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_conv_enc.h>
#include <tetra_prim.h>
#include "tetra_upper_mac.h"
#include <lower_mac/viterbi.h>

struct tetra_blk_param {
	const char *name;
	uint16_t type345_bits;
	uint16_t type2_bits;
	uint16_t type1_bits;
	uint16_t interleave_a;
	uint8_t have_crc16;
};

/* try to aggregate all of the magic numbers somewhere central */
static const struct tetra_blk_param tetra_blk_param[] = {
	[TPSAP_T_SB1] = {
		.name		= "SB1",
		.type345_bits 	= 120,
		.type2_bits	= 80,
		.type1_bits	= 60,
		.interleave_a	= 11,
		.have_crc16	= 1,
	},
	[TPSAP_T_SB2] = {
		.name		= "SB2",
		.type345_bits	= 216,
		.type2_bits	= 144,
		.type1_bits	= 124,
		.interleave_a	= 101,
		.have_crc16	= 1,
	},
	[TPSAP_T_NDB] = {
		.name		= "NDB",
		.type345_bits	= 216,
		.type2_bits	= 144,
		.type1_bits	= 124,
		.interleave_a	= 101,
		.have_crc16	= 1,
	},
	[TPSAP_T_SCH_HU] = {
		.name		= "SCH/HU",
		.type345_bits	= 168,
		.type2_bits	= 112,
		.type1_bits	= 92,
		.interleave_a	= 13,
		.have_crc16	= 1,
	},
	[TPSAP_T_SCH_F] = {
		.name		= "SCH/F",
		.type345_bits	= 432,
		.type2_bits	= 288,
		.type1_bits	= 268,
		.interleave_a	= 103,
		.have_crc16	= 1,
	},
	[TPSAP_T_BBK] = {
		.name		= "BBK",
		.type345_bits	= 30,
		.type2_bits	= 30,
		.type1_bits	= 14,
	},
	[DPSAP_T_SCH_S] = {
		.name		= "SCH/S",
		.type345_bits 	= 120,
		.type2_bits	= 80,
		.type1_bits	= 60,
		.interleave_a	= 11,
		.have_crc16	= 1,
	},
	[DPSAP_T_SCH_H] = {
		.name		= "SCH/H",
		.type345_bits	= 216,
		.type2_bits	= 144,
		.type1_bits	= 124,
		.interleave_a	= 101,
		.have_crc16	= 1,
	},
	[DPSAP_T_SCH_F] = {
		.name		= "SCH/F",
		.type345_bits	= 432,
		.type2_bits	= 288,
		.type1_bits	= 268,
		.interleave_a	= 103,
		.have_crc16	= 1,
	},
	[DPSAP_T_STCH] = {
		.name		= "STCH",
		.type345_bits	= 216,
		.type2_bits	= 144,
		.type1_bits	= 124,
		.interleave_a	= 101,
		.have_crc16	= 1,
	},

};

struct tetra_cell_data {
	uint16_t mcc;
	uint16_t mnc;
	uint32_t colour_code;
	struct tetra_tdma_time time;

	uint32_t scramb_init;
	///
	uint8_t communication_type;
	uint8_t masterslave_link_flag;
	uint8_t gateway_message_flag;
	uint8_t ab_channel_usage;
	uint8_t encryption_state;
	uint16_t repgw_address;
	uint8_t fillbit_indication;
	uint8_t fragmentation_flag;
	uint8_t number_of_SCH_F_slots;
	uint8_t frame_countdown;
	uint8_t dest_address_type;
	uint32_t dest_address;
	uint8_t src_address_type;
	uint32_t src_address;
	uint32_t mni;
};
static struct tetra_cell_data _tcd, *tcd = &_tcd;

enum DMO_SYNC_PDU_TYPE {
	DMAC_SYNC,
	DPRES_SYNC
};

enum DMO_MESSAGE_TYPE {
	DM_RESERVED,
	DM_SDS_OCCUPIED,
	DM_TIMING_REQ,
	DM_TIMING_ACK,
	RESERVED_4,
	RESERVED_5,
	RESERVED_6,
	RESERVED_7,
	DM_SETUP,
	DM_SETUP_PRES,
	DM_CONNECT,
	DM_DISCONNECT,
	DM_CONNECT_ACK,
	DM_OCCUPIED,
	DM_RELEASE,
	DM_TX_CEASED,
	DM_TX_REQ,
	DM_TX_ACCEPT,
	DM_PREEMPT,
	DM_PRE_ACCEPT,
	DM_REJECT,
	DM_INFO,
	DM_SDS_UDATA,
	DM_SDS_DATA,
	DM_SDS_ACK,
	GW_SPECIFIC_MESSAGE,
	RESERVED_26,
	RESERVED_27,
	RESERVED_28,
	RESERVED_29,
	PROPRIETARY_30,
	PROPRIETARY_31
};

int is_bsch(struct tetra_tdma_time *tm)
{
	if (tm->fn == 18 && tm->tn == 4 - ((tm->mn+1)%4))
		return 1;
	return 0;
}

int is_bnch(struct tetra_tdma_time *tm)
{
	if (tm->fn == 18 && tm->tn == 4 - ((tm->mn+3)%4))
		return 1;
	return 0;
}

struct tetra_tmvsap_prim *tmvsap_prim_alloc(uint16_t prim, uint8_t op)
{
	struct tetra_tmvsap_prim *ttp;

	ttp = talloc_zero(NULL, struct tetra_tmvsap_prim);
	ttp->oph.msg = msgb_alloc(412, "tmvsap_prim");
	ttp->oph.sap = TETRA_SAP_TMV;
	ttp->oph.primitive = prim;
	ttp->oph.operation = op;

	return ttp;
}

/* incoming DP-SAP UNITDATA.ind  from PHY into lower MAC */
void dp_sap_udata_ind(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv)
{
	/* various intermediary buffers */
	uint8_t type4[512];
	uint8_t type3dp[512*4];
	uint8_t type3[512];
	uint8_t type2[512];

	const struct tetra_blk_param *tbp = &tetra_blk_param[type];
	struct tetra_mac_state *tms = priv;
	const char *time_str;

	/* DMV-SAP.UNITDATA.ind primitive which we will send to the upper MAC */
	struct tetra_dmvsap_prim *ttp;
	struct dmv_unitdata_param *tup;

	struct msgb *msg;

	ttp = tmvsap_prim_alloc(PRIM_TMV_UNITDATA, PRIM_OP_INDICATION);
	tup = &ttp->u.unitdata;
	msg = ttp->oph.msg;

	/* update the cell time */
	memcpy(&tcd->time, &t_phy_state.time, sizeof(tcd->time));
	time_str = tetra_tdma_time_dump(&tcd->time);

	if (type == DPSAP_T_DSB2 && is_bnch(&tcd->time)) {
		tup->lchan = TETRA_LC_BNCH;
		printf("BNCH FOLLOWS\n");
	}

	DEBUGP("%s %s type5: %s\n", tbp->name, tetra_tdma_time_dump(&tcd->time),
		osmo_ubit_dump(bits, tbp->type345_bits));

	/* De-scramble, pay special attention to SB1 pre-defined scrambling */
	memcpy(type4, bits, tbp->type345_bits);
	if (type == DPSAP_T_SCH_S || type == DPSAP_T_SCH_H) {
		tetra_scramb_bits(SCRAMB_INIT, type4, tbp->type345_bits);
		tup->colour_code = SCRAMB_INIT;
	} else {
		tetra_scramb_bits(tcd->scramb_init, type4, tbp->type345_bits);
		tup->colour_code = tcd->scramb_init;
	}

	DEBUGP("%s %s type4: %s\n", tbp->name, time_str,
		osmo_ubit_dump(type4, tbp->type345_bits));


	if (tbp->interleave_a) {
		/* Run block deinterleaving: type-3 bits */
		block_deinterleave(tbp->type345_bits, tbp->interleave_a, type4, type3);
		DEBUGP("%s %s type3: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type3, tbp->type345_bits));
		/* De-puncture */
		memset(type3dp, 0xff, sizeof(type3dp));
		tetra_rcpc_depunct(TETRA_RCPC_PUNCT_2_3, type3, tbp->type345_bits, type3dp);
		DEBUGP("%s %s type3dp: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type3dp, tbp->type2_bits*4));
		viterbi_dec_sb1_wrapper(type3dp, type2, tbp->type2_bits);
		DEBUGP("%s %s type2: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type2, tbp->type2_bits));
	}

	if (tbp->have_crc16) {
		uint16_t crc = crc16_ccitt_bits(type2, tbp->type1_bits+16);
		printf("CRC COMP: 0x%04x ", crc);
		if (crc == TETRA_CRC_OK) {
			printf("OK\n");
			tup->crc_ok = 1;
			printf("%s %s type1: %s\n", tbp->name, time_str,
				osmo_ubit_dump(type2, tbp->type1_bits));
		} else
			printf("WRONG\n");
	} else if (type == TPSAP_T_BBK) {
		/* FIXME: RM3014-decode */
		tup->crc_ok = 1;
		memcpy(type2, type4, tbp->type2_bits);
		DEBUGP("%s %s type1: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type2, tbp->type1_bits));
	}

	msg->l1h = msgb_put(msg, tbp->type1_bits);
	memcpy(msg->l1h, type2, tbp->type1_bits);

	switch (type) {
	case DPSAP_T_SCH_S:
		printf("SYNC SCH/S PDU TYPE %s(0x%02x) ", osmo_ubit_dump(type2+4, 2), bits_to_uint(type2+4, 2));
		printf("System code %s(0x%02x) \n", osmo_ubit_dump(type2+0, 4), bits_to_uint(type2+0, 4));
		/* obtain information from SYNC PDU (396-3 - 9.1.1 Table 21) */
		if (tup->crc_ok) {
			uint32_t sync_pdu_type = bits_to_uint(type2+4, 2);
			switch (sync_pdu_type) {
			case DMAC_SYNC:
				tcd->communication_type = bits_to_uint(type2+6, 2);
				tcd->masterslave_link_flag = bits_to_uint(type2+8, 1);
				tcd->gateway_message_flag = bits_to_uint(type2+9, 1);
				tcd->ab_channel_usage = bits_to_uint(type2+10, 2);
				tcd->time.tn = bits_to_uint(type2+12, 2);
				tcd->time.fn = bits_to_uint(type2+14, 5);
				tcd->encryption_state = bits_to_uint(type2+19, 2);

				printf("C %d, M/S %d, GW %d, AB %d, TN %d FN %d ENC %d ", 
					tcd->communication_type, tcd->masterslave_link_flag, tcd->gateway_message_flag, tcd->ab_channel_usage, 
					tcd->time.tn, tcd->time.fn, tcd->encryption_state);
				break;

			case DPRES_SYNC:
				break;
			}

			/* compute the scrambling code for the current cell */
			// tcd->scramb_init = tetra_scramb_get_init(tcd->mcc, tcd->mnc, tcd->colour_code);

		}
		/* update the PHY layer time */
		memcpy(&t_phy_state.time, &tcd->time, sizeof(t_phy_state.time));
		tup->lchan = TETRA_LC_SCH_S;
		break;
	case DPSAP_T_SCH_H:
		printf("SYNC SCH/H TYPE %s(0x%02x) \n", osmo_ubit_dump(type2+4, 2), bits_to_uint(type2+4, 2));
		/* obtain information from SYNC PDU (396-3 - 9.1.1 Table 22) */
		if (tup->crc_ok) {
			tcd->repgw_address = bits_to_uint(type2, 10);
			tcd->fillbit_indication = bits_to_uint(type2+10, 1);
			tcd->fragmentation_flag = bits_to_uint(type2+11, 1);

			int pointer = 12;
			if (tcd->fragmentation_flag) {
				tcd->number_of_SCH_F_slots = bits_to_uint(type2+pointer, 4);
				pointer = pointer + 4;
			}
			tcd->frame_countdown = bits_to_uint(type2+pointer, 2);
			pointer = pointer + 2;
			tcd->dest_address_type = bits_to_uint(type2+pointer, 2);
			pointer = pointer + 2;
			if (tcd->dest_address_type != 2) {
				tcd->dest_address = bits_to_uint(type2+pointer, 24);
				pointer = pointer + 24;
			}

			tcd->src_address_type = bits_to_uint(type2+pointer, 2);
			pointer = pointer + 2;
			if (tcd->src_address_type != 2) {
				tcd->src_address = bits_to_uint(type2+pointer, 24);
				pointer = pointer + 24;
			}

			uint16_t mcc, mnc;

			if (tcd->communication_type < 2) {
				tcd->mni = bits_to_uint(type2+pointer, 24);
				tcd->mcc = bits_to_uint(type2+pointer, 10);
				tcd->mnc = bits_to_uint(type2+pointer+10, 14);
				pointer = pointer + 24;
			}

			uint8_t message_type = bits_to_uint(type2+pointer, 5);
			pointer = pointer + 5;

			/* calculate DM Colour Code */
			uint8_t dcc_mni = tcd->mni & 0x3f;
			uint32_t dcc_srcaddr = tcd->src_address & 0xffffff;
			tcd->colour_code = (dcc_srcaddr) | (dcc_mni << 24); 

			tcd->scramb_init = tetra_dmo_scramb_get_init(tcd->mni, tcd->src_address);

			printf("REPGW %d, Fill %d, Frag %d, num %d, FN cnt %d, dst-type %d, dst-addr %d, src-type %d, src %d, mni %d (MCC %d, MNC %d), DCC %d, msg-type %d \n",
				tcd->repgw_address, tcd->fillbit_indication, tcd->fragmentation_flag, tcd-> number_of_SCH_F_slots, tcd->frame_countdown,
				tcd->dest_address_type, tcd->dest_address, tcd->src_address_type, tcd->src_address, tcd->mni, tcd->mcc, tcd->mnc, tcd->colour_code, message_type);
	
		}
		tup->lchan = TETRA_LC_SCH_H;
		break;
	case DPSAP_T_TCH:
		printf("TCH TYPE %s(0x%02x) \n", osmo_ubit_dump(type2+4, 2), bits_to_uint(type2+4, 2));
		tup->lchan = TETRA_LC_TCH;
		break;
	case DPSAP_T_SCH_F:
		printf("SCH/F TYPE %s(0x%02x) \n", osmo_ubit_dump(type2+4, 2), bits_to_uint(type2+4, 2));
		tup->lchan = TETRA_LC_SCH_F;
		break;
	case DPSAP_T_STCH:
		printf("STCH TYPE %s(0x%02x) \n", osmo_ubit_dump(type2+4, 2), bits_to_uint(type2+4, 2));
		tup->lchan = TETRA_LC_STCH;
		break;
	default:
		printf("#### Here be dragon with SAP type %d\n", type);
		/* FIXME: do something */
		break;
	}
	/* send Rx time along with the DMV-UNITDATA.ind primitive */
	memcpy(&tup->tdma_time, &tcd->time, sizeof(tup->tdma_time));

	upper_mac_prim_recv(&ttp->oph, tms);


}

/* incoming TP-SAP UNITDATA.ind  from PHY into lower MAC */
void tp_sap_udata_ind(enum tp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv)
{
	/* various intermediary buffers */
	uint8_t type4[512];
	uint8_t type3dp[512*4];
	uint8_t type3[512];
	uint8_t type2[512];

	const struct tetra_blk_param *tbp = &tetra_blk_param[type];
	struct tetra_mac_state *tms = priv;
	const char *time_str;

	/* TMV-SAP.UNITDATA.ind primitive which we will send to the upper MAC */
	struct tetra_tmvsap_prim *ttp;
	struct tmv_unitdata_param *tup;

	struct msgb *msg;

	ttp = tmvsap_prim_alloc(PRIM_TMV_UNITDATA, PRIM_OP_INDICATION);
	tup = &ttp->u.unitdata;
	msg = ttp->oph.msg;

	/* update the cell time */
	memcpy(&tcd->time, &t_phy_state.time, sizeof(tcd->time));
	time_str = tetra_tdma_time_dump(&tcd->time);

	if (type == TPSAP_T_SB2 && is_bnch(&tcd->time)) {
		tup->lchan = TETRA_LC_BNCH;
		printf("BNCH FOLLOWS\n");
	}

	DEBUGP("%s %s type5: %s\n", tbp->name, tetra_tdma_time_dump(&tcd->time),
		osmo_ubit_dump(bits, tbp->type345_bits));

	/* De-scramble, pay special attention to SB1 pre-defined scrambling */
	memcpy(type4, bits, tbp->type345_bits);
	if (type == TPSAP_T_SB1) {
		tetra_scramb_bits(SCRAMB_INIT, type4, tbp->type345_bits);
		tup->scrambling_code = SCRAMB_INIT;
	} else {
		tetra_scramb_bits(tcd->scramb_init, type4, tbp->type345_bits);
		tup->scrambling_code = tcd->scramb_init;
	}

	DEBUGP("%s %s type4: %s\n", tbp->name, time_str,
		osmo_ubit_dump(type4, tbp->type345_bits));

	/* If this is a traffic channel, dump. */
	if ((type == TPSAP_T_SCH_F) && tms->cur_burst.is_traffic && tms->dumpdir) {
		char fname[PATH_MAX];
		int16_t block[690];
		FILE *f;
		int i;

		/* Open target file */
		snprintf(fname, sizeof(fname), "%s/traffic_%d_%d.out", tms->dumpdir,
			tms->cur_burst.is_traffic, tms->tsn);
		f = fopen(fname, "ab");

		/* Generate a block */
		memset(block, 0x00, sizeof(int16_t) * 690);
		for (i = 0; i < 6; i++)
			block[115*i] = 0x6b21 + i;

		for (i = 0; i < 114; i++)
			block[1+i] = type4[i] ? -127 : 127;

		for (i = 0; i < 114; i++)
			block[116+i] = type4[114+i] ? -127 : 127;

		for (i = 0; i < 114; i++)
			block[231+i] = type4[228+i] ? -127 : 127;

		for (i = 0; i < 90; i++)
			block[346+i] = type4[342+i] ? -127 : 127;

		/* Write it */
		fwrite(block, sizeof(int16_t), 690, f);

		fclose(f);

		/* Write used ssi */
		snprintf(fname, sizeof(fname), "%s/traffic_%d_%d.txt", tms->dumpdir,
			tms->cur_burst.is_traffic, tms->tsn);
		f = fopen(fname, "a");
		fprintf(f, "%d\n", tms->ssi);
		fclose(f);
	}

	if (tbp->interleave_a) {
		/* Run block deinterleaving: type-3 bits */
		block_deinterleave(tbp->type345_bits, tbp->interleave_a, type4, type3);
		DEBUGP("%s %s type3: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type3, tbp->type345_bits));
		/* De-puncture */
		memset(type3dp, 0xff, sizeof(type3dp));
		tetra_rcpc_depunct(TETRA_RCPC_PUNCT_2_3, type3, tbp->type345_bits, type3dp);
		DEBUGP("%s %s type3dp: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type3dp, tbp->type2_bits*4));
		viterbi_dec_sb1_wrapper(type3dp, type2, tbp->type2_bits);
		DEBUGP("%s %s type2: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type2, tbp->type2_bits));
	}

	if (tbp->have_crc16) {
		uint16_t crc = crc16_ccitt_bits(type2, tbp->type1_bits+16);
		printf("CRC COMP: 0x%04x ", crc);
		if (crc == TETRA_CRC_OK) {
			printf("OK\n");
			tup->crc_ok = 1;
			printf("%s %s type1: %s\n", tbp->name, time_str,
				osmo_ubit_dump(type2, tbp->type1_bits));
		} else
			printf("WRONG\n");
	} else if (type == TPSAP_T_BBK) {
		/* FIXME: RM3014-decode */
		tup->crc_ok = 1;
		memcpy(type2, type4, tbp->type2_bits);
		DEBUGP("%s %s type1: %s\n", tbp->name, time_str,
			osmo_ubit_dump(type2, tbp->type1_bits));
	}

	msg->l1h = msgb_put(msg, tbp->type1_bits);
	memcpy(msg->l1h, type2, tbp->type1_bits);

	switch (type) {
	case TPSAP_T_SB1:
		printf("TMB-SAP SYNC CC %s(0x%02x) ", osmo_ubit_dump(type2+4, 6), bits_to_uint(type2+4, 6));
		printf("TN %s(%u) ", osmo_ubit_dump(type2+10, 2), bits_to_uint(type2+10, 2));
		printf("FN %s(%2u) ", osmo_ubit_dump(type2+12, 5), bits_to_uint(type2+12, 5));
		printf("MN %s(%2u) ", osmo_ubit_dump(type2+17, 6), bits_to_uint(type2+17, 6));
		printf("MCC %s(%u) ", osmo_ubit_dump(type2+31, 10), bits_to_uint(type2+31, 10));
		printf("MNC %s(%u)\n", osmo_ubit_dump(type2+41, 14), bits_to_uint(type2+41, 14));
		/* obtain information from SYNC PDU */
		if (tup->crc_ok) {
			tcd->colour_code = bits_to_uint(type2+4, 6);
			tcd->time.tn = bits_to_uint(type2+10, 2);
			tcd->time.fn = bits_to_uint(type2+12, 5);
			tcd->time.mn = bits_to_uint(type2+17, 6);
			tcd->mcc = bits_to_uint(type2+31, 10);
			tcd->mnc = bits_to_uint(type2+41, 14);
			/* compute the scrambling code for the current cell */
			tcd->scramb_init = tetra_scramb_get_init(tcd->mcc, tcd->mnc, tcd->colour_code);
		}
		/* update the PHY layer time */
		memcpy(&t_phy_state.time, &tcd->time, sizeof(t_phy_state.time));
		tup->lchan = TETRA_LC_BSCH;
		break;
	case TPSAP_T_SB2:
	case TPSAP_T_NDB:
		/* FIXME: do something */
		break;
	case TPSAP_T_BBK:
		tup->lchan = TETRA_LC_AACH;
		break;
	case TPSAP_T_SCH_F:
		tup->lchan = TETRA_LC_SCH_F;
		break;
	default:
		/* FIXME: do something */
		break;
	}
	/* send Rx time along with the TMV-UNITDATA.ind primitive */
	memcpy(&tup->tdma_time, &tcd->time, sizeof(tup->tdma_time));

	upper_mac_prim_recv(&ttp->oph, tms);
}


