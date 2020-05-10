/* TETRA upper MAC layer main routine, above TMV-SAP */

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

#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>

#include "tetra_common.h"
#include "tetra_prim.h"
#include "tetra_upper_mac.h"
#include "tetra_mac_pdu.h"
#include "tetra_dll_pdu.h"
#include "tetra_llc_pdu.h"
#include "tetra_mm_pdu.h"
#include "tetra_cmce_pdu.h"
#include "tetra_sndcp_pdu.h"
#include "tetra_mle_pdu.h"
#include "tetra_gsmtap.h"
#include "lower_mac/tetra_lower_mac.h"
#include "hamtetra_config.h"
#include "hamtetra_pdu_generator.h"

static int rx_tm_sdu(struct tetra_mac_state *tms, struct msgb *msg, unsigned int len);

const char *tetra_get_dmo_message_type_name(uint8_t pdut)
{
	return get_value_string(tetra_dmo_message_type_names, pdut);
}


static void rx_bcast(struct tetra_tmvsap_prim *tmvp, struct tetra_mac_state *tms)
{
	struct msgb *msg = tmvp->oph.msg;
	struct tetra_si_decoded sid;
	uint32_t dl_freq, ul_freq;
	int i;

	memset(&sid, 0, sizeof(sid));
	macpdu_decode_sysinfo(&sid, msg->l1h);
	tmvp->u.unitdata.tdma_time.hn = sid.hyperframe_number;

	dl_freq = tetra_dl_carrier_hz(sid.freq_band,
				      sid.main_carrier,
				      sid.freq_offset);

	ul_freq = tetra_ul_carrier_hz(sid.freq_band,
				      sid.main_carrier,
				      sid.freq_offset,
				      sid.duplex_spacing,
				      sid.reverse_operation);

	printf("BNCH SYSINFO (DL %u Hz, UL %u Hz), service_details 0x%04x ",
		dl_freq, ul_freq, sid.mle_si.bs_service_details);
	if (sid.cck_valid_no_hf)
		printf("CCK ID %u", sid.cck_id);
	else
		printf("Hyperframe %u", sid.hyperframe_number);
	printf("\n");
	for (i = 0; i < 12; i++)
		printf("\t%s: %u\n", tetra_get_bs_serv_det_name(1 << i),
			sid.mle_si.bs_service_details & (1 << i) ? 1 : 0);

	memcpy(&tms->last_sid, &sid, sizeof(sid));
}

const char *tetra_alloc_dump(const struct tetra_chan_alloc_decoded *cad, struct tetra_mac_state *tms)
{
	static char buf[64];
	char *cur = buf;
	unsigned int freq_band, freq_offset;

	if (cad->ext_carr_pres) {
		freq_band = cad->ext_carr.freq_band;
		freq_offset = cad->ext_carr.freq_offset;
	} else {
		freq_band = tms->last_sid.freq_band;
		freq_offset = tms->last_sid.freq_offset;
	}

	cur += sprintf(cur, "%s (TN%u/%s/%uHz)",
		tetra_get_alloc_t_name(cad->type), cad->timeslot,
		tetra_get_ul_dl_name(cad->ul_dl),
		tetra_dl_carrier_hz(freq_band, cad->carrier_nr, freq_offset));

	return buf;
}

/* Receive TL-SDU (LLC SDU == MLE PDU) */
static int rx_tl_sdu(struct tetra_mac_state *tms, struct msgb *msg, unsigned int len)
{
	uint8_t *bits = msg->l3h;
	uint8_t mle_pdisc = bits_to_uint(bits, 3);

	printf("TL-SDU(%s): %s", tetra_get_mle_pdisc_name(mle_pdisc),
		osmo_ubit_dump(bits, len));
	switch (mle_pdisc) {
	case TMLE_PDISC_MM:
		printf(" %s", tetra_get_mm_pdut_name(bits_to_uint(bits+3, 4), 0));
		break;
	case TMLE_PDISC_CMCE:
		printf(" %s", tetra_get_cmce_pdut_name(bits_to_uint(bits+3, 5), 0));
		break;
	case TMLE_PDISC_SNDCP:
		printf(" %s", tetra_get_sndcp_pdut_name(bits_to_uint(bits+3, 4), 0));
		printf(" NSAPI=%u PCOMP=%u, DCOMP=%u",
			bits_to_uint(bits+3+4, 4),
			bits_to_uint(bits+3+4+4, 4),
			bits_to_uint(bits+3+4+4+4, 4));
		printf(" V%u, IHL=%u",
			bits_to_uint(bits+3+4+4+4+4, 4),
			4*bits_to_uint(bits+3+4+4+4+4+4, 4));
		printf(" Proto=%u",
			bits_to_uint(bits+3+4+4+4+4+4+4+64, 8));
		break;
	case TMLE_PDISC_MLE:
		printf(" %s", tetra_get_mle_pdut_name(bits_to_uint(bits+3, 3), 0));
		break;
	default:
		break;
	}
	return len;
}

static int rx_tm_sdu(struct tetra_mac_state *tms, struct msgb *msg, unsigned int len)
{
	struct tetra_llc_pdu lpp;
	uint8_t *bits = msg->l2h;

	memset(&lpp, 0, sizeof(lpp));
	tetra_llc_pdu_parse(&lpp, bits, len);

	printf("TM-SDU(%s,%u,%u): ",
		tetra_get_llc_pdut_dec_name(lpp.pdu_type), lpp.ns, lpp.ss);
	if (lpp.tl_sdu && lpp.ss == 0) {
		msg->l3h = lpp.tl_sdu;
		rx_tl_sdu(tms, msg, lpp.tl_sdu_len);
	}
	return len;
}

static void rx_resrc(struct tetra_tmvsap_prim *tmvp, struct tetra_mac_state *tms)
{
	struct msgb *msg = tmvp->oph.msg;
	struct tetra_resrc_decoded rsd;
	int tmpdu_offset;

	memset(&rsd, 0, sizeof(rsd));
	tmpdu_offset = macpdu_decode_resource(&rsd, msg->l1h);
	msg->l2h = msg->l1h + tmpdu_offset;

	printf("RESOURCE Encr=%u, Length=%d Addr=%s ",
		rsd.encryption_mode, rsd.macpdu_length,
		tetra_addr_dump(&rsd.addr));

	if (rsd.addr.type == ADDR_TYPE_NULL)
		goto out;

	if (rsd.chan_alloc_pres)
		printf("ChanAlloc=%s ", tetra_alloc_dump(&rsd.cad, tms));

	if (rsd.slot_granting.pres)
		printf("SlotGrant=%u/%u ", rsd.slot_granting.nr_slots,
			rsd.slot_granting.delay);

	if (rsd.macpdu_length > 0 && rsd.encryption_mode == 0) {
		int len_bits = rsd.macpdu_length*8;
		if (msg->l2h + len_bits > msg->l1h + msgb_l1len(msg))
			len_bits = msgb_l1len(msg) - tmpdu_offset;
		rx_tm_sdu(tms, msg, len_bits);
	}

	tms->ssi = rsd.addr.ssi;

out:
	printf("\n");
}

static void rx_suppl(struct tetra_tmvsap_prim *tmvp, struct tetra_mac_state *tms)
{
	//struct tmv_unitdata_param *tup = &tmvp->u.unitdata;
	struct msgb *msg = tmvp->oph.msg;
	//struct tetra_suppl_decoded sud;
	int tmpdu_offset;

#if 0
	memset(&sud, 0, sizeof(sud));
	tmpdu_offset = macpdu_decode_suppl(&sud, msg->l1h, tup->lchan);
#else
	{
		uint8_t slot_granting = *(msg->l1h + 17);
		if (slot_granting)
			tmpdu_offset = 17+1+8;
		else
			tmpdu_offset = 17+1;
	}
#endif

	printf("SUPPLEMENTARY MAC-D-BLOCK ");

	//if (sud.encryption_mode == 0)
		msg->l2h = msg->l1h + tmpdu_offset;
		rx_tm_sdu(tms, msg, 100);

	printf("\n");
}

static void dump_access(struct tetra_access_field *acc, unsigned int num)
{
	printf("ACCESS%u: %c/%u ", num, 'A'+acc->access_code, acc->base_frame_len);
}

static void rx_aach(struct tetra_tmvsap_prim *tmvp, struct tetra_mac_state *tms)
{
	struct tmv_unitdata_param *tup = &tmvp->u.unitdata;
	struct tetra_acc_ass_decoded aad;

	printf("ACCESS-ASSIGN PDU: ");

	memset(&aad, 0, sizeof(aad));
	macpdu_decode_access_assign(&aad, tmvp->oph.msg->l1h,
				    tup->tdma_time.fn == 18 ? 1 : 0);

	if (aad.pres & TETRA_ACC_ASS_PRES_ACCESS1)
		dump_access(&aad.access[0], 1);
	if (aad.pres & TETRA_ACC_ASS_PRES_ACCESS2)
		dump_access(&aad.access[1], 2);
	if (aad.pres & TETRA_ACC_ASS_PRES_DL_USAGE)
		printf("DL_USAGE: %s ", tetra_get_dl_usage_name(aad.dl_usage));
	if (aad.pres & TETRA_ACC_ASS_PRES_UL_USAGE)
		printf("UL_USAGE: %s ", tetra_get_ul_usage_name(aad.ul_usage));

	/* save the state whether the current burst is traffic or not */
	if (aad.dl_usage > 3)
		tms->cur_burst.is_traffic = aad.dl_usage;
	else
		tms->cur_burst.is_traffic = 0;

	printf("\n");
}

static int rx_tmv_unitdata_ind(struct tetra_tmvsap_prim *tmvp, struct tetra_mac_state *tms)
{
	struct tmv_unitdata_param *tup = &tmvp->u.unitdata;
	struct msgb *msg = tmvp->oph.msg;
	uint8_t pdu_type = bits_to_uint(msg->l1h, 2);
	const char *pdu_name;
	struct msgb *gsmtap_msg;

	if (tup->lchan == TETRA_LC_BSCH)
		pdu_name = "SYNC";
	else if (tup->lchan == TETRA_LC_AACH)
		pdu_name = "ACCESS-ASSIGN";
	else {
		pdu_type = bits_to_uint(msg->l1h, 2);
		pdu_name = tetra_get_macpdu_name(pdu_type);
	}

	printf("TMV-UNITDATA.ind %s %s CRC=%u %s\n",
		tetra_tdma_time_dump(&tup->tdma_time),
		tetra_get_lchan_name(tup->lchan),
		tup->crc_ok, pdu_name);

	if (!tup->crc_ok)
		return 0;

	gsmtap_msg = tetra_gsmtap_makemsg(&tup->tdma_time, tup->lchan,
					  tup->tdma_time.tn,
					  /* FIXME: */ 0, 0, 0,
					msg->l1h, msgb_l1len(msg), tms);
	if (gsmtap_msg)
		tetra_gsmtap_sendmsg(gsmtap_msg);

	switch (tup->lchan) {
	case TETRA_LC_AACH:
		rx_aach(tmvp, tms);
		break;
	case TETRA_LC_BNCH:
	case TETRA_LC_UNKNOWN:
	case TETRA_LC_SCH_F:
		switch (pdu_type) {
		case TETRA_PDU_T_BROADCAST:
			rx_bcast(tmvp, tms);
			break;
		case TETRA_PDU_T_MAC_RESOURCE:
			rx_resrc(tmvp, tms);
			break;
		case TETRA_PDU_T_MAC_SUPPL:
			rx_suppl(tmvp, tms);
			break;
		case TETRA_PDU_T_MAC_FRAG_END:
			if (msg->l1h[3] == TETRA_MAC_FRAGE_FRAG) {
				printf("FRAG/END FRAG: ");
				msg->l2h = msg->l1h+4;
				rx_tm_sdu(tms, msg, 100 /*FIXME*/);
				printf("\n");
			} else
				printf("FRAG/END END\n");
			break;
		default:
			printf("STRANGE pdu=%u\n", pdu_type);
			break;
		}
		break;
	case TETRA_LC_BSCH:
		break;
	default:
		printf("STRANGE lchan=%u\n", tup->lchan);
		break;
	}

	return 0;
}

struct tetra_dmasap_prim *dmasap_prim_alloc(uint16_t prim, uint8_t op)
{
	struct tetra_dmasap_prim *dmasap;

	dmasap = talloc_zero(NULL, struct tetra_dmasap_prim);
	dmasap->oph.msg = msgb_alloc(412, "dmasap_prim");
	dmasap->oph.sap = TETRA_SAP_DMV;
	dmasap->oph.primitive = prim;
	dmasap->oph.operation = op;

	return dmasap;
}

static void parse_dm_message(uint8_t message_type, struct tetra_dmvsap_prim *dmvp){
	struct msgb *msg = dmvp->oph.msg;
	uint8_t *bits = msg->l2h;
	const char *message_name = tetra_get_dmo_message_type_name(message_type);

	if (message_type == DM_SETUP || message_type == DM_CONNECT_ACK || message_type == DM_OCCUPIED) {
		// message dependent elements
		uint8_t timing_flag = bits_to_uint(bits, 1);
		uint8_t lch_in_fn3_flag = bits_to_uint(bits+1, 1);
		uint8_t pre_emption_flag = bits_to_uint(bits+2, 1);
		uint8_t powerclass = bits_to_uint(bits+3, 3);
		uint8_t powercontrol = bits_to_uint(bits+6, 1);
		uint8_t dualwatch_sync = bits_to_uint(bits+9, 1);
		uint8_t twofreq_call = bits_to_uint(bits+10, 1);
		uint8_t circuit_mode_type = bits_to_uint(bits+11, 4);
		uint8_t priority = bits_to_uint(bits+19, 2);
		// DM-SDU elements
		uint8_t end_to_end_encryption = bits_to_uint(bits+21, 1);
		uint8_t call_type = bits_to_uint(bits+22, 1);
		uint8_t external_source = bits_to_uint(bits+23, 1);

		printf("%s PDU: timing: %d, LCH in FN3: %d, pre-emption: %d, power class: %d, " \
			"power control: %d, dual-watch sync: %d, two-freq call: %d, circuit mode: %d, priority level: %d \n" \
			"end-to-end encryption: %d, call type: %d, external source: %d \n", message_name, timing_flag, lch_in_fn3_flag, pre_emption_flag, 
			powerclass, powercontrol, dualwatch_sync, twofreq_call, circuit_mode_type, priority, end_to_end_encryption, call_type, external_source);

		msg->l3h = msg->l2h+21;

	} else if (message_type == DM_RESERVED) {
		uint8_t channel_reservation_type = bits_to_uint(bits, 1);
		uint8_t reservation_time_remaining = bits_to_uint(bits+1, 6);
		uint8_t timing_flag = bits_to_uint(bits+7, 1);
		uint8_t requests_flag = bits_to_uint(bits+8, 1);
		uint8_t changeover_requests_flag = bits_to_uint(bits+9, 1);
		uint8_t requests_bitmap = 0;
		int pointer = 10;
		if (requests_flag == 1) {
			requests_bitmap = bits_to_uint(bits+pointer, 8); pointer += 8;
		} 
		uint8_t powerclass = bits_to_uint(bits+pointer, 3); pointer += 3;
		uint8_t powercontrol = bits_to_uint(bits+pointer, 1); pointer += 1;
		uint8_t priority = bits_to_uint(bits+pointer, 2); pointer += 2;
		uint8_t dualwatch_sync = bits_to_uint(bits+pointer, 1); pointer += 1;
		uint8_t twofreq_call = bits_to_uint(bits+pointer, 1); pointer += 1;

		printf("%s PDU: channel reservation type: %d, reservation time remaining: %d, timing: %d, requests: %d, changeover requests: %d, requests bitmap: %d, " \
			"recent user priority flag: %d, timing change announced: %d timing adjustment: %d, priority level: %d \n" \
			"cease cause: %d \n", message_name, channel_reservation_type, reservation_time_remaining, timing_flag, requests_flag, changeover_requests_flag, 
			requests_bitmap, powerclass, powercontrol, priority, dualwatch_sync, twofreq_call);

		msg->l3h = msg->l2h;

	} else if (message_type == DM_TX_CEASED) {
		uint8_t reservation_time_remaining = bits_to_uint(bits, 6);
		uint8_t timing_flag = bits_to_uint(bits+6, 1);
		uint8_t requests_flag = bits_to_uint(bits+7, 1);
		uint8_t changeover_requests_flag = bits_to_uint(bits+8, 1);
		uint8_t requests_bitmap = 0;
		int pointer = 9;
		if (requests_flag == 1) {
			requests_bitmap = bits_to_uint(bits+pointer, 8); pointer += 8;
		} 
		uint8_t recent_user_priority_flag = bits_to_uint(bits+pointer, 1); pointer += 1;
		uint8_t timing_change_announced = bits_to_uint(bits+pointer, 1); pointer += 1;
		uint8_t timing_adjustment = 0;
		if (timing_change_announced == 1) {
			timing_adjustment = bits_to_uint(bits+pointer, 12); pointer += 12;
		}
		uint8_t priority = bits_to_uint(bits+pointer, 2); pointer += 2;
		// DM-SDU elements
		uint8_t cease_cause = bits_to_uint(bits+pointer, 4); 

		printf("%s PDU: reservation time remaining: %d, timing: %d, requests: %d, changeover requests: %d, requests bitmap: %d, " \
			"recent user priority flag: %d, timing change announced: %d timing adjustment: %d, priority level: %d \n" \
			"cease cause: %d \n", message_name, reservation_time_remaining, timing_flag, requests_flag, changeover_requests_flag, 
			requests_bitmap, recent_user_priority_flag, timing_change_announced, timing_adjustment, priority, cease_cause);

		msg->l3h = msg->l2h+pointer;


	} else if (message_type == DM_SDS_UDATA || message_type == DM_SDS_DATA) {
		// message dependent elements
		uint8_t sds_time_remaining = bits_to_uint(bits, 4);
		uint8_t sds_transaction_type = bits_to_uint(bits+4, 1);
		uint8_t priority = bits_to_uint(bits+5, 2);
		uint8_t fcs_flag = bits_to_uint(bits+7, 1);
		// DM-SDU elements
		int pointer = 8;
		uint8_t additional_addressing_flag = bits_to_uint(bits+pointer, 1);
		uint8_t additional_address_type = 0;
		uint64_t calling_party_tsi = 0;
		if (additional_addressing_flag == 1) {
			additional_address_type = bits_to_uint(bits+pointer, 4); pointer += 4;
			if (additional_address_type == 1) {
				calling_party_tsi = bits_to_uint(bits+pointer, 48); pointer += 48;
			}
		}
		uint8_t short_data_type_identifier = bits_to_uint(bits+pointer, 4); pointer += 4;
		uint64_t user_defined_data = 0;
		uint16_t predefined_status = 0;
		uint16_t length_indicator = 0;
		const char *user_data;
		switch(short_data_type_identifier) {
			case 0: // data1
				user_defined_data = bits_to_uint(bits+pointer, 16); 
				user_data = osmo_ubit_dump(bits+pointer, 16);
				pointer += 16;
				break;
			case 1:	// data2
				user_defined_data = bits_to_uint(bits+pointer, 32);
				user_data = osmo_ubit_dump(bits+pointer, 32);
				pointer += 32;
				break;
			case 2: // data3
				user_defined_data = bits_to_uint(bits+pointer, 64);
				user_data = osmo_ubit_dump(bits+pointer, 64);
				pointer += 64;
				break;
			case 3: // length indicator + data4
				length_indicator = bits_to_uint(bits+pointer, 11);
				pointer += 11;
				if (msg->len > pointer+length_indicator) {
					user_defined_data = bits_to_uint(bits+pointer, length_indicator);
					pointer += length_indicator;
				} else {
					printf("#### not enough bits, man! consider fragmenting...\n");
				}
				break;
			case 4:	// precoded status
				predefined_status = bits_to_uint(bits+pointer, 16);
				pointer += 16;
				break;
		};
		uint32_t fcs_data = 0;
		if (fcs_flag) {
			fcs_data = bits_to_uint(bits+pointer, 32);
			pointer += 4;
		}

		printf("%s PDU: SDS time remaining: %d, SDS transaction type %d, priority level: %d, FCS flag %d - " \
			"additional addressing: %d, add. address types: %d, calling party TSI: %ld, short data type identifier: %d " \
			"user defined data %ld, length indicator %d, precoded status: %d FCS %d \n", message_name, sds_time_remaining, sds_transaction_type, priority, fcs_flag,
			additional_addressing_flag, additional_address_type, calling_party_tsi, short_data_type_identifier, user_defined_data, length_indicator, predefined_status, fcs_data);

		msg->l3h = msg->l2h+8;

	} else
	{
		printf("%s PDU: yet to be defined \n", message_name);
	}
	
}



/* add bits to a fragment. these should really be bit operations and not stuffing one bit per byte */
void append_frag_bits(int slot, uint8_t *bits, int bitlen, int fillbits)
{
	int i=bitlen;
	int l=fragslots[slot].length;
	struct msgb *fragmsgb;
	uint8_t bit;
	int zeroes=0;

	fragmsgb= fragslots[slot].msgb;

	while(i) {
		bit=bits_to_uint(bits, 1);
		msgb_put_u8(fragmsgb,bit);
		if (bit) { zeroes=0; } else { zeroes++; }
		bits++;
		i--;
		l++;
		if (l>4095) { printf("\nFRAG LENGTH ERROR!\n"); return; } /* limit hardcoded for now, the buffer allocated is twice the size just in case */
	}

	fragslots[slot].length=fragslots[slot].length+bitlen;

	if (fillbits) {
		fragslots[slot].length=fragslots[slot].length-zeroes;
		msgb_get(fragmsgb,zeroes);
	}

	fragslots[slot].fragments++;
	fragslots[slot].fragtimer=0;
	printf("\nappend_frag slot=%i len=%i totallen=%i fillbits=%i\n",slot,bitlen,fragslots[slot].length,fillbits);
	// printf("\nFRAGDUMP: %s\n",osmo_ubit_dump((unsigned char *)fragmsgb->l3h,msgb_l3len(fragmsgb)));

}

/* MAC-FRAG PDU */
static void rx_macfrag(struct tetra_dmvsap_prim *tmvp, struct tetra_mac_state *tms,int slot)
{
	struct msgb *msg = tmvp->oph.msg;
	struct tetra_resrc_decoded rsd;
	uint8_t *bits = msg->l1h;
	int n=0;
	int m=0;

	memset(&rsd, 0, sizeof(rsd));
	m=2; uint8_t macpdu_type=bits_to_uint(bits+n, m); n=n+m; /*  MAC-FRAG/END */
	m=1; uint8_t macpdu_subtype=bits_to_uint(bits+n, m); n=n+m; /* 0 - MAC-FRAG */
	m=1; uint8_t fillbits_present=bits_to_uint(bits+n, m); n=n+m;
	int len=msgb_l1len(msg) - n;

	if (fragslots[slot].active) {
		append_frag_bits(slot,bits+n,len,fillbits_present);
	} else {
		printf("\nFRAG: got fragment without start packet for slot=%i\n",slot);
	}
}

/* 9.2.3 DMAC-END PDU */
static void rx_macend(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms, int slot)
{
	struct msgb *msg = dmvp->oph.msg;
	int tmpdu_offset;
	uint8_t *bits = msg->l1h;
	struct msgb *fragmsgb = fragslots[slot].msgb;
	int n=0;
	int m=0;

	m=2; uint8_t macpdu_type=bits_to_uint(bits+n, m); n=n+m;
	m=1; uint8_t macpdu_subtype=bits_to_uint(bits+n, m); n=n+m;
	m=1; uint8_t fillbits_present=bits_to_uint(bits+n, m); n=n+m;
	int len=msgb_l1len(msg)-4;

	fragslots[slot].fragments++;
	if (fragslots[slot].active) {
		append_frag_bits(slot,bits+n,len,fillbits_present);

		/* for now filter out just SDS messages to hide the fact that the fragment stuff doesn't work 100% correctly :) */
		uint8_t *b = fragmsgb->l2h;
		printf("\nFRAGMENT DECODE fragments=%i len=%i slot=%i Encr=%i ",fragslots[slot].fragments,fragslots[slot].length,slot,fragslots[slot].encryption);
		fflush(stdout); /* TODO: remove this in the future, for now leave it so that the printf() is shown if rx_tl_sdu segfaults for somee reason */
		printf("\nMessage: %s\n", osmo_ubit_dump(b, msgb_l2len));

		if (b) {
			uint8_t mle_pdisc = bits_to_uint(b, 3);
			uint8_t proto=bits_to_uint(b+3, 5);
			if ((mle_pdisc==TMLE_PDISC_CMCE)&&(proto==TCMCE_PDU_T_D_SDS_DATA)) {
				printf("\nFRAGMENT DECODE fragments=%i len=%i slot=%i Encr=%i ",fragslots[slot].fragments,fragslots[slot].length,slot,fragslots[slot].encryption);
				fflush(stdout); /* TODO: remove this in the future, for now leave it so that the printf() is shown if rx_tl_sdu segfaults for somee reason */
				rx_tl_sdu(tms, fragmsgb, fragslots[slot].length);
			}
		}
		else 
		{
			printf("\nFRAG: got end frag without start packet for slot=%i\n",slot);
		}
	} else {
		printf("\nFRAGMENT without l3 header dropped slot=%i\n",slot);

	}

	msgb_reset(fragmsgb);
	fragslots[slot].fragments=0;
	fragslots[slot].active=0;
	fragslots[slot].length=0;
	fragslots[slot].fragtimer=0;
}

static void rx_dmo_dmac_data(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms)
{
	struct dmv_unitdata_param *tup = &dmvp->u.unitdata;
	struct msgb *msg = dmvp->oph.msg;
	uint8_t *bits = msg->l1h;
	int pointer = 12;

	uint8_t pdu_type = bits_to_uint(bits, 2);
	uint8_t fill_bit = bits_to_uint(bits+2, 1);
	uint8_t second_half_slot_stolen = bits_to_uint(bits+3, 1);
	uint8_t fragmentation_flag = bits_to_uint(bits+4, 1);
	uint8_t null_pdu = bits_to_uint(bits+5, 1);
	if (null_pdu == 0) {
		uint8_t frame_countdown = bits_to_uint(bits+6, 2);
		uint8_t encryption_status = bits_to_uint(bits+8, 2);
		uint8_t destaddr_type = bits_to_uint(bits+10, 2);
		uint32_t destaddr = 0;
		if (destaddr_type != 2) {
			destaddr =  bits_to_uint(bits+pointer, 24); pointer += 24;
		}
		uint8_t srcaddr_type = bits_to_uint(bits+pointer, 2); pointer += 2;
		uint32_t srcaddr = 0;
		if (srcaddr_type != 2) {
			srcaddr = bits_to_uint(bits+pointer, 24); pointer += 24;
		}
		uint32_t mni = bits_to_uint(bits+pointer, 24); pointer += 24;
		uint8_t message_type = bits_to_uint(bits+pointer, 5); 

		msg->l2h = msg->l1h+pointer;
		const char *message_name = tetra_get_dmo_message_type_name(message_type);

		printf("DMAC-DATA PDU - fill bit: %d, 2nd half stolen: %d, frag.flag: %d, nullPDU: %d, frame countdown: %d, enc.status: %d, " \
			"dst address type: %d, dst address: %d, src address type: %d, src address: %d, MNI: %d, message type: %s (%d) \n", fill_bit, second_half_slot_stolen, fragmentation_flag,
			null_pdu, frame_countdown, encryption_status, destaddr_type, destaddr, srcaddr_type, srcaddr, mni, message_name, message_type);

	} else {
		printf("DMAC-DATA PDU - fill bit: %d, 2nd half stolen: %d, frag.flag: %d, nullPDU: %d \n", fill_bit, second_half_slot_stolen, fragmentation_flag, null_pdu);

	}
}

static void rx_dm_dmacsync_sch_s(uint8_t *bits, struct tetra_dmo_pdu_dmac_sync *sync) {
	sync->system_code = bits_to_uint(bits, 4);
	sync->sync_pdu_type = bits_to_uint(bits+4, 2);
	sync->communication_type = bits_to_uint(bits+6, 2);
	if (sync->communication_type == 1 || sync->communication_type == 3) {
		sync->masterslave_link_flag = bits_to_uint(bits+8, 1);
	}
	if (sync->communication_type == 2 || sync->communication_type == 3) {
		sync->gateway_message_flag = bits_to_uint(bits+9, 1);

	}
	sync->ab_channel_usage = bits_to_uint(bits+10, 2);
	sync->slot_number = bits_to_uint(bits+12, 2) + 1;
	sync->frame_number = bits_to_uint(bits+14, 5);
	sync->airint_encryption_state = bits_to_uint(bits+19, 2);
	if (sync->airint_encryption_state>0) {
		sync->time_variant_parameter = bits_to_uint(bits+21, 29);
		sync->ksg_number = bits_to_uint(bits+51, 4);
		sync->encryption_key_number = bits_to_uint(bits+55, 5);
	}
}

int rx_dm_dmacsync_sch_h(uint8_t *bits, struct tetra_dmo_pdu_dmac_sync *sync) {

	int pointer = 0;
	if (pdu_dmac_sync->communication_type >0 ){
		pdu_dmac_sync->repgw_address = bits_to_uint(bits, 10);
	} else {
		pdu_dmac_sync->repgw_address = 0;
	}
	pdu_dmac_sync->fillbit_indication = bits_to_uint(bits+10, 1);
	pdu_dmac_sync->fragmentation_flag = bits_to_uint(bits+11, 1);
	pointer = 12;
	if (pdu_dmac_sync->fragmentation_flag) {
		pdu_dmac_sync->number_of_sch_f_slots = bits_to_uint(bits+pointer, 4);
		pointer = pointer + 4;
	}
	pdu_dmac_sync->frame_countdown = bits_to_uint(bits+pointer, 2);
	pointer = pointer + 2;
	pdu_dmac_sync->dest_address_type = bits_to_uint(bits+pointer, 2);
	pointer = pointer + 2;
	if (pdu_dmac_sync->dest_address_type != 2) {
		pdu_dmac_sync->dest_address = bits_to_uint(bits+pointer, 24);
		pointer = pointer + 24;
	}

	pdu_dmac_sync->src_address_type = bits_to_uint(bits+pointer, 2);
	pointer = pointer + 2;
	if (pdu_dmac_sync->src_address_type != 2) {
		pdu_dmac_sync->src_address = bits_to_uint(bits+pointer, 24);
		pointer = pointer + 24;
	}

	if (pdu_dmac_sync->communication_type < 2) {
		pdu_dmac_sync->mni = bits_to_uint(bits+pointer, 24);
		pointer = pointer + 24;
	}

	pdu_dmac_sync->message_type = bits_to_uint(bits+pointer, 5);
	pointer = pointer + 5;

	switch(pdu_dmac_sync->message_type) {
		case DM_SETUP: 
		case DM_SETUP_PRES:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 21);
			pdu_dmac_sync->message_fields_len = 21;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+21, 5);
			pdu_dmac_sync->dm_sdu_len = 5;
			break;
		case DM_CONNECT:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 8);
			pdu_dmac_sync->message_fields_len = 8;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+8, 4);
			pdu_dmac_sync->dm_sdu_len = 4;
			break;
		case DM_DISCONNECT:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 0);
			pdu_dmac_sync->message_fields_len = 0;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+0, 4);
			pdu_dmac_sync->dm_sdu_len = 4;
			break;
		case DM_CONNECT_ACK:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 21);
			pdu_dmac_sync->message_fields_len = 21;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+21, 5);
			pdu_dmac_sync->dm_sdu_len = 5;
			break;
		case DM_OCCUPIED:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 21);
			pdu_dmac_sync->message_fields_len = 21;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+21, 5);
			pdu_dmac_sync->dm_sdu_len = 5;
		break;
		case DM_RELEASE:
			// memcpy(pdu_dmac_sync->message_fields, bits+pointer, 21);
			pdu_dmac_sync->message_fields_len = 0;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer, 9);
			pdu_dmac_sync->dm_sdu_len = 9;
		break;
		case DM_TX_CEASED:
			memcpy(pdu_dmac_sync->message_fields, bits+pointer, 32);
			pdu_dmac_sync->message_fields_len = 32;
			memcpy(pdu_dmac_sync->dm_sdu, bits+pointer+32, 4);
			pdu_dmac_sync->dm_sdu_len = 4;
		break;

	}

	uint8_t dcc_rep = pdu_dmac_sync->repgw_address & 0x3f;
	uint32_t dcc_srcaddr = pdu_dmac_sync->src_address & 0xffffff;;
	pdu_dmac_sync->dm_colour_code = (dcc_srcaddr) | (dcc_rep << 24); 

	return pointer;

}


static void rx_dm_dpressync_sch_s(uint8_t *bits, struct tetra_dmo_pdu_dpres_sync *sync) {
	sync->system_code = bits_to_uint(bits, 4);
	sync->sync_pdu_type = bits_to_uint(bits+4, 2);
	sync->communication_type = bits_to_uint(bits+6, 2);
	sync->m_dmo_flag = bits_to_uint(bits+8, 1);
	sync->twofreq_repeater_flag = bits_to_uint(bits+11, 1);
	sync->repeater_operating_modes = bits_to_uint(bits+12, 2);
	sync->spacing_of_uplink = bits_to_uint(bits+14, 6);
	sync->masterslave_link_flag = bits_to_uint(bits+20, 1);
	sync->channel_usage = bits_to_uint(bits+21, 2);
	sync->channel_state = bits_to_uint(bits+23, 2);
	sync->slot_number = bits_to_uint(bits+25, 2) + 1;
	sync->frame_number = bits_to_uint(bits+27, 5);
	sync->power_class = bits_to_uint(bits+32, 3);
	sync->power_control_flag = bits_to_uint(bits+36, 1);
	sync->frame_countdown = bits_to_uint(bits+37, 2);
	sync->priority_level = bits_to_uint(bits+39, 2);
	sync->dn232_dn233 = bits_to_uint(bits+47, 4);
	sync->dt254 = bits_to_uint(bits+51, 3);
	sync->dualwatch_sync_flag = bits_to_uint(bits+54, 1);

}

int rx_dm_dpressync_sch_h(uint8_t *bits, struct tetra_dmo_pdu_dpres_sync *sync) {

	int pointer = 0;
	sync->repgw_address = bits_to_uint(bits, 10);
	sync->mni = bits_to_uint(bits+10, 24);
	sync->validity_time_unit = bits_to_uint(bits+34, 2);
	sync->number_of_validity_time_units = bits_to_uint(bits+36, 6);
	sync->max_dmms_power_class = bits_to_uint(bits+42, 3);
	sync->usage_restriction_type = bits_to_uint(bits+46, 4);

	if (sync->usage_restriction_type >= 8) {
		sync->sckn = bits_to_uint(bits+50, 5);
		sync->edsi_urtc_initialization_value = bits_to_uint(bits+55, 19);
	}

	return pointer;

}

void send_dmac_sync_burst(struct tetra_dmo_pdu_dmac_sync *dmac_sync, enum tdma_master_slave_link_flag link_flag, uint8_t fn, uint8_t tn, uint8_t frame_countdown, struct tetra_mac_state *tms) 
{
	struct tetra_dmvsap_prim *tdp;
	struct dmv_unitdata_param *d_unitdata_param;
	struct msgb *msg;

	/* build DMAC-SYNC SCH/S part */
	tdp = dmvsap_prim_alloc(PRIM_DMV_UNITDATA, PRIM_OP_REQUEST);
	msg = tdp->oph.msg;
	d_unitdata_param = &tdp->u.unitdata;
	d_unitdata_param->colour_code = dmac_sync->dm_colour_code;
	struct tetra_tdma_time *time = &d_unitdata_param->tdma_time;
	time->fn = fn;
	time->tn = tn;
	time->link = link_flag;

	uint8_t sb_type2[80];
	int len = build_pdu_dmac_sync_schs(dmac_sync, link_flag, fn, tn, 0, sb_type2);

	printf("DMV-SAP SCH/S %s - ", osmo_ubit_dump(sb_type2, 60));
	d_unitdata_param->lchan = TETRA_LC_SCH_S;

	msg->l1h = msgb_put(msg, 60);
	memcpy(msg->l1h, sb_type2, 60);
	rx_dmv_unitdata_req(tdp, tms);

	/* build DMAC-SYNC SCH/H part */
	tdp = dmvsap_prim_alloc(PRIM_DMV_UNITDATA, PRIM_OP_REQUEST);
	msg = tdp->oph.msg;
	d_unitdata_param = &tdp->u.unitdata;
	d_unitdata_param->colour_code = dmac_sync->dm_colour_code;
	time = &d_unitdata_param->tdma_time;
	time->fn = fn;
	time->tn = tn;
	time->link = link_flag;

	uint8_t si_type2[140];
	len = build_pdu_dmac_sync_schh(dmac_sync, fn, tn, frame_countdown, si_type2);

	printf(" SCH/H - %s\n", osmo_ubit_dump(si_type2, 124));
	d_unitdata_param->lchan = TETRA_LC_SCH_H;

	msg->l1h = msgb_put(msg, 124);
	memcpy(msg->l1h, si_type2, 124);

	rx_dmv_unitdata_req(tdp, tms);
}

static void rx_dmo_signalling(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms)
{
	struct dmv_unitdata_param *tup = &dmvp->u.unitdata;
	struct msgb *msg = dmvp->oph.msg;
	uint8_t *bits = msg->l1h;
	int pointer = 12;
	struct msgb *fragmsgb;

	if (tup->lchan == TETRA_LC_SCH_S) {

		int slot = 0;
		int tmplen;

		// int len=msgb_l1len(msg);
		msg->l1h=bits+4;

		// if (fragslots[slot].active) printf("\nWARNING: leftover fragment slot\n");
		fragmsgb=fragslots[slot].msgb;
		// printf ("\nFRAGMENT START slot=%i msgb=%p\n",slot,fragmsgb);
		msgb_reset(fragmsgb);

		fragslots[slot].active=1;
		fragslots[slot].fragments=0;
		/* copy the original msgb */
		tmplen=msg->tail - msg->data;
		memcpy(msgb_put(fragmsgb,tmplen),msg->data, tmplen);
		fragslots[slot].length=tmplen;

		if (msg->l1h) {
			fragmsgb->l1h=((void *)msg->l1h-(void *)msg)+(void *)fragmsgb;
		} else {
			fragmsgb->l1h=0;
		}
		if (msg->l2h) {
			fragmsgb->l2h=((void *)msg->l2h-(void *)msg)+(void *)fragmsgb;
		} else {
			fragmsgb->l2h=0;
		}

		uint8_t pdu_system_code = bits_to_uint(bits, 4);
		uint8_t sync_pdu_type = bits_to_uint(bits+4, 2);
		pdu_dmac_sync->sync_pdu_type = sync_pdu_type;

		// should not we clear previous pdu_dmac_sync first?
		memset(pdu_dmac_sync->message_fields, 0, sizeof(pdu_dmac_sync->message_fields));
		memset(pdu_dmac_sync->dm_sdu, 0, sizeof(pdu_dmac_sync->dm_sdu));

		if (sync_pdu_type == 0) { //DMAC-SYNC PDU
			rx_dm_dmacsync_sch_s(bits, pdu_dmac_sync);
		} else {
			rx_dm_dpressync_sch_s(bits, pdu_dpres_sync);
		}


	} else { // tup->lchan == TETRA_LC_SCH_H
		if (pdu_dmac_sync->sync_pdu_type == 0) {
			pointer = rx_dm_dmacsync_sch_h(bits, pdu_dmac_sync);

			uint8_t mcc = (pdu_dmac_sync->mni >> 14);
			uint8_t mnc = (pdu_dmac_sync->mni & 0x3ff);

			printf("DMAC-SYNC PDU (%d): SCH/S - Comm %d, M/S %d, GW msg %d, AB %d, TN %d, FN %d, ENC %d \n", 
					pdu_dmac_sync->sync_pdu_type, pdu_dmac_sync->communication_type, pdu_dmac_sync->masterslave_link_flag, pdu_dmac_sync->gateway_message_flag, 
					pdu_dmac_sync->ab_channel_usage, pdu_dmac_sync->slot_number, pdu_dmac_sync->frame_number, pdu_dmac_sync->airint_encryption_state);
			printf("  REPGW %d, Fill %d, Frag %d, num %d, FN cnt %d, dst-type %d, dst-addr %d, src-type %d, src %d, mni %d (MCC %d, MNC %d), msg-type %d \n",
				pdu_dmac_sync->repgw_address, pdu_dmac_sync->fillbit_indication, pdu_dmac_sync->fragmentation_flag, pdu_dmac_sync->number_of_sch_f_slots, 
				pdu_dmac_sync->frame_countdown, pdu_dmac_sync->dest_address_type, pdu_dmac_sync->dest_address, pdu_dmac_sync->src_address_type, 
				pdu_dmac_sync->src_address, pdu_dmac_sync->mni, mcc, mnc, pdu_dmac_sync->message_type);


		} else {
			pointer = rx_dm_dpressync_sch_h(bits, pdu_dpres_sync);

			printf("DPRES-SYNC PDU (%d): SCH/S - Comm: %d, M-DMO: %d, 2-freq rep: %d, rep. operating modes: %d, spacing: %d, M/S link: %d, ch. usage: %d, ch. state: %d \n" \
			 "  TN: %d, FN: %d, powerclass: %d, power control: %d, frame countdown: %d, priority: %d, DN232/DN233: %d, DT254: %d, dualwatch sync: %d \n" \
			 "SCH/H - REP addr: %d, DM-REP MNI: %d, Validity unit: %d, validity units: %d, DM-MS max power-class: %d, URT: %d \n", 
				pdu_dpres_sync->sync_pdu_type, pdu_dpres_sync->communication_type, pdu_dpres_sync->m_dmo_flag, pdu_dpres_sync->twofreq_repeater_flag, 
				pdu_dpres_sync->repeater_operating_modes, pdu_dpres_sync->spacing_of_uplink, pdu_dpres_sync->masterslave_link_flag, pdu_dpres_sync->channel_usage,
				pdu_dpres_sync->channel_state, pdu_dpres_sync->slot_number, pdu_dpres_sync->frame_number, pdu_dpres_sync->power_class, pdu_dpres_sync->power_control_flag,
				pdu_dpres_sync->frame_countdown, pdu_dpres_sync->priority_level, pdu_dpres_sync->dn232_dn233, pdu_dpres_sync->dt254, pdu_dpres_sync->dualwatch_sync_flag,
				pdu_dpres_sync->repgw_address, pdu_dpres_sync->mni, pdu_dpres_sync->validity_time_unit, pdu_dpres_sync->number_of_validity_time_units, 
				pdu_dpres_sync->max_dmms_power_class, pdu_dpres_sync->usage_restriction_type);

		}

		uint16_t my_repaddr = (REP_ADDRESS & 0x3ff);

		// ready to repeat DM-MASTER DM-SETUP / DM-SETUP PRES request
		if (pdu_dmac_sync->repgw_address==my_repaddr && pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->communication_type == 1  && pdu_dmac_sync->frame_countdown > 0 && (pdu_dmac_sync->message_type == DM_SETUP || pdu_dmac_sync->message_type == DM_SETUP_PRES) && pdu_dmac_sync->processed==false) {
			uint8_t master_fcountdown = pdu_dmac_sync->frame_countdown;
			uint8_t out_fn = (pdu_dmac_sync->frame_number+master_fcountdown+1) % 18;  // motoway fcn+1, spec fcn
			uint8_t out_tn = 1;
			tms->cur_burst.is_traffic = 1;
			tms->mode_of_operation = DM_MAC_MODE_TRAFFIC_SPEECH;

			pdu_dmac_sync->processed = true;
			printf("## we are gonna repeat - out starting at slave link FN: %d\n", out_fn);

			for (int fn=0; fn<(DN232+1); fn++) { 
				for (int tn=1; tn<5; tn++) {
					send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_SLAVE, out_fn+fn, tn, (DN232-fn), tms);
					tms->channel_state = DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED;
				}
			}

		} else if (pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->frame_countdown == 0 && (pdu_dmac_sync->message_type == DM_SETUP  || pdu_dmac_sync->message_type == DM_SETUP_PRES) && pdu_dmac_sync->processed==true) {
			pdu_dmac_sync->processed = false;
			printf("DM-SETUP processed flag cleared!\n");

		} else if (pdu_dmac_sync->sync_pdu_type == 0 && (pdu_dmac_sync->message_type == DM_SETUP || pdu_dmac_sync->message_type == DM_SETUP_PRES) && pdu_dmac_sync->processed==true) {
			printf("ignoring the repetitive DMAC-SYNC from DM-MASTER\n");

		// repeat DM-SLAVE DM-CONNECT request
		} else if (pdu_dmac_sync->repgw_address==my_repaddr && pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->communication_type == 1 && pdu_dmac_sync->frame_countdown > 0 && pdu_dmac_sync->message_type == DM_CONNECT && pdu_dmac_sync->processed == false) {
			uint8_t master_fcountdown = pdu_dmac_sync->frame_countdown;
			uint8_t out_fn = (pdu_dmac_sync->frame_number+master_fcountdown+1) % 18;  // motoway fcn+1, spec fcn
			uint8_t out_tn = 3;

			pdu_dmac_sync->processed = true;
			printf("## we are gonna repeat - out starting at master link FN: %d\n", out_fn);
			for (int fn=0; fn<=master_fcountdown; fn++) { 
				send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_MASTER, out_fn+fn, 1, (master_fcountdown-fn), tms);
				if ((master_fcountdown-fn) > 0) {
					send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_MASTER, out_fn+fn, 3, (master_fcountdown-fn), tms);
				}
			}

		} else if (pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->frame_countdown == 0 && (pdu_dmac_sync->message_type == DM_CONNECT) && pdu_dmac_sync->processed==true) {
			pdu_dmac_sync->processed = false;
			printf("DM-CONNECT processed flag cleared!\n");

		} else if (pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->message_type == DM_CONNECT && pdu_dmac_sync->processed==true) {
			printf("ignoring the repetitive DMAC-SYNC from DM-MASTER\n");

		// DMAC-SYNC DM-OCCUPIED
		} else if (tms->channel_state == DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED && pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->message_type == DM_OCCUPIED)  {
			// repeat DMAC-SYNC burst
			tms->channel_state_last_chg = 0; // reset counter for keepalive
			send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_SLAVE, pdu_dmac_sync->frame_number, pdu_dmac_sync->slot_number, 0, tms);


		// DMAC-SYNC DM-RELEASE - change channel state to free
		} else if (pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->message_type == DM_RELEASE) {
			tms->cur_burst.is_traffic = 0;
			tms->mode_of_operation = DM_MAC_MODE_SYNC_SIGNALLING;
			tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_FREE;
			tms->channel_state_last_chg = 0;
			send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_SLAVE, pdu_dmac_sync->frame_number, pdu_dmac_sync->slot_number, 0, tms);

			// rx_dmv_unitdata_req(NULL, tms);

		// DMAC-SYNC DM-RESERVED
		} else if (pdu_dmac_sync->sync_pdu_type == 0 && pdu_dmac_sync->message_type == DM_RESERVED) {
			tms->channel_state = DM_CHANNEL_S_DMREP_ACTIVE_RESERVED;
			tms->channel_state_last_chg = 0;
			send_dmac_sync_burst(pdu_dmac_sync, DM_LINK_SLAVE, pdu_dmac_sync->frame_number, pdu_dmac_sync->slot_number, 0, tms);

			// rx_dmv_unitdata_req(NULL, tms);

		}



		int slot = 0;
		// uint8_t fillbits_present=bits_to_uint(bits+10, 1); 
		uint8_t fillbits_present = pdu_dmac_sync->fillbit_indication;

		int len=msgb_l1len(msg);

		if (fragslots[slot].active) {
			int oldlen = fragslots[slot].length;
			append_frag_bits(slot,bits,len,fillbits_present);
			fragmsgb=fragslots[slot].msgb;
			fragmsgb->l2h=fragmsgb->l1h+(oldlen+pointer-4); // aim to the message type field of full message

		} else {
			printf("\nFRAG: got fragment without start packet for slot=%i\n",slot);
		}

	}

}

void rx_dmo_traffic(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms)
{
	printf("# what to do with traffic? repeat! #\n");
	struct msgb *tchmsg = dmvp->oph.msg;

	struct tetra_dmvsap_prim *tdp;
	tdp = dmvsap_prim_alloc(PRIM_DMV_UNITDATA, PRIM_OP_REQUEST);
	struct msgb *msg = tdp->oph.msg;
	struct dmv_unitdata_param *d_unitdata_param = &tdp->u.unitdata;

	d_unitdata_param->colour_code = pdu_dmac_sync->dm_colour_code;

	struct tetra_tdma_time *time = &d_unitdata_param->tdma_time;
	struct timing_slot *slot = tms->slot;
	time->fn = slot->fn;
	time->tn = slot->tn;
	time->link = DM_LINK_SLAVE;

	printf(" TCH - %s\n", osmo_ubit_dump(tchmsg->l1h, tchmsg->data_len));
	d_unitdata_param->lchan = TETRA_LC_TCH;

	msg->l1h = msgb_put(msg, tchmsg->data_len);
	memcpy(msg->l1h, tchmsg->l1h, tchmsg->data_len);

	rx_dmv_unitdata_req(tdp, tms);



}

/* used by layer 2 to deliver received message to layer 3 */
static int rx_dmv_unitdata_ind(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms)
{
	struct dmv_unitdata_param *tup = &dmvp->u.unitdata;
	struct msgb *msg = dmvp->oph.msg;
	uint8_t pdu_type = bits_to_uint(msg->l1h, 2);
	const char *pdu_name;
	struct msgb *gsmtap_msg;

	if (tup->lchan == TETRA_LC_SCH_S || tup->lchan == TETRA_LC_SCH_H) {
		pdu_name = "SYNC";
		// tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_OCCUPIED;
	} else {
		pdu_type = bits_to_uint(msg->l1h, 2);
		pdu_name = tetra_get_dmacpdu_name(pdu_type);
	}

	printf("DMV-UNITDATA.ind %s %s CRC=%u %s\n",
		tetra_tdma_time_dump(&tup->tdma_time),
		tetra_get_lchan_name(tup->lchan),
		tup->crc_ok, pdu_name);

	if (!tup->crc_ok)
		return 0;

	// int slot=tup->tdma_time.tn;
	int slot = 0;

	gsmtap_msg = tetra_gsmtap_makemsg(&tup->tdma_time, tup->lchan,
					  tup->tdma_time.tn,
					  /* FIXME: */ 0, 0, 0,
					msg->l1h, msgb_l1len(msg), tms);
	if (gsmtap_msg)
		tetra_gsmtap_sendmsg(gsmtap_msg);

	switch (tup->lchan) {
	case TETRA_LC_SCH_S:
	case TETRA_LC_SCH_H:
		rx_dmo_signalling(dmvp, tms); 
		break;

	case TETRA_LC_STCH:
		if (pdu_type == TETRA_PDU_T_DMAC_DATA) {
			rx_dmo_dmac_data(dmvp, tms);
		}
		break;

	case TETRA_LC_TCH:
		rx_dmo_traffic(dmvp, tms);
		break;

	case TETRA_LC_BNCH:
	case TETRA_LC_UNKNOWN:
	case TETRA_LC_SCH_F:
		switch (pdu_type) {
		case TETRA_PDU_T_DMAC_DATA:
			rx_dmo_dmac_data(dmvp, tms);
			break;
		case TETRA_PDU_T_MAC_FRAG_END:
			if (msg->l1h[3] == TETRA_MAC_FRAGE_FRAG) {
				printf("FRAG/END FRAG: ");
				msg->l2h = msg->l1h+4;
				// rx_tm_sdu(tms, msg, 100 /*FIXME*/);
				rx_macfrag(dmvp, tms, slot);
				printf("\n");
			} else {
				printf("FRAG/END END\n");
				rx_macend(dmvp, tms, slot);
			}
			break;
		case TETRA_PDU_T_DMAC_USIGNAL:
			printf("DMAC-U-SIGNAL PDU\n");
		default:
			printf("STRANGE pdu=%u\n", pdu_type);
			break;
		}
		break;
	case TETRA_LC_BSCH:
		break;
	default:
		printf("STRANGE lchan=%u\n", tup->lchan);
		break;
	}
	fflush(stdout);
	return 0;
}


int upper_mac_prim_recv(struct osmo_prim_hdr *op, void *priv)
{
	struct tetra_tmvsap_prim *tmvp;
	struct tetra_dmvsap_prim *dmvp;
	struct tetra_mac_state *tms = priv;
	int rc;

	switch (op->sap) {
	case TETRA_SAP_TMV:
		tmvp = (struct tetra_tmvsap_prim *) op;
		rc = rx_tmv_unitdata_ind(tmvp, tms);
		break;
	case TETRA_SAP_DMV:
		dmvp = (struct tetra_dmvsap_prim *) op;
		rc = rx_dmv_unitdata_ind(dmvp, tms);
		break;
	default:
		printf("primitive on unknown sap\n");
		break;
	}

	talloc_free(op->msg);
	talloc_free(op);

	return rc;
}
