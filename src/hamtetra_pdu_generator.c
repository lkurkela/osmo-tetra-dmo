/*  This class contains functions for PDU generation */

#include "tetra_common.h"
#include "tetra_dll_pdu.h"
#include "hamtetra_pdu_generator.h"
#include <lower_mac/tetra_lower_mac.h>

int build_pdu_dpres_sync_schs(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out)
{
    uint8_t pdu_sync_SCHS[8];		/* 60 bits */
    struct bitvec bv;

	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHS;
	bv.data_len = sizeof(pdu_sync_SCHS);

    uint8_t dn232_dn233 = DN233 | (DN232 << 2);

	//bitvec_set_uint(&bv, 0, 4);	/* alignment */
	/* According to Table 21.73: SYNC PDU Contents */
	bitvec_set_uint(&bv, 13, 4);	/* System Code  */
	bitvec_set_uint(&bv, 1, 2);	    /* Sync PDU type */
	bitvec_set_uint(&bv, 1, 2);	    /* Communication Type */
    bitvec_set_uint(&bv, 0, 1);	    /* M-DMO flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Reserved */
    bitvec_set_uint(&bv, 0, 1);	    /* Two-frequency repeater flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Repeater operating modes */
    bitvec_set_uint(&bv, 0, 6);	    /* Spacing of uplink */
    bitvec_set_uint(&bv, 0, 1);	    /* Master/slave link flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Channel usage */
    bitvec_set_uint(&bv, 0, 2);	    /* Channel state */
    bitvec_set_uint(&bv, tn, 2);	/* Slot number */
	bitvec_set_uint(&bv, fn, 5);	/* Frame number */
    bitvec_set_uint(&bv, 1, 3);	    /* Power class */
    bitvec_set_uint(&bv, 1, 1);	    /* Power control flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
    bitvec_set_uint(&bv, frame_countdown, 2);	/* Frame countdown */
    bitvec_set_uint(&bv, 0, 2);	    /* Reserved / priority */
    bitvec_set_uint(&bv, 0, 6);	    /* Reserved */
    bitvec_set_uint(&bv, dn232_dn233, 4);	    /* Values of DN232 and DN233 */
    bitvec_set_uint(&bv, DT254, 3); /* Value of DT254 */
    bitvec_set_uint(&bv, 0, 1);	    /* Presence signal dual watch sync flag */
    bitvec_set_uint(&bv, 0, 5);	    /* Reserved */

	// printf("DPRES-SYNC SCH/S PDU: %s ", osmo_hexdump(pdu_sync_SCHS, sizeof(pdu_sync_SCHS)));
   	uint8_t sb_type2[80];
   	uint8_t sb_type5[120];

	memset(sb_type2, 0, sizeof(sb_type2));
   	osmo_pbit2ubit(sb_type2, pdu_sync_SCHS, 60);
    build_encoded_block_sch(DPSAP_T_SCH_S, sb_type2, out);

    return 0;
}

int build_pdu_dpres_sync_schh(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out)
{
    uint8_t pdu_sync_SCHH[16];	    /* 124 bits */
    struct bitvec bv;

	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHH;
	bv.data_len = sizeof(pdu_sync_SCHH);

    bitvec_set_uint(&bv, REP_ADDRESS, 10);	/* Repeater address */
    bitvec_set_uint(&bv, REP_MCC, 10);	    /* Repeater MNI-MCC */
    bitvec_set_uint(&bv, REP_MNC, 14);	    /* Repeater MNI-MNC */
	bitvec_set_uint(&bv, 3, 2);	    /* Validity time unit (3=not restricted) */
    bitvec_set_uint(&bv, 0, 6);	    /* Validity time unit count */
	bitvec_set_uint(&bv, 1, 3);	    /* Max DM-MS power class */
	bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
	bitvec_set_uint(&bv, 0, 4);	    /* Usage restriction type (URT) */
    bitvec_set_uint(&bv, 0, 72);	/* URT addressing  */
	bitvec_set_uint(&bv, 0, 2);	    /* Reserved */

	// printf(" SCH/H PDU: %s\n", osmo_hexdump(pdu_sync_SCHH, sizeof(pdu_sync_SCHH)));
   	uint8_t si_type2[140];
	uint8_t si_type5[216];

	memset(si_type2, 0, sizeof(si_type2));
	osmo_pbit2ubit(si_type2, pdu_sync_SCHH, 124);
    build_encoded_block_sch(DPSAP_T_SCH_H, si_type2, out);

    return 0;
}


int build_pdu_dpress_sync(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t channel_state, uint8_t *out)
{
	uint8_t sb_type5[120];
	uint8_t si_type5[216];

    build_pdu_dpres_sync_schs(fn, tn, frame_countdown, sb_type5);
    build_pdu_dpres_sync_schh(fn, tn, frame_countdown, si_type5);

    int len = build_dm_sync_burst(out, sb_type5, si_type5);
    printf("DPRES-SYNC burst: %s\n", osmo_ubit_dump(out, len));
    return len;
}

int build_pdu_dpress_sync_gate(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out)
{
    uint8_t pdu_sync_SCHS[8];		/* 60 bits */
    uint8_t pdu_sync_SCHH[16];	    /* 124 bits */

    struct bitvec bv;

	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHS;
	bv.data_len = sizeof(pdu_sync_SCHS);

    uint8_t dn232_dn233 = DN233 | (DN232 << 2);

	//bitvec_set_uint(&bv, 0, 4);	/* alignment */
	/* According to Table 21.73: SYNC PDU Contents */
	bitvec_set_uint(&bv, 13, 4);	/* System Code  */
	bitvec_set_uint(&bv, 1, 2);	    /* Sync PDU type */
	bitvec_set_uint(&bv, 3, 2);	    /* Communication Type DM-REP/GATE */
    bitvec_set_uint(&bv, 0, 1);	    /* M-DMO flag */
    bitvec_set_uint(&bv, 1, 1);	    /* SwMI availability flag */
    bitvec_set_uint(&bv, 1, 1);	    /* DM-REP function flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Two-frequency repeater flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Repeater operating modes */
    bitvec_set_uint(&bv, 0, 6);	    /* Spacing of uplink */
    bitvec_set_uint(&bv, 0, 1);	    /* Master/slave link flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Channel usage */
    bitvec_set_uint(&bv, 3, 2);	    /* Channel state */
    bitvec_set_uint(&bv, tn, 2);	/* Slot number */
	bitvec_set_uint(&bv, fn, 5);	/* Frame number */
    bitvec_set_uint(&bv, 1, 3);	    /* Power class */
    bitvec_set_uint(&bv, 1, 1);	    /* Power control flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Registration phase terminated  */
    bitvec_set_uint(&bv, frame_countdown, 2);	/* Frame countdown */
    bitvec_set_uint(&bv, 0, 2);	    /* Reserved / Timing for DM-REP / priority */
    bitvec_set_uint(&bv, 2, 2);	    /* Registrations permited */
    bitvec_set_uint(&bv, 0, 4);	    /* Registration label */
    bitvec_set_uint(&bv, 4, 4);	    /* Registration phase time remaining */
    // bitvec_set_uint(&bv, dn232_dn233, 4);	    /* Values of DN232 and DN233 */
    bitvec_set_uint(&bv, 1, 3); /* Value of DT254 / registration access parameter */
    // bitvec_set_uint(&bv, DT254, 3); /* Value of DT254 / registration access parameter */
    bitvec_set_uint(&bv, 0, 1);	    /* Registrations forwarded flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Gateway encryption state */
    bitvec_set_uint(&bv, 0, 1);	    /* System wide services not available */
    bitvec_set_uint(&bv, 0, 3);	    /* Reserved */

	// printf("DPRES-SYNC SCH/S PDU: %s ", osmo_hexdump(pdu_sync_SCHS, sizeof(pdu_sync_SCHS)));
   	uint8_t sb_type2[80];
   	uint8_t sb_type5[120];

	memset(sb_type2, 0, sizeof(sb_type2));
   	osmo_pbit2ubit(sb_type2, pdu_sync_SCHS, 60);
    build_encoded_block_sch(DPSAP_T_SCH_S, sb_type2, sb_type5);


	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHH;
	bv.data_len = sizeof(pdu_sync_SCHH);

    bitvec_set_uint(&bv, REP_ADDRESS, 10);	/* Repeater address */
    bitvec_set_uint(&bv, REP_MCC, 10);	    /* Repeater MNI-MCC */
    bitvec_set_uint(&bv, REP_MNC, 14);	    /* Repeater MNI-MNC */
	bitvec_set_uint(&bv, 3, 2);	    /* Validity time unit (3=not restricted) */
    bitvec_set_uint(&bv, 0, 6);	    /* Validity time unit count */
	bitvec_set_uint(&bv, 1, 3);	    /* Max DM-MS power class */
	bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
	bitvec_set_uint(&bv, 0, 4);	    /* Usage restriction type (URT) */
    bitvec_set_uint(&bv, 0, 72);	/* URT addressing  */
	bitvec_set_uint(&bv, 0, 2);	    /* Reserved */

	// printf(" SCH/H PDU: %s\n", osmo_hexdump(pdu_sync_SCHH, sizeof(pdu_sync_SCHH)));
   	uint8_t si_type2[140];
	uint8_t si_type5[216];

	memset(si_type2, 0, sizeof(si_type2));
	osmo_pbit2ubit(si_type2, pdu_sync_SCHH, 124);
    build_encoded_block_sch(DPSAP_T_SCH_H, si_type2, si_type5);

    int len = build_dm_sync_burst(out, sb_type5, si_type5);
    printf("DPRES-SYNC burst: %s\n", osmo_ubit_dump(out, len));
    return len;
}

int build_pdu_dmac_sync_schs(struct tetra_dmo_pdu_dmac_sync *dmac_sync, uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out)
{
    uint8_t pdu_sync_SCHS[8];		/* 60 bits */
    uint8_t sb_type2[80];

    struct bitvec bv;
    memset(&bv, 0, sizeof(bv));
    bv.data = pdu_sync_SCHS;
    bv.data_len = sizeof(pdu_sync_SCHS);

    bitvec_set_uint(&bv, dmac_sync->system_code, 4);	/* System Code  */
    bitvec_set_uint(&bv, 0, 2);	    /* Sync PDU type */
    bitvec_set_uint(&bv, 1, 2);	    /* Communication Type */
    bitvec_set_uint(&bv, 0, 1);	    /* Master/slave link flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Gateway generated message flag */
    bitvec_set_uint(&bv, 0, 2);	    /* A/B channel usage */
    bitvec_set_uint(&bv, tn, 2);	/* Slot number */
    bitvec_set_uint(&bv, fn, 5);	/* Frame number */
    bitvec_set_uint(&bv, dmac_sync->airint_encryption_state, 2);   /* Air interface encryption state */
    bitvec_set_uint(&bv, 0, 39);	    /* Reserved, encryption not yet supported */

    osmo_pbit2ubit(out, pdu_sync_SCHS, 60);

    return bv.data_len;

}

int build_pdu_dmac_sync_schh(struct tetra_dmo_pdu_dmac_sync *dmac_sync, uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out)
{

    uint8_t pdu_sync_SCHH[16];	    /* 124 bits */
    uint8_t si_type2[140];

    struct bitvec bv;
    memset(&bv, 0, sizeof(bv));
    bv.data = pdu_sync_SCHH;
    bv.data_len = sizeof(pdu_sync_SCHH);

    bitvec_set_uint(&bv, dmac_sync->repgw_address, 10);	/* Repeater address */
    bitvec_set_uint(&bv, dmac_sync->fillbit_indication, 1);	    /* Fillbit indication */
    bitvec_set_uint(&bv, dmac_sync->fragmentation_flag, 1);	    /* Fragment flag */
    bitvec_set_uint(&bv, 1-fn, 2);	    /* frame countdown */
    bitvec_set_uint(&bv, dmac_sync->dest_address_type, 2);	    /* destination address type */
    bitvec_set_uint(&bv, dmac_sync->dest_address, 24);	    /* destination address */
    bitvec_set_uint(&bv, dmac_sync->src_address_type, 2);	    /* source address type */
    bitvec_set_uint(&bv, dmac_sync->src_address, 24);	    /* source address */
    bitvec_set_uint(&bv, dmac_sync->mni, 24);	/* MNI  */
    bitvec_set_uint(&bv, dmac_sync->message_type, 5);	    /* Message type */
    // bitvec_set_uint(&bv, dmac_sync->message_fields, pdu_dmac_sync->message_fields_len);	    /* Message dependent elements */
    // bitvec_add_array(&bv, dmac_sync->message_fields, pdu_dmac_sync->message_fields_len, false, 8);
    for (int i=0; i<dmac_sync->message_fields_len; i++) {
        bitvec_set_uint(&bv, dmac_sync->message_fields[i], 1);
    }
    // bitvec_set_uint(&bv, pdu_dmac_sync->dm_sdu, pdu_dmac_sync->dm_sdu_len);	    /* DM-SDU */
    // bitvec_add_array(&bv, pdu_dmac_sync->dm_sdu, pdu_dmac_sync->dm_sdu_len, false, 8);
    for (int i=0; i<dmac_sync->dm_sdu_len; i++) {
        bitvec_set_uint(&bv, dmac_sync->dm_sdu[i], 1);
    }
    if (dmac_sync->fillbit_indication==1) {
        bitvec_set_uint(&bv, 1, 1);
    }

    osmo_pbit2ubit(out, pdu_sync_SCHH, 124);

    return bv.data_len;

}

