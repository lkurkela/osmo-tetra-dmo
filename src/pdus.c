#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/utils.h>

#include "pdus.h"


uint8_t pdu_sync_SCHS[8];		/* 60 bits */
uint8_t pdu_sync_SCHH[16];	    /* 124 bits */

void dm_sync_pdu_schh(uint16_t address, uint8_t rep_mcc, uint16_t rep_mnc){
	struct bitvec bv;
	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHH;
	bv.data_len = sizeof(pdu_sync_SCHH);

    bitvec_set_uint(&bv, address, 10);	/* Repeater address */
    bitvec_set_uint(&bv, rep_mcc, 10);	    /* Repeater MNI-MCC */
    bitvec_set_uint(&bv, rep_mnc, 14);	    /* Repeater MNI-MNC */
	bitvec_set_uint(&bv, 3, 2);	    /* Validity time unit (3=not restricted) */
    bitvec_set_uint(&bv, 0, 6);	    /* Validity time unit count */
	bitvec_set_uint(&bv, 1, 3);	    /* Max DM-MS power class */
	bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
	bitvec_set_uint(&bv, 0, 4);	    /* Usage restriction type (URT) */
    bitvec_set_uint(&bv, 0, 72);	/* URT addressing  */
	bitvec_set_uint(&bv, 0, 2);	    /* Reserved */

	// printf(" SCH/H PDU: %s\n", osmo_hexdump(pdu_sync_SCHH, sizeof(pdu_sync_SCHH)));
}

void dm_sync_pres_pdu_schs(uint8_t tn, uint8_t fn, uint8_t frame_countdown, uint8_t dn232_dn233, uint8_t dt254){
	struct bitvec bv;
	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHS;
	bv.data_len = sizeof(pdu_sync_SCHS);


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
    bitvec_set_uint(&bv, dt254, 3); /* Value of DT254 */
    bitvec_set_uint(&bv, 0, 1);	    /* Presence signal dual watch sync flag */
    bitvec_set_uint(&bv, 0, 5);	    /* Reserved */

	// printf("DPRES-SYNC SCH/S PDU: %s ", osmo_hexdump(pdu_sync_SCHS, sizeof(pdu_sync_SCHS)));

}
