#include <stdint.h>

extern uint8_t pdu_sync_SCHS[8];		/* 60 bits */
extern uint8_t pdu_sync_SCHH[16];	    /* 124 bits */	

void dm_sync_pdu_schh(uint16_t address, uint8_t rep_mcc, uint16_t rep_mnc);

void dm_sync_pres_pdu_schs(uint8_t tn, uint8_t fn, uint8_t frame_countdown, uint8_t dn232_dn233, uint8_t dt254);
