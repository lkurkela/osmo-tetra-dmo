#ifndef TETRA_BURST_H
#define TETRA_BURST_H

#include <stdint.h>

enum tp_sap_data_type {
	TPSAP_T_SB1,
	TPSAP_T_SB2,
	TPSAP_T_NDB,
	TPSAP_T_BBK,
	TPSAP_T_SCH_HU,
	TPSAP_T_SCH_F,
};

enum dp_sap_data_type {
	DPSAP_T_DSB1,
	DPSAP_T_DSB2,
	DPSAP_T_DNB1,
	DPSAP_T_DLB,
	DPSAP_T_SCH_F,
	DPSAP_T_SCH_S,
	DPSAP_T_SCH_H,
	DPSAP_T_STCH,
	DPSAP_T_TCH,
	DPSAP_T_TCH_S_F,
	DPSAP_T_TCH_S_H
};


extern void dp_sap_udata_ind(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv);
extern void tp_sap_udata_ind(enum tp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv);

/* 9.4.4.2.6 Synchronization continuous downlink burst */
int build_sync_c_d_burst(uint8_t *buf, const uint8_t *sb, const uint8_t *bb, const uint8_t *bkn);

/* 9.4.4.2.5 Normal continuous downlink burst */
int build_norm_c_d_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bb, const uint8_t *bkn2, int two_log_chan);

/* EN 300 396-2 - 9.4.3.2.1 DM Synchronization Burst */
int build_dm_sync_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2);

/* EN 300 396-2 - 9.4.3.2.1 DM Normal Burst */
/* Type: 1 if preamble 1 and training sequence 1 (TCH, SCH/F), anything else if preamble 2 and training sequence 2 (STCH+TCH, STCH+STCH)*/
int build_dm_norm_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2, const uint8_t type);

enum tetra_train_seq {
	TETRA_TRAIN_NORM_1,
	TETRA_TRAIN_NORM_2,
	TETRA_TRAIN_NORM_3,
	TETRA_TRAIN_SYNC,
	TETRA_TRAIN_EXT,
};

/* find a TETRA training sequence in the burst buffer indicated */
int tetra_find_train_seq(const uint8_t *in, unsigned int end_of_in,
			 uint32_t mask_of_train_seq, unsigned int *offset);

#endif /* TETRA_BURST_H */
