#ifndef TETRA_PRIM_H
#define TETRA_PRIM_H

#include <stdint.h>

#include <osmocom/core/prim.h>

#include "tetra_common.h"

enum tetra_saps {
	TETRA_SAP_TP,	/* between PHY and lower MAC */
	TETRA_SAP_TMV,	/* between lower and upper MAC */
	TETRA_SAP_TMA,
	TETRA_SAP_TMB,
	TETRA_SAP_TMD,
	TETRA_SAP_DP,
	TETRA_SAP_DMV,
	TETRA_SAP_DMA,
	TETRA_SAP_DMC,
	TETRA_SAP_DMD
};

/* Table 23.1 */
enum tmv_sap_prim {
	PRIM_TMV_UNITDATA,
	PRIM_TMV_CONFIGURE,
};

/* Table 23.2 */
struct tmv_unitdata_param {
	uint32_t mac_block_len;		/* length of mac block */
	enum tetra_log_chan lchan;	/* to which lchan do we belong? */
	int crc_ok;			/* was the CRC verified OK? */
	uint32_t scrambling_code;	/* which scrambling code was used */
	struct tetra_tdma_time tdma_time;/* TDMA timestamp  */
	//uint8_t mac_block[412];		/* maximum num of bits in a non-QAM chan */
};

/* Table 23.3 */
struct tmv_configure_param {
	/* FIXME */
	uint32_t scrambling_rx;
};

struct tetra_tmvsap_prim {
	struct osmo_prim_hdr oph;
	union {
		struct tmv_unitdata_param unitdata;
		struct tmv_configure_param configure;
	} u;
};

enum dmv_sap_prim {
	PRIM_DMV_UNITDATA,
};

/* 300 396-3 - Table 19 */
struct dmv_unitdata_param {
	uint32_t mac_block_len;		/* length of mac block */
	enum tetra_log_chan lchan;	/* to which lchan do we belong? */
	int crc_ok;			/* was the CRC verified OK? */
	uint32_t colour_code;	/* which scrambling code was used */
	uint8_t rssi;
	uint32_t report;
	struct tetra_tdma_time tdma_time;/* TDMA timestamp  */
	//uint8_t mac_block[412];		/* maximum num of bits in a non-QAM chan */
};

struct tetra_dmvsap_prim {
	struct osmo_prim_hdr oph;
	union {
		struct dmv_unitdata_param unitdata;
	} u;
};

/* 300 396-3 - Table 10 */
struct dma_report_param {
	uint32_t handle;
	uint32_t report;
};

/* 300 396-3 - Table 11 */
struct dma_unitdata_param {
	uint16_t dm_sdu_length; 
	uint8_t dm_sdu_block[4096]; // or should be at oph->message ?
	uint8_t dest_address_type;
	uint32_t dest_address;
	uint8_t src_address_type;
	uint32_t src_address;
	uint8_t communication_type;
    uint8_t message_type;
	uint8_t airint_encryption_state;
    uint8_t priority_level;
    uint8_t circuit_mode_type;
	uint8_t sds_transaction_type;
	uint8_t fcs_flag;
	uint8_t recent_user_priority;
	uint8_t reservation_time;
	uint8_t new_call_preemption;
	uint8_t preemption_type;
	uint8_t stealing_priority;
	uint8_t number_of_repeats;
	uint8_t immediate_retransmission;
	uint8_t changeover_request;
	uint8_t recent_user_changeover_request;
	uint8_t call_setup_after_preemption_changeover;
	uint8_t incomplete_dm_sds_data_ack_received;
	uint8_t lower_level_quality_information;
	uint8_t dm_channel;
};

struct tetra_dmasap_prim {
	struct osmo_prim_hdr oph;
	union {
		struct dma_unitdata_param unitdata;
		struct dma_report_param report;
	} u;
	
};

#endif
