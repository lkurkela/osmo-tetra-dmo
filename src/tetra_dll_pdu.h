#ifndef TETRA_DLL_PDU_H
#define TETRA_DLL_PDU_H

#include <osmocom/core/linuxlist.h>

/* Table 21.1 */
enum tetra_dll_pdu_t {
	TDLL_PDUT_DMAC_SYNC		= 0,
	TDLL_PDUT_DMAC_DATA		= 1,
	TDLL_PDUT_DMAC_FRAG		= 2,
	TDLL_PDUT_DMAC_END		= 3,
	TDLL_PDUT_DMAC_U_SIGNAL	= 4,
	TDLL_PDUT_DMAC_TRAFFIC	= 5,
};
const char *tetra_get_llc_pdut_name(uint8_t pdut);

/* TETRA DLL state */
struct tdll_state {
	struct llist_head list;

	struct {
		struct llist_head defrag_list;
	} rx;
};

/* entry in the defragmentation queue */
struct tdll_defrag_q_e {
	struct llist_head list;
	unsigned int ns;	/* current de-fragmenting */
	unsigned int last_ss;	/* last received S(S) */

	struct msgb *tl_sdu;
};

struct tetra_dmo_pdu_dmac_sync {
    uint8_t system_code;
    uint8_t sync_pdu_type;
	uint8_t communication_type;
	uint8_t masterslave_link_flag;
	uint8_t gateway_message_flag;
	uint8_t ab_channel_usage;
    uint8_t slot_number;
    uint8_t frame_number;
	uint8_t airint_encryption_state;
    uint32_t time_variant_parameter;
    uint8_t ksg_number;
    uint8_t encryption_key_number;
	uint16_t repgw_address;
	uint8_t fillbit_indication;
	uint8_t fragmentation_flag;
	uint8_t number_of_sch_f_slots;
	uint8_t frame_countdown;
	uint8_t dest_address_type;
	uint32_t dest_address;
	uint8_t src_address_type;
	uint32_t src_address;
	uint32_t mni;
    uint8_t message_type;
	uint8_t message_fields[128]; // message dependent fields;
	uint8_t message_fields_len;
	uint8_t *dm_sdu[128]; //message 
	uint8_t dm_sdu_len;
	bool processed;
};
static struct tetra_dmo_pdu_dmac_sync _pdu_dmac_sync, *pdu_dmac_sync = &_pdu_dmac_sync;

struct tetra_dmo_pdu_dpres_sync {
    uint8_t system_code;
    uint8_t sync_pdu_type;
	uint8_t communication_type;
	uint8_t m_dmo_flag;
	uint8_t twofreq_repeater_flag;
	uint8_t repeater_operating_modes;
	uint8_t spacing_of_uplink;	
	uint8_t masterslave_link_flag;
	uint8_t channel_usage;
	uint8_t channel_state;
    uint8_t slot_number;
    uint8_t frame_number;
	uint8_t power_class;
	uint8_t power_control_flag;
	uint8_t frame_countdown;
	uint8_t priority_level;
	uint8_t dn232_dn233;
	uint8_t dt254;
	uint8_t dualwatch_sync_flag;
	uint16_t repgw_address;
	uint32_t mni;
	uint8_t validity_time_unit;
	uint8_t number_of_validity_time_units;
	uint8_t max_dmms_power_class;
	uint8_t usage_restriction_type;
	uint8_t sckn;
	uint32_t edsi_urtc_initialization_value;
	__uint128_t urt_addressing;
};
static struct tetra_dmo_pdu_dpres_sync _pdu_dpres_sync, *pdu_dpres_sync = &_pdu_dpres_sync;


struct tetra_dmo_pdu_dm_setup {
    // message dependent elements
    uint8_t timing_flag;
    uint8_t lch_in_frame_3;
    uint8_t preemption_flag;
    uint8_t power_class;
    uint8_t power_control_flag;
    uint8_t dualwatch_sync_flag;
    uint8_t twofreq_call_flag;
    uint8_t circuit_mode_type;
    uint8_t priority_level;
    // DM-SDU elements
    uint8_t end_to_end_enc_flag;
    uint8_t call_type_flag;
    uint8_t ext_source_flag;
};

#endif /* TETRA_DLL_PDU_H */
