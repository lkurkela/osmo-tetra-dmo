#ifndef TETRA_UPPER_MAC_H
#define TETRA_UPPER_MAC_H

#include "tetra_prim.h"

enum tetra_dmo_message_type {
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

static const struct value_string tetra_dmo_message_type_names[] = {
	{ DM_RESERVED,		"DM-RESERVED" },
    { DM_SDS_OCCUPIED,  "DM-SDS OCCUPIED" },
	{ DM_TIMING_REQ,    "DM-TIMING REQUEST" },
	{ DM_TIMING_ACK,    "DM-TIMING ACK" },
	{ DM_SETUP,         "DM-SETUP" },
	{ DM_SETUP_PRES,    "DM-SETUP PRES" },
	{ DM_CONNECT,       "DM-CONNECT" },
	{ DM_DISCONNECT,    "DM-DISCONNECT" },
	{ DM_CONNECT_ACK,   "DM-CONNECT ACK" },
	{ DM_OCCUPIED,      "DM-OCCUPIED" },
	{ DM_RELEASE,       "DM-RELEASE" },
	{ DM_TX_CEASED,     "DM-TX CEASED" },
	{ DM_TX_REQ,        "DM-TX REQUEST" },
	{ DM_TX_ACCEPT,     "DM-TX ACCEPT" },
	{ DM_PREEMPT,       "DM-PREEMPT" },
	{ DM_PRE_ACCEPT,    "DM-PRE ACCEPT" },
	{ DM_REJECT,        "DM-REJECT" },
	{ DM_INFO,          "DM-INFO" },
	{ DM_SDS_UDATA,     "DM-SDS UDATA" },
	{ DM_SDS_DATA,      "DM-SDS DATA" },
	{ DM_SDS_ACK,       "DM-SDS ACK" },
	{ GW_SPECIFIC_MESSAGE,  "GATEWAY SPECIFIC MSG" },
	{ 0, NULL }
};
const char *tetra_get_dmo_message_type_name(uint8_t pdut);

int upper_mac_prim_recv(struct osmo_prim_hdr *op, void *priv);

#endif
