#ifndef TETRA-DMO-REP_H
#define TETRA-DMO-REP_H

#include "tetra_prim.h"

void dp_sap_udata_req(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, struct tetra_tdma_time tdma_time);

#endif