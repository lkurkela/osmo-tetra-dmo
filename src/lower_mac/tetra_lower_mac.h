#ifndef TETRA_LOWER_MAC_H
#define TETRA_LOWER_MAC_H

#include "tetra_prim.h"

struct tetra_dmvsap_prim *dmvsap_prim_alloc(uint16_t prim, uint8_t op);
int rx_dmv_unitdata_req(struct tetra_dmvsap_prim *dmvp, struct tetra_mac_state *tms);

#endif
