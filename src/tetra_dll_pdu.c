/* Implementation of some PDU parsing of the TETRA LLC */


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <osmocom/core/utils.h>

#include "tetra_common.h"
#include "tetra_dll_pdu.h"

static const struct value_string tetra_dll_pdut_names[] = {
	{ TDLL_PDUT_DMAC_SYNC,		"DMAC-SYNC" },
	{ 0, NULL }
};
const char *tetra_get_dll_pdut_name(uint8_t pdut)
{
	return get_value_string(tetra_dll_pdut_names, pdut);
}
