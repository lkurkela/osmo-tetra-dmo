#include "timing.h"
#include <stdlib.h>
#include <stdio.h>

struct timing_state *timing_init()
{
	struct timing_state *s;
	s = calloc(1, sizeof(*s));

	// TODO

	return s;
}


int timing_rx_burst(struct timing_state *s, const uint8_t *bits, int len, uint64_t ts)
{
	printf("%20ld: RX\n", ts);
	// TODO
	return 0;
}


int timing_tx_burst(struct timing_state *s, uint8_t *bits, int maxlen, uint64_t *ts)
{
	//uint64_t tnow = *ts;
	// TODO
	return -1;
}
