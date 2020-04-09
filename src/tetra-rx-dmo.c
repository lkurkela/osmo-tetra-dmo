/* Test program for tetra burst synchronizer */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <fcntl.h>
#include <sys/stat.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>

#include "tetra_common.h"
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_sync.h>
#include "tetra_gsmtap.h"

#include <zmq.h>
#include "suo.h"
#define ENCODED_MAXLEN 0x900

void *tetra_tall_ctx;
void *zmq_rx_socket;

int floats_to_bits(const struct frame *in, struct frame *out, size_t maxlen) 
{
	out->m = in->m; // Copy metadata
	size_t len = in->m.len;
	if (len > maxlen) len = maxlen;
	out->m.len = len;

	uint8_t buffy_in[len], buffy_out[len];
	memcpy(buffy_in, in->data, len);

	for (int i = 0; i < len; ++i) {
		int sym = buffy_in[i];
		if (sym < 128) {
			buffy_out[i] = 0;
		} else {
			buffy_out[i] = 1;
		}
	}

	// memcpy(out->data, buffy_out, len);
	// purkka
	memcpy(out->data, buffy_out+2, len-2);
	out->m.len = len-2;

	return 0;
}

int main(int argc, char **argv)
{
	int opt;
	struct tetra_rx_state *trs;
	struct tetra_mac_state *tms;

	tms = talloc_zero(tetra_tall_ctx, struct tetra_mac_state);
	tetra_mac_state_init(tms);
	tms->infra_mode = TETRA_INFRA_DMO; // FIXME 

	trs = talloc_zero(tetra_tall_ctx, struct tetra_rx_state);
	trs->burst_cb_priv = tms;

	while ((opt = getopt(argc, argv, "d:")) != -1) {
		switch (opt) {
		case 'd':
			tms->dumpdir = strdup(optarg);
			break;
		default:
			fprintf(stderr, "Unknown option %c\n", opt);
		}
	}

	if (argc <= optind) {
		fprintf(stderr, "Usage: %s [-d DUMPDIR] <rx-zmq-address>\n", argv[0]);
		exit(1);
	}

	const char *endpoint = argv[optind];
	void *zmq_context = zmq_ctx_new();
    zmq_rx_socket = zmq_socket(zmq_context, ZMQ_SUB);
    int connret = zmq_connect(zmq_rx_socket, argv[optind]);
	zmq_setsockopt(zmq_rx_socket, ZMQ_SUBSCRIBE, "", 0);

	// tetra_gsmtap_init("localhost", 0);

	while (1) {
		int nread;
		zmq_msg_t input_msg;
		zmq_msg_init(&input_msg);
		nread = zmq_msg_recv(&input_msg, zmq_rx_socket, 0);
		
		if (nread >= 0) {
			char encoded_buf[sizeof(struct frame) + ENCODED_MAXLEN];
			struct frame *encoded = (struct frame *)encoded_buf;

			int rc = floats_to_bits(zmq_msg_data(&input_msg), encoded, ENCODED_MAXLEN);

			tetra_burst_sync_in(trs, encoded->data, encoded->m.len);
		}
		zmq_msg_close(&input_msg);
	}

	zmq_ctx_destroy(zmq_context);

	free(tms->dumpdir);
	talloc_free(trs);
	talloc_free(tms);

	exit(0);
}
