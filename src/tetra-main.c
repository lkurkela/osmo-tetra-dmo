#include <assert.h>
#include <zmq.h>
#include "suo.h"
#include "timing.h"
#include "slotter.h"

void *zmq_context;
struct timing_state *timing1;
struct slotter_state *slotter1;

#define BURST_MAXBITS 600
struct burst_bits {
	struct metadata m;
	uint8_t data[BURST_MAXBITS];
};

int main(void)
{
	void *zmq_rx_socket, *zmq_tx_socket;
	zmq_context = zmq_ctx_new();

	slotter1 = slotter_init();
	timing1 = timing_init();

	zmq_rx_socket = zmq_socket(zmq_context, ZMQ_SUB);
	/* Subscribe to both received frames and transmitter ticks */
	if (zmq_connect(zmq_rx_socket, "ipc:///tmp/dpsk-modem-rx") < 0)
		return 1;
	if (zmq_connect(zmq_rx_socket, "ipc:///tmp/dpsk-modem-tx-tick") < 0)
		return 2;
	zmq_setsockopt(zmq_rx_socket, ZMQ_SUBSCRIBE, "", 0);

	zmq_tx_socket = zmq_socket(zmq_context, ZMQ_PUB);
	if (zmq_connect(zmq_tx_socket, "ipc:///tmp/dpsk-modem-tx") < 0)
		return 3;

	struct burst_bits tx_msg = {
		{
			.id = 1,
			.flags = METADATA_TIME | METADATA_NO_LATE
		}
	};
	for (;;) {
		struct burst_bits rx_msg;
		int nread;
		nread = zmq_recv(zmq_rx_socket, &rx_msg, sizeof(rx_msg), 0);
		if (nread >= sizeof(struct metadata)) {
			/* It's a received burst */
			int len = nread - sizeof(struct metadata);

			/* Convert to bits by hard decision */
			int i;
			for (i = 0; i < len; i++)
				rx_msg.data[i] = rx_msg.data[i] >= 0x80 ? 1 : 0;

			timing_rx_burst(timing1, rx_msg.data, len, rx_msg.m.time);

		} else if (nread == sizeof(struct timing)) {
			/* It's a transmitter tick */

			int len;
			uint64_t ts = rx_msg.m.time;
			len = timing_tx_burst(timing1, tx_msg.data, BURST_MAXBITS, &ts);

			if (len >= 0) {
				assert(len <= BURST_MAXBITS);
				tx_msg.m.len = len;
				tx_msg.m.time = ts;
				zmq_send(zmq_tx_socket, &tx_msg, sizeof(struct metadata) + len, 0);
			}
		}
	}
	return 0;
}
