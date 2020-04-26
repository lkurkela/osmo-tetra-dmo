/* Alternative main program that directly plugs the L1/L2 protocol stack
 * into libsuo. This way, it runs in the same thread with signal processing
 * in the modem. */

#include <assert.h>
#include "suo.h"
#include "hamtetra_timing.h"
#include "hamtetra_slotter.h"
#include "signal-io/soapysdr_io.h"
#include "modem/burst_dpsk_receiver.h"
#include "modem/psk_transmitter.h"

struct timing_state *timing1;
struct slotter_state *slotter1;

#define BURST_MAXBITS 600
struct burst_bits {
	struct metadata m;
	uint8_t data[BURST_MAXBITS];
};


static int hamtetra_rx_output_frame(void *arg, const struct frame *in)
{
	int len = in->m.len;
	if (len < 0 || len > BURST_MAXBITS)
		return -1;

	struct burst_bits b;
	b.m = in->m;

	/* Convert to bits by hard decision */
	int i;
	for (i = 0; i < len; i++)
		b.data[i] = in->data[i] >= 0x80 ? 1 : 0;

	return timing_rx_burst(timing1, b.data, len, b.m.time);
}


static int hamtetra_tx_input_frame(void *arg, struct frame *out, size_t maxlen, timestamp_t timenow)
{
	int len;
	uint64_t ts = timenow;
	len = timing_tx_burst(timing1, out->data, BURST_MAXBITS, &ts);
	if (len >= 0) {
		assert(len <= BURST_MAXBITS);
		out->m.len = len;
		out->m.time = ts;
		out->m.flags = METADATA_TIME | METADATA_NO_LATE;
	}
	return len;
}


static int dummy_tick(void *arg, timestamp_t timenow)
{
	(void)arg; (void)timenow;
	return 0;
}


const struct rx_output_code hamtetra_rx_output_code = { "HamTetra", NULL, NULL, NULL, NULL, NULL, hamtetra_rx_output_frame, dummy_tick };

const struct tx_input_code hamtetra_tx_input_code = { "HamTetra", NULL, NULL, NULL, NULL, NULL, hamtetra_tx_input_frame, dummy_tick };


int main(void)
{
	const double samplerate = 0.5e6;

	// Operating frequency
	const double tetra_freq = 416.2375e6;

	// Offset from SDR center frequency to avoid 1/f noise and DC
	const double offset_freq = 37500;

	slotter1 = slotter_init();
	timing1 = timing_init();

	// Less timing margin is enough here
	timing1->ahead_time = 3e6;

	timing1->slotter = slotter1;
	slotter1->timing = timing1;

	struct burst_dpsk_receiver_conf *rx_conf = burst_dpsk_receiver_code.init_conf();
	rx_conf->samplerate = samplerate;
	rx_conf->centerfreq = offset_freq;
	void *rx_arg = burst_dpsk_receiver_code.init(rx_conf);

	burst_dpsk_receiver_code.set_callbacks(rx_arg, &hamtetra_rx_output_code, NULL);

	struct psk_transmitter_conf *tx_conf = psk_transmitter_code.init_conf();
	tx_conf->samplerate = samplerate;
	tx_conf->centerfreq = offset_freq;
	void *tx_arg = psk_transmitter_code.init(tx_conf);

	psk_transmitter_code.set_callbacks(tx_arg, &hamtetra_tx_input_code, NULL);

	struct soapysdr_io_conf *io_conf = soapysdr_io_code.init_conf();
	io_conf->samplerate = samplerate;
	io_conf->rx_centerfreq =
	io_conf->tx_centerfreq = tetra_freq - offset_freq;
	io_conf->rx_antenna = "LNAL";
	io_conf->tx_antenna = "BAND1";
	io_conf->rx_gain = 50;
	io_conf->tx_gain = 50;
	io_conf->buffer = 1024;
	io_conf->tx_latency = 6144;
	soapysdr_io_code.set_conf(io_conf, "soapy-driver", "lime");
	void *io_arg = soapysdr_io_code.init(io_conf);

	soapysdr_io_code.set_callbacks(io_arg, &burst_dpsk_receiver_code, rx_arg, &psk_transmitter_code, tx_arg);
	soapysdr_io_code.execute(io_arg);

	return 0;
}
