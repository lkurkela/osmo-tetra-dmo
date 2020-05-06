/* Alternative main program that directly plugs the L1/L2 protocol stack
 * into libsuo. This way, it runs in the same thread with signal processing
 * in the modem. */

#include "suo.h"
#include "hamtetra_timing.h"
#include "hamtetra_slotter.h"
#include "hamtetra_mac.h"
#include "signal-io/soapysdr_io.h"
#include "modem/burst_dpsk_receiver.h"
#include "modem/psk_transmitter.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct timing_state *timing1;
struct slotter_state *slotter1;
static const struct signal_io_code *io_code;
static void *io_arg;

#define BURST_MAXBITS 600
struct burst_msg {
	struct metadata m;
	uint8_t data[BURST_MAXBITS];
};


static int hamtetra_rx_output_frame(void *arg, const struct frame *in)
{
	int len = in->m.len;
	if (len < 0 || len > BURST_MAXBITS)
		return -1;

	struct burst_msg b;
	b.m = in->m;

	/* Convert to bits by hard decision */
	int i;
	for (i = 0; i < len; i++)
		b.data[i] = in->data[i] >= 0x80 ? 1 : 0;

	return timing_rx_burst(timing1, b.data, len, b.m.time);
}


static int hamtetra_tx_input_frame(void *arg, struct frame *out, size_t maxlen, uint64_t ts)
{
	int len;
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


static int hamtetra_init(const char *hw, double tetra_freq, int mode)
{
	// Sample rate for SDR
	double samplerate = 0.5e6;

	// Offset from SDR center frequency to avoid 1/f noise and DC
	double offset_freq = 37500;

	slotter1 = slotter_init();
	timing1 = timing_init();

	timing1->slotter = slotter1;
	slotter1->timing = timing1;

	mac_hamtetra_init();

	if (strcmp(hw, "file") == 0) {
		return -1; // not implemented
		//io_code = &file_io_code;
		// TODO: file I/O
	} else {
		io_code = &soapysdr_io_code;
		struct soapysdr_io_conf *io_conf = soapysdr_io_code.init_conf();
		io_conf->rx_on = 1;
		io_conf->tx_on = 1;
		if (mode == 0) // monitor only
			io_conf->tx_on = 0;

		io_conf->buffer = 1024;
		io_conf->tx_latency = 6144;

		// SDR specific configuration
		if (strcmp(hw, "limesdr") == 0) {
			io_conf->rx_antenna = "LNAL";
			io_conf->tx_antenna = "BAND1";
			goto limesdr_common;
		} else if (strcmp(hw, "limemini") == 0) {
			io_conf->rx_antenna = "LNAW";
			io_conf->tx_antenna = "BAND2";
			goto limesdr_common;
		} else if (strcmp(hw, "limenet") == 0) {
			io_conf->rx_antenna = "LNAL";
			io_conf->tx_antenna = "BAND2";
		limesdr_common:
			soapysdr_io_code.set_conf(io_conf, "device:driver", "lime");
			soapysdr_io_code.set_conf(io_conf, "rx_stream:latency", "0");
			soapysdr_io_code.set_conf(io_conf, "tx_stream:latency", "0");
			io_conf->rx_gain = 50;
			io_conf->tx_gain = 50;

		} else if (strcmp(hw, "usrp") == 0) {
			soapysdr_io_code.set_conf(io_conf, "device:driver", "uhd");
			io_conf->rx_antenna = "TX/RX";
			io_conf->tx_antenna = "TX/RX";
			io_conf->rx_gain = 50;
			io_conf->tx_gain = 70;

		} else if (strcmp(hw, "audio-in") == 0) {
			// Receive only. Full-duplex audio is not supported yet
			io_conf->tx_on = 0;
			goto audio_common;
		} else if (strcmp(hw, "audio-out") == 0) {
			/* Transmit only. Does not actually work
			 * since SoapyAudio has no transmit support. */
			io_conf->rx_on = 0;
		audio_common:
			soapysdr_io_code.set_conf(io_conf, "device:driver", "audio");
			samplerate = 48000;
			offset_freq = 12000;

		} else if (strcmp(hw, "rtlsdr") == 0) {
			soapysdr_io_code.set_conf(io_conf, "device:driver", "rtlsdr");
			io_conf->tx_on = 0;
			samplerate = 300000;
			io_conf->buffer = 0x4000;
			io_conf->rx_gain = 40;
		} else {
			printf("Unknown hardware %s\n", hw);
			return -1;
		}

		io_conf->samplerate = samplerate;
		io_conf->rx_centerfreq =
		io_conf->tx_centerfreq = tetra_freq - offset_freq;

		if (!io_conf->rx_on) {
			// This works better for TX-only use
			io_conf->tx_cont = 1;
			io_conf->use_time = 0;
		}

		io_arg = soapysdr_io_code.init(io_conf);
	}
	if (io_arg == NULL)
		return -1;

	struct burst_dpsk_receiver_conf *rx_conf = burst_dpsk_receiver_code.init_conf();
	rx_conf->samplerate = samplerate;
	rx_conf->centerfreq = offset_freq;
	rx_conf->syncpos = 143; // To get complete slots for DMO
	void *rx_arg = burst_dpsk_receiver_code.init(rx_conf);

	if (rx_arg == NULL)
		return -1;

	burst_dpsk_receiver_code.set_callbacks(rx_arg, &hamtetra_rx_output_code, NULL);

	struct psk_transmitter_conf *tx_conf = psk_transmitter_code.init_conf();
	tx_conf->samplerate = samplerate;
	tx_conf->centerfreq = offset_freq;
	void *tx_arg = psk_transmitter_code.init(tx_conf);

	if (tx_arg == NULL)
		return -1;

	psk_transmitter_code.set_callbacks(tx_arg, &hamtetra_tx_input_code, NULL);

	io_code->set_callbacks(io_arg, &burst_dpsk_receiver_code, rx_arg, &psk_transmitter_code, tx_arg);
	return 0;
}


int main(int argc, char *argv[])
{
	if (argc < 3) {
		fprintf(stderr,
			"Use: %s HARDWARE FREQUENCY [MODE]\n"
			"HARDWARE options are:\n"
			"   limesdr   LimeSDR USB, antennas on RX1_L and TX1_1 ports\n"
			"   limemini  LimeSDR Mini\n"
			"   limenet   LimeNET Micro\n"
			"   usrp      USRP B200\n"
			"   rtlsdr    RTL-SDR (receive only)\n"
			"   audio-in  Audio centered at 12 kHz (receive only)\n"
			"MODE options are:\n"
			"   0         DMO monitor\n"
			"   1         DMO repeater (default)\n"
			"FREQUENCY is TETRA signal center frequency in MHz.\n"
			"For example: %s limemini 416.2375 0\n",
			argv[0], argv[0]);
		return 1;
	}

	int mode = 1;
	if (argc >= 4)
		mode = atoi(argv[3]);
	if (hamtetra_init(argv[1], 1e6 * atof(argv[2]), mode) < 0) {
		fprintf(stderr, "Initialization failed\n");
		return 2;
	}

	io_code->execute(io_arg);

	return 0;
}
