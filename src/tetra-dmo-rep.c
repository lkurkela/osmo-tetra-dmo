/*
* This is TETRA Type 1A DM-REP implementation.
*
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include <math.h>
#include <sys/time.h>
#include <pthread.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>

#include <lower_mac/crc_simple.h>
#include <lower_mac/tetra_conv_enc.h>
#include <lower_mac/tetra_interleave.h>
#include <lower_mac/tetra_scramb.h>
#include <lower_mac/tetra_rm3014.h>
#include <lower_mac/viterbi.h>

#include "tetra_common.h"
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_sync.h>

#include <zmq.h>
#include "suo.h"

void *tetra_tall_ctx;
struct tetra_mac_state *tms;

#define ENCODED_MAXLEN 0x900
void *zmq_rx_socket;
void *zmq_tx_socket;

#define REP_MCC 244
#define REP_MNC 2
#define REP_ADDRESS 1099

#define DN232 1         // number of frames to transmit DM-SETUP (PRES) => 2
#define DN233 1         // number of frames to transmit DSB heading a DM-SDS UDATA/DATA => 2
#define DN253 2         // number of frames in with DM-REP transmits  the free-channel presence signal (2-4)
#define DT254 1         // Presence signal every 2 multiframes

#define swap16(x) ((x)<<8)|((x)>>8)


/* pfft begins - not invented here */
void tp_sap_udata_ind(enum tp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv) { }
void dp_sap_udata_ind(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv) { }
/* pfft ends */


static const int8_t presence_signal_multiframe_count[8] = {
	[0] = 0,
	[1] = 2,
	[2] = 5,
	[3] = 10,
    [4] = 15,
    [5] = 20,
    [6] = 30,
    [7] = 60
};

// timeslot struct
struct timeslot {
    uint8_t fn;
    uint8_t tn;
    uint64_t start_ts;  // start of timeslot in us
    uint16_t len;
    uint8_t burst[512]; // burst bit-per-byte encoded 
};

// multiframe buffer, 18 frames with 4 timeslots = 72 timeslots
struct multiframe {
    struct timeslot tn[72]; 
};

// global frame buffer to both threads
struct multiframe frame_buf;

// initialize the global framebuffer with timeslot numbers and timestamps
void initialize_framebuffer(uint64_t base_time_ts)
{
    uint64_t time_ts = base_time_ts; // microseconds

    for (int i = 0; i<72; i++) {
        struct timeslot *tn = &frame_buf.tn[i];
        tn->tn = (i%4) + 1;
        tn->fn = floor( (float)i/4 ) + 1;
        tn->len = 0;
        time_ts = time_ts + (85000/6);
        tn->start_ts = time_ts;
    }
};

// set next start timestamp of a timeslot for next round
void reprime_timeslot(uint8_t slotnum) {
    struct timeslot *tn = &frame_buf.tn[slotnum];
    uint64_t old_start = tn->start_ts;
    uint64_t multiframe_duration = 3060000/3;
    tn->start_ts = old_start + multiframe_duration;
    tn->len = 0;
};

// common function with tetra-rx-dmo so should be moved to common library
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

	memcpy(out->data, buffy_out+2, len-2);
	out->m.len = len-2;
	return 0;
}

int bits_to_floats(const struct frame *in, struct frame *out, size_t maxlen) 
{
	out->m = in->m; // Copy metadata
	size_t len = in->m.len;
	if (len > maxlen) len = maxlen;
	out->m.len = len;

	uint8_t buffy_in[len], buffy_out[len];
	memcpy(buffy_in, in->data, len);

	for (int i = 0; i < len; ++i) {
		int sym = buffy_in[i];
		if (sym < 1) {
			buffy_out[i] = 0;
		} else {
			buffy_out[i] = 0xff;
		}
	}

	memcpy(out->data, buffy_out, len);
	out->m.len = len;
	return 0;
}



void build_pdu_dpress_sync(uint8_t fn, uint8_t tn, uint8_t frame_countdown, uint8_t *out) 
{
    uint8_t pdu_sync_SCHS[8];		/* 60 bits */
    uint8_t pdu_sync_SCHH[16];	    /* 124 bits */

    struct bitvec bv;

	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHS;
	bv.data_len = sizeof(pdu_sync_SCHS);

    uint8_t dn232_dn233 = DN233 | (DN232 << 2);

	//bitvec_set_uint(&bv, 0, 4);	/* alignment */
	/* According to Table 21.73: SYNC PDU Contents */
	bitvec_set_uint(&bv, 13, 4);	/* System Code  */
	bitvec_set_uint(&bv, 1, 2);	    /* Sync PDU type */
	bitvec_set_uint(&bv, 1, 2);	    /* Communication Type */
    bitvec_set_uint(&bv, 0, 1);	    /* M-DMO flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Reserved */
    bitvec_set_uint(&bv, 0, 1);	    /* Two-frequency repeater flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Repeater operating modes */
    bitvec_set_uint(&bv, 0, 6);	    /* Spacing of uplink */
    bitvec_set_uint(&bv, 0, 1);	    /* Master/slave link flag */
    bitvec_set_uint(&bv, 0, 2);	    /* Channel usage */
    bitvec_set_uint(&bv, 0, 2);	    /* Channel state */
    bitvec_set_uint(&bv, tn, 2);	/* Slot number */
	bitvec_set_uint(&bv, fn, 5);	/* Frame number */
    bitvec_set_uint(&bv, 1, 3);	    /* Power class */
    bitvec_set_uint(&bv, 1, 1);	    /* Power control flag */
    bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
    bitvec_set_uint(&bv, frame_countdown, 2);	/* Frame countdown */
    bitvec_set_uint(&bv, 0, 2);	    /* Reserved / priority */
    bitvec_set_uint(&bv, 0, 6);	    /* Reserved */
    bitvec_set_uint(&bv, dn232_dn233, 4);	    /* Values of DN232 and DN233 */
    bitvec_set_uint(&bv, DT254, 3); /* Value of DT254 */
    bitvec_set_uint(&bv, 0, 1);	    /* Presence signal dual watch sync flag */
    bitvec_set_uint(&bv, 0, 5);	    /* Reserved */

	// printf("DPRES-SYNC SCH/S PDU: %s ", osmo_hexdump(pdu_sync_SCHS, sizeof(pdu_sync_SCHS)));

	memset(&bv, 0, sizeof(bv));
	bv.data = pdu_sync_SCHH;
	bv.data_len = sizeof(pdu_sync_SCHH);

    bitvec_set_uint(&bv, REP_ADDRESS, 10);	/* Repeater address */
    bitvec_set_uint(&bv, REP_MCC, 10);	    /* Repeater MNI-MCC */
    bitvec_set_uint(&bv, REP_MNC, 14);	    /* Repeater MNI-MNC */
	bitvec_set_uint(&bv, 3, 2);	    /* Validity time unit (3=not restricted) */
    bitvec_set_uint(&bv, 0, 6);	    /* Validity time unit count */
	bitvec_set_uint(&bv, 1, 3);	    /* Max DM-MS power class */
	bitvec_set_uint(&bv, 0, 1);	    /* Reserved */
	bitvec_set_uint(&bv, 0, 4);	    /* Usage restriction type (URT) */
    bitvec_set_uint(&bv, 0, 72);	/* URT addressing  */
	bitvec_set_uint(&bv, 0, 2);	    /* Reserved */

	// printf(" SCH/H PDU: %s\n", osmo_hexdump(pdu_sync_SCHH, sizeof(pdu_sync_SCHH)));

	uint8_t sb_type2[80];
	uint8_t sb_master[80*4];
	uint8_t sb_type3[120];
	uint8_t sb_type4[120];
	uint8_t sb_type5[120];

	uint8_t si_type2[140];
	uint8_t si_master[144*4];
	uint8_t si_type3[216];
	uint8_t si_type4[216];
	uint8_t si_type5[216];

    uint8_t burst[255*2];
	uint16_t crc;
	uint8_t *cur;

	memset(sb_type2, 0, sizeof(sb_type2));
	cur = sb_type2;

	/* SYNC SCH/S */
	cur += osmo_pbit2ubit(sb_type2, pdu_sync_SCHS, 60);

	crc = ~crc16_ccitt_bits(sb_type2, 60);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	/* Append 4 tail bits: type-2 bits */
	cur += 4;

	// printf("SYNC type2: %s\n", osmo_ubit_dump(sb_type2, 80));

    
	/* Run rate 2/3 RCPC code: type-3 bits*/
	{
        struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, sb_type2, 80, sb_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb_master, 120, sb_type3);
		free(ces);
	}
	// printf("SYNC type3: %s\n", osmo_ubit_dump(sb_type3, 120));

	/* Run (120,11) block interleaving: type-4 bits */
	block_interleave(120, 11, sb_type3, sb_type4);
	// printf("SYNC type4: %s\n", osmo_ubit_dump(sb_type4, 120));

	/* Run scrambling (all-zero): type-5 bits */
	memcpy(sb_type5, sb_type4, 120);
	tetra_scramb_bits(SCRAMB_INIT, sb_type5, 120);
	// printf("SYNC type5: %s\n", osmo_ubit_dump(sb_type5, 120));


	/* SYNC SCH/H */
	memset(si_type2, 0, sizeof(si_type2));
	cur = si_type2;
	cur += osmo_pbit2ubit(si_type2, pdu_sync_SCHH, 124);

	/* Run it through CRC16-CCITT */
	crc = ~crc16_ccitt_bits(si_type2, 124);
	crc = swap16(crc);
	cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);

	/* Append 4 tail bits: type-2 bits */
	cur += 4;

	// printf("SI type2: %s\n", osmo_ubit_dump(si_type2, 140));

	/* Run rate 2/3 RCPC code: type-3 bits */
	{
		struct conv_enc_state *ces = calloc(1, sizeof(*ces));
		conv_enc_init(ces);
		conv_enc_input(ces, si_type2, 144, si_master);
		get_punctured_rate(TETRA_RCPC_PUNCT_2_3, si_master, 216, si_type3);
		free(ces);
	}

	// printf("SI type3: %s\n", osmo_ubit_dump(si_type3, 216));

	/* Run (216,101) block interleaving: type-4 bits */
	block_interleave(216, 101, si_type3, si_type4);
	// printf("SI type4: %s\n", osmo_ubit_dump(si_type4, 216));

	/* Run scrambling (all-zero): type-5 bits */
	memcpy(si_type5, si_type4, 216);
	tetra_scramb_bits(SCRAMB_INIT, si_type5, 216);
	// printf("SI type5: %s\n", osmo_ubit_dump(si_type5, 216));

    build_dm_sync_burst(burst, sb_type5, si_type5); 
    // printf("DPRES-SYNC burst: %s\n", osmo_ubit_dump(burst, 255*2));
    memcpy(out, burst, 510);

}

// rx thread
void *rep_rx_thread()
{
    struct timeval now;
    // setup ZMQ RX
	void *zmq_context = zmq_ctx_new();
    zmq_rx_socket = zmq_socket(zmq_context, ZMQ_SUB);
    int connret = zmq_connect(zmq_rx_socket, "tcp://localhost:43300");
	zmq_setsockopt(zmq_rx_socket, ZMQ_SUBSCRIBE, "", 0);

	while (1) {
        gettimeofday(&now, NULL);
        uint64_t now_us = (double)now.tv_sec*1.0e6+(double)now.tv_usec;

		int nread;
		zmq_msg_t input_msg;
		zmq_msg_init(&input_msg);
		nread = zmq_msg_recv(&input_msg, zmq_rx_socket, ZMQ_DONTWAIT);
		
		if (nread >= 0) {
			char encoded_buf[sizeof(struct frame) + ENCODED_MAXLEN];
			struct frame *encoded = (struct frame *)encoded_buf;
			int rc = floats_to_bits(zmq_msg_data(&input_msg), encoded, ENCODED_MAXLEN);
            printf("RX burst: %s\n", osmo_ubit_dump(encoded->data, encoded->m.len));

		} else {
            // no burst in buffer
            if (tms->channel_state == DM_CHANNEL_S_MS_IDLE_UNKNOWN) {
                if (now_us > (tms->channel_state_last_chg+2.04e6)) { // 2 multiframes
                    printf("[RX-%ld] DMO channel state change: %d -> %d \n",now_us, tms->channel_state, DM_CHANNEL_S_MS_IDLE_FREE);
                    tms->channel_state = DM_CHANNEL_S_MS_IDLE_FREE;
                    tms->channel_state_last_chg = now_us;
                }
                
            }
        }
		zmq_msg_close(&input_msg);
	}

	zmq_ctx_destroy(zmq_context);



};

// tx thread
void rep_tx_thread() 
{
    unsigned int multiframe_counter = 0;
    uint8_t multiframes_since_last_presence = 0;
    unsigned int pointer_tn = 0;
    unsigned int counter = 0;
   	struct timeval now;

   	void *zmq_context = zmq_ctx_new();
    zmq_tx_socket = zmq_socket(zmq_context, ZMQ_PUB);
    int connret = zmq_connect(zmq_tx_socket, "tcp://localhost:43301");


    gettimeofday(&now, NULL);
    uint64_t now_us = (double)now.tv_sec*1.0e6+(double)now.tv_usec;
    uint64_t one_sec = now_us + 1000000;

    initialize_framebuffer(one_sec);

    struct timeslot current_ts = frame_buf.tn[pointer_tn];
    struct timeslot next_ts = frame_buf.tn[pointer_tn+1];

    while (1) { 
        gettimeofday(&now, NULL);
        now_us = (double)now.tv_sec*1.0e6+(double)now.tv_usec;

        if (now_us > next_ts.start_ts){
            current_ts = next_ts;
            
            if (current_ts.len > 0) {
                printf("[TX-%ld] [%d] %02d/%02d/%d - start ts: %ld, len: %d, burst: %s\n", now_us, tms->channel_state, multiframe_counter,current_ts.fn, current_ts.tn, current_ts.start_ts, current_ts.len, osmo_ubit_dump(current_ts.burst, current_ts.len));
            }
            if (current_ts.len > 0) {
                // send burst to modem
                char encoded_buf[sizeof(struct frame) + ENCODED_MAXLEN];
			    struct frame *encoded = (struct frame *)encoded_buf;

                encoded->m.flags = 0;
                encoded->m.time = 0;
                encoded->m.len = current_ts.len;
                memcpy(encoded->data,current_ts.burst,current_ts.len);

           		zmq_msg_t input_msg;
    		    zmq_msg_init_size(&input_msg, sizeof(struct frame)+encoded->m.len);

                int rc = bits_to_floats(encoded, zmq_msg_data(&input_msg), ENCODED_MAXLEN);

                // rc = zmq_send(zmq_tx_socket, encoded, sizeof(struct frame)+encoded->m.len, ZMQ_DONTWAIT);
                rc = zmq_msg_send(&input_msg, zmq_tx_socket, ZMQ_DONTWAIT);
                // printf("ZMQSEND: %d", rc);

            }
            reprime_timeslot(pointer_tn);


            counter++;
            pointer_tn = counter%72;
            next_ts = frame_buf.tn[pointer_tn];

            // when rolling over to next multiframe
            if (next_ts.fn < current_ts.fn) {
                multiframe_counter++;
                if (tms->channel_state == DM_CHANNEL_S_MS_IDLE_FREE) {
                    if (++multiframes_since_last_presence >= presence_signal_multiframe_count[DT254]) {
                        // printf("presence signal away!\n");
                        multiframes_since_last_presence = 0;

                        for (int i = DN253; i>0; i--) { // repeat for N frames
                            uint8_t frame_num = 18-i;
                            struct timeslot *burst_slot;

                            burst_slot = &frame_buf.tn[4*frame_num];
                            build_pdu_dpress_sync(burst_slot->fn, burst_slot->tn, i-1, burst_slot->burst);
                            burst_slot->len = 510;

                            burst_slot = &frame_buf.tn[4*frame_num+1];
                            build_pdu_dpress_sync(burst_slot->fn, burst_slot->tn, i-1, burst_slot->burst);
                            burst_slot->len = 510;

                            burst_slot = &frame_buf.tn[4*frame_num+2];
                            build_pdu_dpress_sync(burst_slot->fn, burst_slot->tn, i-1, burst_slot->burst);
                            burst_slot->len = 510;

                            burst_slot = &frame_buf.tn[4*frame_num+3];
                            build_pdu_dpress_sync(burst_slot->fn, burst_slot->tn, i-1, burst_slot->burst);
                            burst_slot->len = 510;

                        }
                    }

                }

            }

        }
    }

};


int main(int argc, char **argv)
{
    pthread_t thread_rx_handle;
   	struct timeval now;

    // initializations
	tms = talloc_zero(tetra_tall_ctx, struct tetra_mac_state);
	tetra_mac_state_init(tms);
	tms->infra_mode = TETRA_INFRA_DMO; 

    gettimeofday(&now, NULL);
    uint64_t now_us = (double)now.tv_sec*1.0e6+(double)now.tv_usec;
    tms->channel_state_last_chg=now_us;
    tms->channel_state=DM_CHANNEL_S_MS_IDLE_UNKNOWN;

    printf("## TETRA Type 1A DM-REP ##\n");

    // state machine
    pthread_create(&thread_rx_handle, NULL, rep_rx_thread, NULL);

    rep_tx_thread();

	talloc_free(tms);

    exit (0);
}