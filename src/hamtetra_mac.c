/*  Contains functionality to implement the business logic of DM MAC layer 
 *
 */

#include <osmocom/core/talloc.h>

#include "hamtetra_config.h"
#include "hamtetra_mac.h"
#include "hamtetra_pdu_generator.h"

void *tetra_tall_ctx;

struct tetra_mac_state *tms;
struct tetra_tdma_time time;


// timeslot struct
struct timeslot {
    uint8_t fn;             // frame 1-18
    uint8_t tn;             // timeslot 1-4
    uint8_t slave_fn;       // slave link frame nr
    uint8_t slave_tn;       // slave link timeslot nr
    int len;                // length of burst
    uint8_t delayed;        // send n. multiframes later
    uint8_t burst[512];     // burst bit-per-byte encoded 
};

// multiframe buffer, 18 frames with 4 timeslots = 72 timeslots
struct multiframe {
    struct timeslot tn[72]; 
};

// global frame buffer to both threads
struct multiframe frame_buf_master;
struct multiframe frame_buf_sent;

// initialize the global framebuffer with timeslot numbers and timestamps
void initialize_framebuffer(uint64_t base_time_ts)
{
    for (int i = 0; i<72; i++) {
        struct timeslot *tn = &frame_buf_master.tn[i];
        tn->tn = (i%4) + 1;
        tn->fn = floor( (float)i/4 ) + 1;
        tn->slave_tn = ((i-3)%4) + 1;
        tn->slave_fn = floor( (float)(i-3)/4 ) + 1;
        tn->len = 0;
        tn->delayed = 0;
    }


    for (int i = 0; i<72; i++) {
        struct timeslot *tn = &frame_buf_sent.tn[i];
        tn->tn = (i%4) + 1;
        tn->fn = floor( (float)i/4 ) + 1;
        tn->slave_tn = ((i-3)%4) + 1;
        tn->slave_fn = floor( (float)(i-3)/4 ) + 1;
        tn->len = -1;
        tn->delayed = 0;
    }

};

// set next start timestamp of a timeslot for next round
void reprime_timeslot(uint8_t slotnum) {
    struct timeslot *tn = &frame_buf_master.tn[slotnum];
    if (tn->delayed>0) {
        tn->delayed--;
    } else {
        tn->len = 0;
    }

};

void sent_buffer_set(struct timing_slot *slot, int len) {
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    frame_buf_sent.tn[slotnum].len = len;
}

int sent_buffer_get(struct timing_slot *slot) {
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    return frame_buf_sent.tn[slotnum].len;
}


// lower mac wants to send a burst
void dp_sap_udata_req(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, struct tetra_tdma_time tdma_time, struct tetra_mac_state *tms_req)
{
    uint8_t slotnum = (4*(tdma_time.fn-1))+tdma_time.tn;
    if (tdma_time.link == DM_LINK_SLAVE) {
        slotnum = (slotnum+3) % 72;
    }
    printf("BURST OUT - scheduled to slave link buffer slot %d/%d (%d)\n", tdma_time.fn, tdma_time.tn, slotnum);
    struct timeslot *burst_slot;
    burst_slot = &frame_buf_master.tn[slotnum];
    memcpy(burst_slot->burst, bits, len);
    burst_slot->len = len;

    if (tms_req->channel_state != tms->channel_state) {
        tms->channel_state = tms_req->channel_state;
        tms->channel_state_last_chg = 0;
    }

}


unsigned int multiframe_counter = 0;
uint8_t multiframes_since_last_presence = 0;
int presence_signal_counter = 0;

void mac_hamtetra_init()
{
    // initialize MAC state structure
	tms = talloc_zero(tetra_tall_ctx, struct tetra_mac_state);
	tetra_mac_state_init(tms);
    tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_UNKNOWN;

    // initialiaze send buffer
    initialize_framebuffer(0);

    presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;

}

int mac_request_tx_buffer_content(uint8_t *bits, struct timing_slot *slot)
{
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    int len = -1;

    tms->channel_state_last_chg += 1; // increment last change counter

    // channel house keeping
    switch (tms->channel_state) {
        case DM_CHANNEL_S_DMREP_IDLE_UNKNOWN:
            printf("[DM_CHANNEL_S_DMREP_IDLE_UNKNOWN] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // change channel state to FREE if it has been idle for two multiframes since become UNKNOWN
            if (tms->channel_state_last_chg > 143) {
                tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_FREE;
                tms->channel_state_last_chg = 0;
                presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;
            }
            break;

        case DM_CHANNEL_S_DMREP_IDLE_FREE:
            printf("[DM_CHANNEL_S_DMREP_IDLE_FREE] last chg: %ld - TX slot: %2u %2u %2u ", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // send DPRES-SYNC presence signal burst periodically on IDLE channel
            if (--presence_signal_counter < (DN253*4)){
                uint8_t countdown = (presence_signal_counter-1) / 4;
                len = build_pdu_dpress_sync(slot->fn, slot->tn, countdown, bits);
                if (presence_signal_counter == 0) {
                    presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;
                }
                sent_buffer_set(slot, len);
                printf(" - burst len: %d\n", len);
                return len;
            }
            break;

        case DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED:
            printf("[DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);
            break;

        case DM_CHANNEL_S_DMREP_ACTIVE_RESERVED:
            printf("[DM_CHANNEL_S_DMREP_ACTIVE_RESERVED] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);
            break;


        default:
            printf("[STATE %d] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state, tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);
            break;

    }

    // buffer contains something to send
    if (frame_buf_master.tn[slotnum].len > 0){
        if (frame_buf_master.tn[slotnum].delayed > 0) {
            frame_buf_master.tn[slotnum].delayed--;
        } else {
            struct timeslot *burst_slot;
            burst_slot = &frame_buf_master.tn[slotnum];
            memcpy(bits, burst_slot->burst, burst_slot->len);
            len = burst_slot->len;
            burst_slot->len = 0; // clear the sent
        }
    } 
    
    sent_buffer_set(slot,len);

    printf(" - burst len: %d\n", len);
    return len;
}

/* intermediate function between PHY and lower mac DP-SAP unitdata indicate function to maintain state of the channel and filter echos */
void mac_dp_sap_udata_ind_filter(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv, struct timing_slot *slot)
{
    
    int flen = sent_buffer_get(slot);
    // if everything good, pass parameter through
    if (flen < 0 || slot->changed==1) {
        // printf("mac_dp_sap_udata_ind_filter says hello (%d/%d:%d)\n", slot->fn, slot->tn, flen);
        dp_sap_udata_ind(type, bits, len, priv, slot);
    } else {
        printf("[MAC FILTER] burst filtered (%d/%d:%d)\n", slot->fn, slot->tn, flen);

    }

}
