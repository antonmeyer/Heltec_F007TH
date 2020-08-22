#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "Arduino.h"

#include "esp32-hal.h"
#include <functional>

typedef struct
{ // a half rmt item
    union {
        struct
        {
            uint16_t dur : 15;
            uint16_t lvl : 1;
        };
        uint16_t val;
    };
} rmt_pulse_t;

static void wrap_rxhandler(uint32_t *data, size_t len); //forward declaration

class F007TH_RMT
{
public:
    F007TH_RMT(uint8_t RFpin)
    {
        rmt_obj_t *rmt_recv = NULL;

        if ((rmt_recv = rmtInit(RFpin, false, RMT_MEM_192)) == NULL)
        {
            Serial.println("init receiver failed\n");
        }
        float realTick = rmtSetTick(rmt_recv, 1000);

        Serial.printf("real tick set to: %fns\n", realTick);

        rmtSetFilter(rmt_recv, true, 255);
        rmtSetRxThreshold(rmt_recv, 2000);

        // Ask to start reading
        rmtRead(rmt_recv, wrap_rxhandler);
    }; //constructor
    static void rxhandler(uint32_t *data, size_t len);

    static inline bool in_range(int duration_ticks, int target_us, int margin_us)
    {
        return (duration_ticks < (target_us + margin_us)) && (duration_ticks > (target_us - margin_us));
    }

    static inline bool is_short(int dur)
    {
        return in_range(dur, 500, 150);
    }

    static inline bool is_long(int dur)
    {
        return in_range(dur, 1000, 150);
    }

    static void addBit(char bit, uint64_t bits)
    {
        //Serial.print(" ");
        //Serial.print(bit,BIN);
        bits <<= 1; // 1 shift left
        bits |= bit;
        return; //we should later return if we are ready
    }

    static void decodemsg(uint64_t bits)
    {
        uint8_t *ba = (uint8_t *)&bits; //pointer to 8x byte array
        /* for (int i = 0; i<8; i++) {
        Serial.printf("%2X",ba[i]);
        } 
        Serial.println(); */

        if (ba[5] == 0x45)
        { // Ambient msg
            byte stnId = (ba[3] & B01110000) >> 4;
            int chTemp = ((ba[3] & B00000111) << 8) + ba[2];
            int chHum = ba[1];

            Serial.printf("%d:%.2f:%d\n", stnId + 1, 0.0556 * (chTemp - 720), chHum);
        }
    }
}; //Class F007TH_RMT

void F007TH_RMT::rxhandler(uint32_t *data, size_t len)
{

    //big questionmark, data could be fragments of a message
    if (len < 25)
        return;
    rmt_pulse_t *pulses = NULL;
    uint64_t bits;  //= 0;  //hold the decoded bits
    uint8_t bitcnt; // = 0; // counts the decoded bits
    size_t ok = 0;
    char state = 0;
    char sync;
    int8_t oBit;
    pulses = (rmt_pulse_t *)data;

    for (size_t i = 0; i < len * 2; i++)
    {
        switch (state)
        {
        case 0:
        { //reset
            //Serial.print("r");
            bits = 0;
            bitcnt = 0;
            oBit = -1;
            state = 1;
            sync = 0;
        }
        case 1:
        { //seraching for the 1st Zero after sync bits
            /*
        if ( is_short (pulses[i].dur) || is_long (pulses[i].dur)) {
            //Serial.printf("%3d:%4d:%d\n",i,pulses[i].dur,pulses[i].lvl);
            ok++;
        };
        */
            if (is_short(pulses[i].dur))
                sync++;
            if (is_long(pulses[i].dur))
            {
                ok++;
                if (sync > 10)
                {
                    //Serial.printf("i: %d; sync: %d;lvl: %d\n", i, sync, pulses[i].lvl);
                    state = 2;
                    oBit = pulses[i].lvl; //should be a 0 in normal conditions
                }
                ok += sync;
                sync = 0;
            }
            break;
        } //case state 1

        case 2: // we are in sync the idea is 1T is 1 digit, 10 -> 1, 01->0, 11,00 is an error
        {
            if (is_short(pulses[i].dur))
            { //1T
                if (oBit > -1)
                { // 1+1 T
                    addBit(oBit, bits);
                    bitcnt++;
                    oBit = -1; //no remaining T
                }
                else
                    oBit = pulses[i].lvl; // we did not had a remaining T, now we have 1
                ok++;
            }
            else if (is_long(pulses[i].dur))
            { // 2T
                if (oBit > -1)
                { //1+2 T
                    addBit(oBit, bits);
                    bitcnt++;
                    oBit = pulses[i].lvl;
                }
                else
                    state = 0; // manchester violation
                ok++;
            }
            else
                state = 0; // pulse lenght does not fit

            if (bitcnt > 49) //special for F007TH
            {
                state = 0;
                decodemsg(bits);
            }
        }  // case 2
        }; //case switch
    };     //for loop
    Serial.printf("len: %4d; ok: %4d\n", len * 2, ok);
}

static inline void wrap_rxhandler(uint32_t *data, size_t len)
{ //this wrapper is needed for the c callback
    F007TH_RMT::rxhandler(data, len);
};
