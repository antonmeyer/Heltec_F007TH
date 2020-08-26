#include "Arduino.h"

#include "esp32-hal-rmt.h"
#include "soc/rmt_struct.h"

#include <driver/rmt.h>
#include "esp_log.h"
#include "absHum.h"

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

typedef struct
{
    uint8_t hum;
    float temp;
} stationvals_t;

class F007TH_RMT
{
    uint8_t RFpin;
    uint8_t maxSens; // number of Sensors ; absolute upper limit is 8
    char state;
    uint8_t bitcnt; // = 0; // counts the decoded bits
    uint64_t bits;  //hold the decoded bits
    int8_t oBit;
    char sync;

public:
    stationvals_t allvals[8]; //hold all values from 8 Stations

    RingbufHandle_t rb;
    rmt_config_t rmt_rx;

    F007TH_RMT(uint8_t pin, uint8_t maxSensors = 8)
    {
        //handover Rx Pin; call startRX afterwards
        RFpin = pin;
        memset(allvals, 0, sizeof(allvals)); // set all to 0
        maxSens = maxSensors;
        initRx();
    }

    void initRx()
    {
        rmt_rx.channel = RMT_CHANNEL_0;
        rmt_rx.gpio_num = (gpio_num_t)RFpin;
        rmt_rx.clk_div = 80;
        rmt_rx.mem_block_num = 8;
        rmt_rx.rmt_mode = RMT_MODE_RX;

        rmt_rx.rx_config.filter_en = true;
        rmt_rx.rx_config.filter_ticks_thresh = 255;
        rmt_rx.rx_config.idle_threshold = 2000;

        rmt_config(&rmt_rx);
        rmt_set_rx_filter(rmt_rx.channel, 1, 255); // just to double check
        rmt_driver_install(rmt_rx.channel, 2048, 0);

        rmt_get_ringbuf_handle(rmt_rx.channel, &rb);
        state = 0;

        //rmt_rx_start(rmt_rx.channel, 1);
    }

    char startRx()
    {
        return rmt_rx_start(rmt_rx.channel, 1);
    };
    char stopRx()
    {
        return rmt_rx_stop(rmt_rx.channel);
    }

    char rxHandler2()
    {
        size_t rx_size;
        char result = 0;
        unsigned int uxwait, uxfree, uxread, uxwrite;
        vRingbufferGetInfo(rb, &uxfree, &uxread, &uxwrite, &uxwait); 
        //if (uxwait < 100) return 1;
        //Serial.println(rmt_get_mem_len(rmt_rx.channel ));
        //xRingbufferPrintInfo(rb);
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 200);
        //xRingbufferPrintInfo(rb);
        if (item != NULL)
        {
            if (rx_size > 100)
            { //Serial.println(xRingbufferGetCurFreeSize(rb));
                Serial.printf("free:%d, read:%d, write:%d, wait:%d\n", uxfree, uxread, uxwrite, uxwait);
                rxhandler(item, rx_size);
                result = 1;
            }
            vRingbufferReturnItem(rb, (void *)item);
        }
        return result;
    }
    inline char ChkSumOk(uint8_t *buff);
    void rxhandler(rmt_item32_t *data, size_t len);

    inline bool in_range(int duration_ticks, int target_us, int margin_us)
    {
        return (duration_ticks < (target_us + margin_us)) && (duration_ticks > (target_us - margin_us));
    }

    inline bool is_short(int dur)
    {
        return in_range(dur, 500, 250);
    }

    inline bool is_long(int dur)
    {
        return in_range(dur, 1000, 250);
    }

    void addBit(char bit)
    {
        //Serial.print(" ");
        //Serial.print(bit,BIN);
        bits <<= 1; // 1 shift left
        bits |= bit;
        return; //we should later return if we are ready
    }

    void decodemsg()
    {
        uint8_t *ba = (uint8_t *)&bits; //pointer to 8x byte array
        /* for (int i = 0; i<8; i++) {
        Serial.printf("%2X",ba[i]);
        } 
        Serial.println(); */

        //Serial.printf("%X:%X:", chksum, ba[0]);
        if (ba[5] == 0x45)
        { // Ambient msg
            byte stnId = (ba[3] & B01110000) >> 4;
            float chTemp = 0.0556 * (((ba[3] & B00000111) << 8) + ba[2] - 720);
            int chHum = ba[1];

            //Serial.printf("%d:%.2f:%d:%s\n", stnId + 1, chTemp, chHum, (ChkSumOk(ba) ? "OK" : "nOK"));

            if (ChkSumOk(ba))
            {
                allvals[stnId].hum = chHum;
                allvals[stnId].temp = chTemp;
                Serial.printf("%d:%.2f:%d:%.2f\n", stnId + 1, chTemp, chHum, absHum(chTemp, chHum));
            }
        }
    }

    void fillsendstr(const char *nid, char *s, uint8_t slen)
    { //fill all values into a given string
        uint8_t written = 0;
        written = snprintf(s, slen, "nodeid=%s&values=", nid);
        for (uint8_t i = 0; (i < maxSens) && (written < slen); i++)
        {
            written += snprintf(s + written, slen - written, (i != 0 ? ";%.2f;%u" : "%.2f;%u"), allvals[i].temp, allvals[i].hum);
        };
    };
}; //Class F007TH_RMT

void F007TH_RMT::rxhandler(rmt_item32_t *data, size_t len)
{
    //big questionmark, data could be fragments of a message
    // we should assume that a message might be distributed over to rxhandler callbacks
    //therefor we keep state, bits, bitcnt and sync global in object
    unsigned long start2 = micros();
    rmt_pulse_t *pulses = (rmt_pulse_t *)data;

    //just for statistics
    size_t ok = 0;
    size_t resets = 0;

    for (size_t i = 0; i < len / sizeof(rmt_pulse_t); i++)
    {
        switch (state)
        {
        case 0:
        { //reset
            //Serial.print("r");
            resets++;
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
            {
                sync++;
                ok++;
            }
            else if (is_long(pulses[i].dur))
            {
                ok++;
                if (sync > 10)
                { // if not enough sync bits it is unlikly to have a valid message
                    //Serial.printf("i: %d; sync: %d;lvl: %d\n", i, sync, pulses[i].lvl);
                    state = 2;
                    oBit = pulses[i].lvl; //should be a 0 in normal conditions
                }
                sync = 0;
            }
            else
                state = 0; // not a valid pulse; a cheap reset could be sync = 0
            break;
        } //case state 1

        case 2: // we are in sync the idea is 1T is 1 digit, 10 -> 1, 01->0, 11,00 is an error
        {
            if (is_short(pulses[i].dur))
            { //1T
                if (oBit > -1)
                { // 1+1 T
                    addBit(oBit);
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
                    addBit(oBit);
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
                decodemsg();
            }
        }  // case 2
        }; //case switch
    };     //for loop
    Serial.printf("len: %4d; ok: %4d resets: %d time: %ld state: %d sync: %d\n", len / sizeof(rmt_pulse_t), ok, resets, (micros() - start2), state, sync);
}

inline char F007TH_RMT::ChkSumOk(uint8_t *buff)
{
    uint8_t mask = 0x7C;
    uint8_t chksum = 0x64;
    uint8_t data;

    for (uint8_t byteCnt = 5; byteCnt > 0; byteCnt--)
    {
        int bitCnt;
        data = buff[byteCnt];

        for (bitCnt = 7; bitCnt >= 0; bitCnt--)
        {
            uint8_t bit;

            // Rotate mask right
            bit = mask & 1;
            mask = (mask >> 1) | (mask << 7);
            if (bit)
            {
                mask ^= 0x18;
            }

            // XOR mask into chksum if data bit is 1
            if (data & 0x80)
            {
                chksum ^= mask;
            }
            data <<= 1;
        }
    }
    return chksum == buff[0];
}
