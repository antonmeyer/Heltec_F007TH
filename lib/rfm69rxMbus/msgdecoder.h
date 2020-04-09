//just some encapsulations for value extracting
//be careful: Byte ordering ntoh function include did not work

#include "decoder3o6.h"

static inline uint32_t get_serial(const uint8_t *const packet)
{
    uint32_t serial;
    memcpy(&serial, &packet[4], sizeof(serial));
    //return __builtin_bswap32(serial);
    return serial;
}
static inline uint16_t get_vendor(const uint8_t *const packet)
{
    uint16_t vendor;
    memcpy(&vendor, &packet[2], sizeof(vendor));
    return __builtin_bswap16(vendor);
    //return vendor;
}
static inline uint16_t get_type(const uint8_t *const packet)
{
    uint16_t type;
    type = 0;
    memcpy(&type, &packet[8], sizeof(type));
    return type;
}
static inline uint32_t get_last(const uint8_t *const packet)
{ //this value is the counter from tha last period
    uint32_t last;
    last = 0;
    memcpy(&last, &packet[16], 3);
    return last;
}
static inline uint32_t get_current(const uint8_t *const packet)
{ //this is the diff since the last period
    uint32_t curent;
    curent = 0;
    memcpy(&curent, &packet[20], 3);
    return curent;
}
static inline float get_temp1(const uint8_t *const packet)
{
    uint16_t temp1;
    memcpy(&temp1, &packet[22], sizeof(temp1));
    return (float)((float)temp1) / 100;
}
static inline float get_temp2(const uint8_t *const packet)
{
    uint16_t temp2;
    memcpy(&temp2, &packet[24], sizeof(temp2));
    return (float)((float)temp2) / 100;
}
static inline uint16_t get_actHKZ(const uint8_t *const packet)
{
    uint16_t actHKZ;
    memcpy(&actHKZ, &packet[20], sizeof(actHKZ));
    return actHKZ;
}
static inline uint16_t get_prevHKZ(const uint8_t *const packet)
{
    uint16_t prevHKZ;
    memcpy(&prevHKZ, &packet[16], sizeof(prevHKZ));
    return prevHKZ;
}

unsigned char mBusMsg[64];

void printmsg(unsigned char RxBuffer[], unsigned char RxBufferlen, byte rx_rssi)
{
    //unsigned char RSSI = rfm69.getLastRSSI();
    {
        memset(mBusMsg, 0, sizeof(mBusMsg));
        if (decode3o6Block(RxBuffer, mBusMsg, RxBufferlen) != DecErr)
        {

            Serial.print("mbmsg: ");
            //for (i = 0; i < mBusMsg[0] + 1; i++)
            for (uint8_t i = 0; i < RxBufferlen * 2 / 3; i++)
            {
                char tempstr[3];
                sprintf(tempstr, "%02X", mBusMsg[i]);
                Serial.print(tempstr);
            }
            Serial.print(":");
            Serial.print((RxBufferlen * 2 / 3), HEX);

            Serial.print(":");
            Serial.println(rx_rssi / -2.0);

            uint16_t mtype = get_type(mBusMsg);

            Serial.print("msgdec: ");
            Serial.print(get_vendor(mBusMsg), HEX);
            Serial.print(";");
            Serial.print(get_serial(mBusMsg), HEX);
            Serial.print(";");
            Serial.print(mtype, HEX);
            Serial.print(";");

            if (mtype == 0x8069)
            {
                Serial.print(get_temp1(mBusMsg));
                Serial.print(";");
                Serial.print(get_temp2(mBusMsg));
                Serial.print(";");
                Serial.print(get_actHKZ(mBusMsg));
                Serial.print(";");
                Serial.println(get_prevHKZ(mBusMsg));
            }
            else if ((mtype & 0xFF00) == 0x4300)
            {
                Serial.print(get_last(mBusMsg));
                Serial.print(";");
                Serial.println(get_current(mBusMsg));
            }
            else
            {
                Serial.println();
            };
        }
    }
}
