#include <Arduino.h>
#include "decoder3o6.h"

#ifndef MBUSMETERS
#define MBUSMETERS

class mbCalDate
{

public:
    uint16_t date; // bits y:7 m:4 d:5

    inline char * getasString(char * s)
    {
        //Serial.println(date,HEX);
        sprintf(s, "%02d.%02d.%02d", (date & 0x1F), ((date >> 5) & 0xF), (date >> 9));
        s[8] = 0;
        return s;
    };

    inline uint8_t getyear()
    {
        return (uint8_t)(date >> 9);
    };

    inline void setyear(uint8_t y)
    {
        date = (date & 0x7F) || (uint16_t)((uint16_t)y << 9);
    };
};

struct mbmtrVals_t
{
    uint16_t prevVal;
    uint16_t curVal;

    uint8_t msgcnt : 4; //just to douple check
    uint8_t eqcnt : 4;  //if msgcnt is <2 overwrite values ... difficult to decide
};

class mbMeterGrp
{
    const uint16_t vendor = 0;
    const uint8_t mtype = 0;
    const uint8_t version = 0;

    const mbCalDate prevDate;
    const mbCalDate curDate;

    uint32_t serlst[20];
    mbmtrVals_t mbmvals[20];
};

// we can have 2 types of operation: scan mode or filter mode
// scan mode creates objects from received mBusMsgs
// filter mode compares against prefilled IDs
// but finaly also scan filters existing meters

class wMBusMsg
{ // we might add low level like RSSI, location and group information, vendor might be set static
    // for a group of meters we might have one vendor, mtype, version prevDate and cur date, and a specialtpe for HKZ

    //ToDo definition of HKZ (temp1,2) by mtype
    // definition of Watermeters : 0x61 cold water, 0x62 warm water
    // WMZ 0x43
    // we do not keep data, as the intention is anyhow to handover the data
    // or should we add here the 3o6 decoder
    // and the print msg toString
public:

    uint8_t mbmsg[64]; //hold the message   
    uint8_t mtype;
    uint16_t vendor;
    mbCalDate prevDate;
    mbCalDate curDate;

    char printstr[10]; //ToDo could be dangerous in multi-threat calls

    void parseraw(uint8_t *raw, uint8_t len)
    {
        memset(mbmsg, 0, sizeof(mbmsg));
        decode3o6Block(raw, mbmsg, len); //error handling
        vendor = get_vendor();
        mtype = get_mtype();

        prevDate.date = get_prevDate();
        curDate.date = get_curDate();
    };
     inline uint32_t get_serial()
    {
        return *((uint32_t *)&mbmsg[4]);
    }
     inline uint16_t get_vendor()
    {
        return __builtin_bswap16(*((uint16_t *)&mbmsg[2]));
        //return __builtin_bswap16(vendor);
        //return vendor;
    }
     inline uint8_t get_mtype()
    {
        return (uint8_t)mbmsg[9];
    }
     inline uint8_t get_version()
    {
        return (uint8_t)mbmsg[8];
    }
     inline uint32_t get_prevWMZ()
    {                                                  //this value is the counter from tha last period
        return (*(uint32_t *)&mbmsg[16]) & 0x00FFFFFF; //only 3 bytes
    }
     inline uint32_t get_curWMZ()
    {                                                  //this is the diff since the last period
        return (*(uint32_t *)&mbmsg[20]) & 0x00FFFFFF; //only 3 bytes
    }
     inline float get_temp1()
    {
        uint16_t temp1 = *((uint16_t *)&mbmsg[22]);
        return (float)((float)temp1) / 100;
    }
     inline float get_temp2()
    {
        uint16_t temp2 = *((uint16_t *)&mbmsg[24]);
        return (float)((float)temp2) / 100;
    }

     inline uint16_t get_prevWZ()
    {
        return *(uint16_t *)&mbmsg[16];
    }

     inline uint16_t get_curWZ()
    {
        return *(uint16_t *)&mbmsg[20];
    }

     inline uint16_t get_prevDate()
    {
        return *(uint16_t *)&mbmsg[14];
    }

     inline uint16_t get_curDate()
    {
        uint16_t tmp;
        uint8_t mtype = get_mtype();
        if (mtype == 0x43)
        {                                                         //WMZ
            tmp = (uint16_t)(mbmsg[19] & 0x78) << 2;              //month         
            tmp |= (((*(uint16_t *)&mbmsg[23]) >> 7) & 0x001F); //day
            return tmp;
        }
        else if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80))
        { //WZ HKZ
            return ((*(uint16_t *)&mbmsg[18]) >> 4) & 0x1FF;
        };
        return 0;
        //ToDo the year is a tricky one. we need to derive it from the prev
    }

    void printmsg()
    {
        //unsigned char RSSI = rfm69.getLastRSSI();
        {
//#define debug_decoder
#ifdef debug_decoder
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
#endif

            Serial.print("msgdec: ");
            // Serial.print(rx_rssi / -2.0);
            //Serial.print(":");
            Serial.print(vendor, HEX);
            Serial.print(";");
            Serial.print(get_serial(), HEX);
            Serial.print(";");
            Serial.print(mtype, HEX);
            Serial.print(";");

            if (vendor == 0x6850)
            { //techem
                Serial.print(prevDate.getasString(printstr));
                Serial.print(";");
                Serial.print(curDate.getasString(printstr));
                Serial.print(";");
            }

            if (mtype == 0x80)
            {
                Serial.print(get_temp1());
                Serial.print(";");
                Serial.print(get_temp2());
                Serial.print(";");
            }
            if (mtype == 0x43)
            {
                Serial.print(get_prevWMZ());
                Serial.print(";");
                Serial.println(get_curWMZ());
            }
            else if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80))
            {
                Serial.print(get_prevWZ());
                Serial.print(";");
                Serial.println(get_curWZ());
            }

            else
            {
                Serial.println();
            };
            //else Serial.print(rx_rssi / -2.0);
        };
    };
};

struct metergroup_t
{
    const uint16_t vendor;
    const uint8_t mtype;
    const uint8_t version;

    const mbCalDate prevDate;
    const mbCalDate curDate;

    //mbusMeter_t meters[20];
};

struct mbusHKZ_t
{
    const uint32_t sernum;
    uint16_t prevVal;
    uint16_t curVal;
    float temp1, temp2;
};

struct mbusHKZgroup_t
{
    const uint16_t vendor;
    const uint8_t mtype;
    const uint8_t version;

    const mbCalDate prevDate;
    const mbCalDate curDate;

    mbusHKZ_t HKZmeters[20];
};

#endif