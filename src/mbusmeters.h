#include <Arduino.h>
#include "decoder3o6.h"
#include <endian.h> 

#ifndef MBUSMETERS
#define MBUSMETERS

class mbCalDate
{

public:
    uint16_t date; // bits y:7 m:4 d:5

    inline char *getasString(char *s)
    {
        //Serial.println(date,HEX);
        snprintf(s, 9, "%02d.%02d.%02d", (date & 0x1F), ((date >> 5) & 0xF), (date >> 9));
        //s[8] = 0;
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
    // mbCalDate curDate, prevDate;
    uint16_t preVal;
    uint16_t curVal;

    uint8_t msgcnt : 4; //just to douple check
    uint8_t eqcnt : 4;  //if msgcnt is <2 overwrite values ... difficult to decide
};

// we can have 2 types of operation: scan mode or filter mode
// scan mode creates objects from received mBusMsgs
// filter mode compares against prefilled IDs
// but finaly also scan should be able to filter received existing messages

class wMBusMsg
{ // we might add low level like RSSI, location and group information, vendor might be set static
    // for a group of meters we might have one vendor, mtype, version prevDate and cur date, and a specialtpe for HKZ

    //ToDo definition of HKZ (temp1,2) by mtype
    // definition of Watermeters : 0x61 cold water, 0x62 warm water
    // WMZ 0x43
    // we do not keep data, as the intention is anyhow to handover the data
    // this object is focused on techem, it might work for others but not tested
public:
    uint8_t mbmsg[64]; //hold the message
    uint8_t mbmsg_len;
    uint8_t mtype;
    uint16_t vendor;
    mbCalDate prevDate;
    mbCalDate curDate;
    char printstr[10]; //ToDo could be dangerous in multi-threat calls

    //ToDo all the 16 and 32 bit casts should be ntoh wrapped to make it Byteordering portable
    char parseraw(uint8_t *raw, uint8_t len)
    {
        memset(mbmsg, 0, sizeof(mbmsg));
        if (decode3o6Block(raw, mbmsg, len) == DecErr)
            return 0; //error handling
        mbmsg_len = len * 2 / 3;
        vendor = get_vendor();
        get_calDates(); //get both mbcalDates and mtype
        return 1;       //OK
    };
    inline uint32_t get_serial()
    {
        return le32toh(*(uint32_t *)&mbmsg[4]);
    }
    inline uint16_t get_vendor()
    {
        return be16toh(*((uint16_t *)&mbmsg[2]));
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
    inline uint32_t get_prevVal()
    { //this value is the counter from tha last bill period
        if (mtype == 0x43)
            return le32toh(*(uint32_t *)&mbmsg[16]) & 0x00FFFFFF; //only 3 bytes
        if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80))
            return le16toh(*(uint16_t *)&mbmsg[16]);
    return 0;
    }
    inline uint32_t get_curVal()
    { //this is the diff since the last period
        if (mtype == 0x43)
            return le32toh(*(uint32_t *)&mbmsg[20]) & 0x00FFFFFF; //only 3 bytes
        if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80))
            return le16toh(*(uint16_t *)&mbmsg[20]);
        return 0;
    }

    inline float get_temp1()
    {
        uint16_t temp1 = le16toh(*((uint16_t *)&mbmsg[22]));
        return (float)((float)temp1) / 100;
    }
    inline float get_temp2()
    {
        uint16_t temp2 = le16toh(*((uint16_t *)&mbmsg[24]));
        return (float)((float)temp2) / 100;
    }

    inline void get_calDates()
    {
        //first we get the prev Date as it has a year
        prevDate.date = (*(uint16_t *)&mbmsg[14]);

        // now get the current Date, which is much more tricky
        mtype = get_mtype(); //diffrerent types have different message formats
        if (mtype == 0x43)   //WMZ
        {
            curDate.date = (uint16_t)(mbmsg[19] & 0x78) << 2;            //month
            curDate.date |= (((*(uint16_t *)&mbmsg[23]) >> 7) & 0x001F); //day
        }
        else if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80))
        { //WZ HKZ
            curDate.date = ((*(uint16_t *)&mbmsg[18]) >> 4) & 0x1FF;
        };

        curDate.date |= prevDate.date & 0xFE00; //we add the year from prev Date
        if (prevDate.date > curDate.date)
        {                          //prevdate is always smaler than current date
            curDate.date += 0x200; // we add 1 year, bit arethmetic
        };
        //ToDo the year is a tricky one. we need to derive it from the prev
    }

    void printmsg()
    {
        //unsigned char RSSI = rfm69.getLastRSSI();
//#define debug_decoder
#ifdef debug_decoder
        Serial.print("mbmsg: ");
        //for (i = 0; i < mBusMsg[0] + 1; i++)
        for (uint8_t i = 0; i < mbmsg_len; i++)
        {
            char tempstr[3];
            sprintf(tempstr, "%02X", mbmsg[i]);
            Serial.print(tempstr);
        }
        Serial.print(":");
        Serial.println((mbmsg_len), HEX);

        // Serial.print(":");
        // Serial.println(rx_rssi / -2.0);
#endif

        //Serial.print("msgdec: ");
        // Serial.print(rx_rssi / -2.0);
        //Serial.print(":");
        char s2[30];
        snprintf(s2, sizeof(s2), "msgdec: %04X;%08X;%02X", vendor, get_serial(), mtype);
        Serial.print(s2);

        if (vendor == 0x6850)
        { //techem
            Serial.print(";");
            Serial.print(prevDate.getasString(printstr));
            Serial.print(";");
            Serial.print(curDate.getasString(printstr));

            if (mtype == 0x80)
            {
                snprintf(s2, sizeof(s2), ";%.2f;%.2f", get_temp1(), get_temp2());
                Serial.print(s2);
            }

            if ((mtype == 0x62) || (mtype == 0x61) || (mtype == 0x80) || (mtype == 0x43))
            {
                Serial.print(";");
                Serial.print(get_prevVal());
                Serial.print(";");
                Serial.print(get_curVal());
            }
            Serial.println();
        };
        //else Serial.print(rx_rssi / -2.0);
    };
};

class mbMeterGrp
{
public:
    uint16_t vendor;
    uint8_t mtype;
    uint8_t lstlen;
    uint8_t nrcap; // number of captured meters

    //??const uint8_t version = 0;

    // we assume that all members have the same calendar dates
    // there might be short intervals where there are not in time sync
    // ToDo how to determine, which is the correct date
    // curDate might be used to determine the current date, with some variation at midnight
    mbCalDate prevDate;
    mbCalDate curDate;

    uint32_t *serlst; //list of serial numbers of interesst, will be set in constructor

    //mbmtrVals_t mbmvals[20];
    mbmtrVals_t *mtrvals; // array of meters values

    inline mbMeterGrp(uint16_t vndr, uint8_t mt, uint32_t *lst, uint8_t len)
    {
        vendor = vndr;
        mtype = mt;
        serlst = lst; //take over list
        lstlen = len;
        // alloc the space for the values 1 set per meter
        mtrvals = (mbmtrVals_t *)calloc(lstlen, sizeof(mbmtrVals_t));
    }

    inline void clearvals()
    { // clears all values, hard core
        memset(mtrvals, 0, lstlen * sizeof(mbmtrVals_t));
        nrcap =0;
    }

    inline char complete()
    { // all meters received at least once
        return (lstlen == nrcap);
    }

    inline char matchgrp(wMBusMsg msg)
    { //cehcks if msg fits to group, returns result 1 = yes

        if ((vendor != msg.vendor) || (mtype != msg.mtype))
            return 0;
        //after first check seach now for the serail
        //Serial.println("match vendor and mtype");
    
        for (uint8_t idx = 0; idx < lstlen; idx++)
        {
            //Serial.println(serlst[idx],HEX);
            if (serlst[idx] == msg.get_serial())
            // match
            {
                //update values for mtrvals[idx]
#ifdef debug
                Serial.print("match:");
                Serial.println(serlst[idx], HEX);
#endif //debug

                //ToDo type dependant handling
                mtrvals[idx].curVal = msg.get_curVal();
                mtrvals[idx].preVal = msg.get_prevVal();
                if (mtrvals[idx].msgcnt == 0)
                {
                    nrcap++;
                    mtrvals[idx].msgcnt = 1;
                };
                //ToDo we should improve that, might be to double check
                //mtrvals[idx].curDate.date = msg.curDate.date;
                //mtrvals[idx].prevDate.date = msg.prevDate.date;
                return 1;
            }
        }
        return 0;
    } // func matchgrp

    inline void fillsendstr(const char * nid, char *s, uint8_t slen)
    { //fill all values into a given string
        uint8_t written = 0;
        written = snprintf(s, slen, "nodeid=%s&values=", nid);
        for (uint8_t i = 0; (i < lstlen) && (written < slen); i++) {
            written += snprintf(s + written, slen - written, (i != 0 ? ";%u" : "%u"), mtrvals[i].curVal);
        };
    };
};

struct mbusHKZ_t
{
    const uint32_t sernum;
    uint16_t prevVal;
    uint16_t curVal;
    float temp1, temp2;
};

#endif