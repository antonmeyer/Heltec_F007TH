#include <Arduino.h>
#ifndef MBUSMETERS
#define MBUSMETERS

struct mbusCalDate
{
    // Todo clarify bitorder for a < > compare as 16 bit
    uint8_t day : 5;   // 1 ..31
    uint8_t month : 4; //1 ..12
    uint8_t year : 7;  // the remaining of 16 bit based on 2000
};

char mbcaldatestring[10];
char *mbusCalDate_getString(mbusCalDate mcd)
{

    scanf(mbcaldatestring, "%2d.%2d.%2d", mcd.day, mcd.month, mcd.year);
    mbcaldatestring[8] = 0; //zero terminated
    return mbcaldatestring;
}

struct mbusmeter
{   // we might add low level like RSSI, location and group information, vendor might be set static
    // for a group of meters we might have one vendor, mtype, version prevDate and cur date, and a specialtpe for HKZ
    uint32_t sernum;
    uint16_t vendor;
    uint8_t mtype;
    uint8_t version;

    mbusCalDate prevDate;
    mbusCalDate curDate;

    uint16_t prevVal;
    uint16_t curVal;

    float temp1, temp2;
};

//mbusmeter mbusmeters[10];

struct mbusMeter_t
{
    const uint32_t sernum;
    uint16_t prevVal;
    uint16_t curVal;
};

struct metergroup_t
{
    const uint16_t vendor;
    const uint8_t mtype;
    const uint8_t version;

    const mbusCalDate prevDate;
    const mbusCalDate curDate;

    mbusMeter_t meters[20];
};

struct mbusHKZ_t {
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

    const mbusCalDate prevDate;
    const mbusCalDate curDate;

    mbusHKZ_t HKZmeters[20];
};


//ToDo should include all the handling of the meters data
//decoupled by the RF receiver message (frame, packet)

#endif