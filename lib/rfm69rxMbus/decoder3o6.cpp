#include "decoder3o6.h"
#include "Arduino.h"

// #define debug_decoder 
unsigned char decode3o6(unsigned char *encodedData, unsigned char *decodedData, unsigned char lastByte)
{
    unsigned char data[4];

    // - Perform decoding on the input data -
    if (!lastByte)
    {
        data[0] = decodeTab[(*(encodedData + 2) & 0x3F)];
        data[1] = decodeTab[((*(encodedData + 2) & 0xC0) >> 6) | ((*(encodedData + 1) & 0x0F) << 2)];
    }
    // If last byte, ignore postamble sequence
    else
    {
        data[0] = 0x00;
        data[1] = 0x00;
    }

    data[2] = decodeTab[((*(encodedData + 1) & 0xF0) >> 4) | ((*encodedData & 0x03) << 4)];
    data[3] = decodeTab[((*encodedData & 0xFC) >> 2)];

    // - Check for invalid data coding -
    if ((data[0] == 0xFF) | (data[1] == 0xFF) |
        (data[2] == 0xFF) | (data[3] == 0xFF))
    {
#ifdef debug_decoder 
        Serial.print("_e_e_e");
#endif
        return (DecErr);
    }
    else
#ifdef debug_decoder
        Serial.print("      ");
#endif
    // - Shift the encoded values into a unsigned char buffer -
    *decodedData = (data[3] << 4) | (data[2]);
    if (!lastByte)
        *(decodedData + 1) = (data[1] << 4) | (data[0]);

    return (0); // no error
}

unsigned char decode3o6Block(unsigned char *encoded, unsigned char *decoded, unsigned char encodedSize)
{

    unsigned char bytesRemaining;
    unsigned char decodingStatus = 0;

    bytesRemaining = encodedSize;

#ifdef debug_decoder
    Serial.print("raw: ");
    for (int i = 0; i < encodedSize; i++)
    {
        if (encoded[i] < 0x10) Serial.print(0); //leadin zero ugly but ..yeah ..
        Serial.print(encoded[i], HEX);
    }
    Serial.println();
    Serial.print("err: ");
#endif //debug_decoder
    while (bytesRemaining && (decodingStatus != DecErr) )
    {

        // If last byte ..strange what about 2 last bytes ..something is wired here
        if (bytesRemaining == 1)
        {
            decodingStatus = decode3o6(encoded, decoded, 1);
            bytesRemaining -= 1;
        }
        else
        {
            decodingStatus = decode3o6(encoded, decoded, 0);
            bytesRemaining -= 3;

            encoded += 3;
            decoded += 2;
        }

    } // end while bytesRemaining
#ifdef debug_decoder
    Serial.println();
#endif
    return decodingStatus;
}

//----------------------------------------------------------------------------
// void encode3outof6 (uint8 *uncodedData, uint8 *encodedData, uint8 lastByte)
//
//  DESCRIPTION:
//    Performs the "3 out 6" encoding on a 16-bit data value into a
//    24-bit data value. When encoding on a 8 bit variable, a postamle
//    sequence is added.
//
//  ARGUMENTS:
//        uint8 *uncodedData      - Pointer to data
//        uint8 *encodedData      - Pointer to store the encoded data
//        uint8 lastByte          - Only one byte left in data buffer
//----------------------------------------------------------------------------

void encode3outof6(unsigned char *uncodedData, unsigned char *encodedData, unsigned char lastByte)
{

    unsigned char data[4];

    // - Perform encoding -

    // If last byte insert postamble sequence
    if (lastByte)
    {
        data[1] = 0x14;
    }
    else
    {
        data[0] = encodeTab[*(uncodedData + 1) & 0x0F];
        data[1] = encodeTab[(*(uncodedData + 1) >> 4) & 0x0F];
    }

    data[2] = encodeTab[(*uncodedData) & 0x0F];
    data[3] = encodeTab[((*uncodedData) >> 4) & 0x0F];

    // - Shift the encoded 6-bit values into a byte buffer -
    *(encodedData + 0) = (data[3] << 2) | (data[2] >> 4);
    *(encodedData + 1) = (data[2] << 4) | (data[1] >> 2);

    if (!lastByte)
    {
        *(encodedData + 2) = (data[1] << 6) | data[0];
    }
}
