uint8_t Checksum(int length, uint8_t *buff)
{
    uint8_t mask = 0x7C;
    uint8_t checksum = 0x64;
    uint8_t data;
    
 
    for ( uint8_t byteCnt=0; byteCnt < length; byteCnt++)
    {
        int bitCnt;
        data = buff[byteCnt];
 
        for ( bitCnt= 7; bitCnt >= 0 ; bitCnt-- )
        {
            uint8_t bit;
 
            // Rotate mask right
            bit = mask & 1;
            mask = (mask >> 1 ) | (mask << 7);
            if ( bit )
            {
                mask ^= 0x18;
            }
 
            // XOR mask into checksum if data bit is 1
            if ( data & 0x80 )
            {
                checksum ^= mask;
            }
            data <<= 1;
        }
    }
    return checksum;
}
