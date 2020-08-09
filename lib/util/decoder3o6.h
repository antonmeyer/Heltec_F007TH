#ifndef decoder3o6_h
#define decoder3o6_h
#define DECODING_3OUTOF6_OK 0
#define DECODING_3OUTOF6_ERROR 1



static const unsigned char LenOk = 0x1;
static const unsigned char LenErr = 0x2;
static const unsigned char DecErr = 0x4;
static const unsigned char CRCErr = 0x8;

unsigned char decode3o6(unsigned char *encodedData, unsigned char *decodedData, unsigned char lastByte);
unsigned char decode3o6Block(unsigned char *encoded, unsigned char *decoded, unsigned char encodedSize);
void encode3outof6(unsigned char *uncodedData, unsigned char *encodedData, unsigned char lastByte);

// Table for encoding for a 4-bit data into 6-bit
// "3 out of 6" coded data.
static unsigned char encodeTab[16] = {0x16,  // 0x0 "3 out of 6" encoded
								0x0D,  // 0x1 "3 out of 6" encoded
								0x0E,  // 0x2 "3 out of 6" encoded
								0x0B,  // 0x3 "3 out of 6" encoded
								0x1C,  // 0x4 "3 out of 6" encoded
								0x19,  // 0x5 "3 out of 6" encoded
								0x1A,  // 0x6 "3 out of 6" encoded
								0x13,  // 0x7 "3 out of 6" encoded
								0x2C,  // 0x8 "3 out of 6" encoded
								0x25,  // 0x9 "3 out of 6" encoded
								0x26,  // 0xA "3 out of 6" encoded
								0x23,  // 0xB "3 out of 6" encoded
								0x34,  // 0xC "3 out of 6" encoded
								0x31,  // 0xD "3 out of 6" encoded
								0x32,  // 0xE "3 out of 6" encoded
								0x29}; // 0xF "3 out of 6" encoded

static unsigned char decodeTab[64] = {0xFF,  //  "3 out of 6" encoded 0x00 decoded
									  0xFF,  //  "3 out of 6" encoded 0x01 decoded
									  0xFF,  //  "3 out of 6" encoded 0x02 decoded
									  0xFF,  //  "3 out of 6" encoded 0x03 decoded
									  0xFF,  //  "3 out of 6" encoded 0x04 decoded
									  0xFF,  //  "3 out of 6" encoded 0x05 decoded
									  0xFF,  //  "3 out of 6" encoded 0x06 decoded
									  0xFF,  //  "3 out of 6" encoded 0x07 decoded
									  0xFF,  //  "3 out of 6" encoded 0x08 decoded
									  0xFF,  //  "3 out of 6" encoded 0x09 decoded
									  0xFF,  //  "3 out of 6" encoded 0x0A decoded
									  0x03,  //  "3 out of 6" encoded 0x0B decoded
									  0xFF,  //  "3 out of 6" encoded 0x0C decoded
									  0x01,  //  "3 out of 6" encoded 0x0D decoded
									  0x02,  //  "3 out of 6" encoded 0x0E decoded
									  0xFF,  //  "3 out of 6" encoded 0x0F decoded
									  0xFF,  //  "3 out of 6" encoded 0x10 decoded
									  0xFF,  //  "3 out of 6" encoded 0x11 decoded
									  0xFF,  //  "3 out of 6" encoded 0x12 decoded
									  0x07,  //  "3 out of 6" encoded 0x13 decoded
									  0xFF,  //  "3 out of 6" encoded 0x14 decoded
									  0xFF,  //  "3 out of 6" encoded 0x15 decoded
									  0x00,  //  "3 out of 6" encoded 0x16 decoded
									  0xFF,  //  "3 out of 6" encoded 0x17 decoded
									  0xFF,  //  "3 out of 6" encoded 0x18 decoded
									  0x05,  //  "3 out of 6" encoded 0x19 decoded
									  0x06,  //  "3 out of 6" encoded 0x1A decoded
									  0xFF,  //  "3 out of 6" encoded 0x1B decoded
									  0x04,  //  "3 out of 6" encoded 0x1C decoded
									  0xFF,  //  "3 out of 6" encoded 0x1D decoded
									  0xFF,  //  "3 out of 6" encoded 0x1E decoded
									  0xFF,  //  "3 out of 6" encoded 0x1F decoded
									  0xFF,  //  "3 out of 6" encoded 0x20 decoded
									  0xFF,  //  "3 out of 6" encoded 0x21 decoded
									  0xFF,  //  "3 out of 6" encoded 0x22 decoded
									  0x0B,  //  "3 out of 6" encoded 0x23 decoded
									  0xFF,  //  "3 out of 6" encoded 0x24 decoded
									  0x09,  //  "3 out of 6" encoded 0x25 decoded
									  0x0A,  //  "3 out of 6" encoded 0x26 decoded
									  0xFF,  //  "3 out of 6" encoded 0x27 decoded
									  0xFF,  //  "3 out of 6" encoded 0x28 decoded
									  0x0F,  //  "3 out of 6" encoded 0x29 decoded
									  0xFF,  //  "3 out of 6" encoded 0x2A decoded
									  0xFF,  //  "3 out of 6" encoded 0x2B decoded
									  0x08,  //  "3 out of 6" encoded 0x2C decoded
									  0xFF,  //  "3 out of 6" encoded 0x2D decoded
									  0xFF,  //  "3 out of 6" encoded 0x2E decoded
									  0xFF,  //  "3 out of 6" encoded 0x2F decoded
									  0xFF,  //  "3 out of 6" encoded 0x30 decoded
									  0x0D,  //  "3 out of 6" encoded 0x31 decoded
									  0x0E,  //  "3 out of 6" encoded 0x32 decoded
									  0xFF,  //  "3 out of 6" encoded 0x33 decoded
									  0x0C,  //  "3 out of 6" encoded 0x34 decoded
									  0xFF,  //  "3 out of 6" encoded 0x35 decoded
									  0xFF,  //  "3 out of 6" encoded 0x36 decoded
									  0xFF,  //  "3 out of 6" encoded 0x37 decoded
									  0xFF,  //  "3 out of 6" encoded 0x38 decoded
									  0xFF,  //  "3 out of 6" encoded 0x39 decoded
									  0xFF,  //  "3 out of 6" encoded 0x3A decoded
									  0xFF,  //  "3 out of 6" encoded 0x3B decoded
									  0xFF,  //  "3 out of 6" encoded 0x3C decoded
									  0xFF,  //  "3 out of 6" encoded 0x3D decoded
									  0xFF,  //  "3 out of 6" encoded 0x3E decoded
									  0xFF}; // "3 out of 6" encoded 0x3F decoded



#endif