// this is just a note for later use

//ToDo should include all the handling of the meters data
//decoupled by the RF receiver message (frame, mBusMsg)

//############## Test array
uint8_t temsg[]= {
  0x36, 0x44, 0x68, 0x50, 0x50, 0x60, 0x58, 0x30, 0x22, 0x43, 0xa1, 0x00,
  0xde, 0x28, 0xea, 0xf9, 0x00, 0x38, 0x21, 0x00, 0x00, 0x00, 0xe9, 0xc0,
  0x04, 0x31, 0x38, 0x12, 0x8a, 0x3c, 0xe0, 0xdc, 0x56, 0x69, 0x99, 0x27,
  0xe2, 0xe9, 0xb3, 0xc1, 0xff, 0x5e, 0xeb, 0xe9, 0x89, 0x7a, 0x9d, 0x34,
  0x8b, 0x22, 0x32, 0xa4, 0x80, 0x81, 0x08
}; //len = 55

inline void testmsg () {
wMBusMsg MBMtest;

MBMtest.parseraw(testbuffer, sizeof(testbuffer));

}