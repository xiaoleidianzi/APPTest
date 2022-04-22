#include "version.h"


const unsigned int Software_Ver  __attribute__((at(VERINFO_ADDR_BASE + 0x00))) = 0xAAAA0100;
const unsigned int Compiler_Date __attribute__((at(VERINFO_ADDR_BASE + 0x04))) = 0x20220310;
