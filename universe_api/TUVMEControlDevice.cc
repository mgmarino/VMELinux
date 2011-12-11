#include "TUVMEControlDevice.hh"
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include "ConcurrentVX40x.h"

TUVMEControlDevice::TUVMEControlDevice(): TUVMEDevice((uint32_t)-1)
{
  fRevisionID = 0;
}

TUVMEControlDevice::~TUVMEControlDevice()
{
}

int TUVMEControlDevice::Open()
{
  int status = TUVMEDevice::Open();
  if (status < 0) return status;
  /* Get the Revision id. */
  status = Read((char*)&fRevisionID, sizeof(uint32_t), PCI_CLASS);
  if (status < 0) {
    fRevisionID = 0;
    return status;
  }
  fRevisionID &= 0xf;
  return 0;
}

void TUVMEControlDevice::SetHWByteSwap(bool doByteSwap)
{
  /* Currently, we only support the Concurrent board 
   * and VMIC-based boards with this funcionality.   */
  EBoardType type = GetBoardType();
  uint8_t value = 0;
  uint32_t temp = 0;
  switch (type) {
    case kCCT:
      value = ReadIOPortMemory( CONCURRENT_VX_CSR0 );
      /* First make sure the bits are cleared. */
      value &= ~ ( CONCURRENT_HW_BYTE_SWAP_MASTER | 
                   CONCURRENT_HW_BYTE_SWAP_SLAVE  |
                   CONCURRENT_HW_BYTE_SWAP_FAST );
      /* Now set them. */
      value |= CONCURRENT_HW_BYTE_SWAP_MASTER;
      WriteIOPortMemory( CONCURRENT_VX_CSR0, value );
      break;
    case kVMIC:
      if ( ioctl(fFileNum, UNIVERSE_IOCREAD_VME_COMM, &temp) < 0 ) return; 

      // We disable the big endianness and the bypass.  
      temp &= ~UNIVERSE_VMIC_ENABLE_ENDIAN_CONV_BYPASS;
      temp |= UNIVERSE_VMIC_ENABLE_MASTER_BIG_ENDIAN;
      temp |= UNIVERSE_VMIC_ENABLE_SLAVE_BIG_ENDIAN;
      if ( ioctl(fFileNum, UNIVERSE_IOCSET_VME_COMM, temp) < 0 ) return;
      break;
    default:
      break;
  }

}

size_t TUVMEControlDevice::GetPCIMemorySize()
{
  if (!fIsOpen) return 0;
  size_t argument = 0;
  if (ioctl(fFileNum, UNIVERSE_IOCGET_MEM_SIZE, &argument) < 0) return 0;
  return argument;
}

void TUVMEControlDevice::SetDSNegationSpeed(ECycleSpeeds speed) 
{
  if (!fIsOpen) return;
  /* The revision has to be 01 or 02. */ 
  if (fRevisionID != 1 || fRevisionID != 2) return;
  uint32_t scratch = 0;
  if (Read((char*)&scratch, sizeof(uint32_t), U2SPEC) < 0) return;
  scratch &= 0xFFFFFCFF;
  switch (speed) {
    case kNormal:
      break;
    case kFaster:
      scratch |= 0x00000100; 
      break;
    case kFastest:
      scratch |= 0x00000200;
      break;
    default: 
      break;
  }
  Write((char*)scratch, sizeof(uint32_t), U2SPEC);
}

void TUVMEControlDevice::SetDSHighTimeBLTs(ECycleSpeeds speed) 
{
  if (!fIsOpen) return;
  /* The revision has to be 01 or 02. */ 
  if (fRevisionID != 1 || fRevisionID != 2) return;
  uint32_t scratch = 0;
  if (Read((char*)&scratch, sizeof(uint32_t), U2SPEC) < 0) return;
  scratch &= 0xFFFFFBFF;
  switch (speed) {
    case kNormal:
      break;
    case kFaster:
      scratch |= 0x00000400;
      break;
    case kFastest:
      scratch |= 0x00000400;
      break;
    default: 
      break;
  }
  Write((char*)scratch, sizeof(uint32_t), U2SPEC);
}

TUVMEControlDevice::EBoardType TUVMEControlDevice::GetBoardType()
{
  if (!fIsOpen) return kUnknown;
  size_t argument = 0;
  if (ioctl(fFileNum, UNIVERSE_IOCGET_BOARD_TYPE, &argument) < 0) return kUnknown;
  return (EBoardType)argument;
 
}

int TUVMEControlDevice::ReadIOPortMemory(uint16_t address)
{
  if (!fIsOpen) return -1;
  struct universe_ioport_ioctl tempStruct;
  tempStruct.address = address;
  if ( ioctl(fFileNum, UNIVERSE_IOCIO_PORT_READ, &tempStruct) < 0 ) return -1;
  return tempStruct.value;
}

int TUVMEControlDevice::WriteIOPortMemory(uint16_t address, uint8_t value)
{
  if (!fIsOpen) return -1;
  struct universe_ioport_ioctl tempStruct;
  tempStruct.address = address;
  tempStruct.value = value;
  if ( ioctl(fFileNum, UNIVERSE_IOCIO_PORT_WRITE, &tempStruct) < 0 ) return -1;
  return 0;

}
