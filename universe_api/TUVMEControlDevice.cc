#include "TUVMEControlDevice.hh"
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <unistd.h>

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
  if (!fIsOpen) return;
  ioctl(fFileNum, UNIVERSE_IOCSET_HW_BYTESWAP, ((doByteSwap) ? UNIVERSE_IOCMASTER_BYTESWAP : 0));  
}

size_t TUVMEControlDevice::GetPCIMemorySize()
{
  if (!fIsOpen) return 0;
  uint32_t argument = 0;
  if (ioctl(fFileNum, UNIVERSE_IOCGET_MEM_SIZE, (uint32_t)&argument) < 0) return 0;
  return (size_t)argument;
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
