#include "TUVMEDMADevice.hh"
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <unistd.h>

TUVMEDMADevice::TUVMEDMADevice(): TUVMEDevice((uint32_t)-1)
{
  fUseNoIncrement = false;
}

TUVMEDMADevice::~TUVMEDMADevice()
{
}

int TUVMEDMADevice::Enable()
{
  if (!fIsOpen) return -1;
  uint32_t ctlRegister = 0x0;
  switch (fAddressSpace) {
    case kA16:
      ctlRegister |= DMA_VAS_A16;
      break;
    case kA24:
      ctlRegister |= DMA_VAS_A24;
      break;
    case kA32:
      ctlRegister |= DMA_VAS_A32;
      break;
    case kCRCSR:
      ctlRegister |= DMA_VAS_CRCSR;
      break;
    case kUser1:
      ctlRegister |= DMA_VAS_USER1;
      break;
    case kUser2:
      ctlRegister |= DMA_VAS_USER2;
      break;
    default:
      return -1;
  }

  switch (fDataWidth) {
    case kD8:
      ctlRegister |= DMA_VDW_8BIT;
      break;
    case kD16:
      ctlRegister |= DMA_VDW_16BIT;
      break;
    case kD32:
      ctlRegister |= DMA_VDW_32BIT;
      break;
    case kD64:
      ctlRegister |= DMA_VDW_64BIT;
      break;
    default:
      return -1;
  }
  
  switch (fMode) {
    case kProgram:
      ctlRegister |= DMA_PGM_PROG;
      break;
    case kData:
      ctlRegister |= DMA_PGM_DATA;
      break;
    default:
      return -1;
  }

  switch (fType) {
    case kNonPrivileged:
      ctlRegister |= DMA_SUPER_NONP;
      break;
    case kSuper:
      ctlRegister |= DMA_SUPER_SUP;
      break;
    default:
      return -1;
  }

  ctlRegister |= (fUseBLTs) ? DMA_VCT_USE_BLT : DMA_VCT_NO_BLT;
  
  if (fUseNoIncrement) ctlRegister |= DMA_NO_INCREMENT;
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_CTL
			  , ctlRegister) < 0 ) return -1;  
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_VME, fVMEAddress) < 0 ) return -1; 
  return 0;

}
