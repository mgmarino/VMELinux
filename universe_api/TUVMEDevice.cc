#include "TUVMEDevice.hh"
#include <sstream>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <cstring>

TUVMEDeviceLock TUVMEDevice::fSystemLock;

TUVMEDevice::TUVMEDevice(uint32_t devNumber)
{
  Reset();
  fIsOpen = false;
  fFileNum = -1;
  fDevNumber = devNumber;
}

TUVMEDevice::~TUVMEDevice()
{
  Close();
}

int32_t TUVMEDevice::Open()  
{
  std::ostringstream os;
  if (fDevNumber >= (int32_t)kNumberOfDevices) {
    return -1; //Error
  } 
  os << "/dev/"<< GetDeviceStringName();
  if (fDevNumber >= 0) {
    os << fDevNumber;
  }
  if ((fFileNum = open(os.str().c_str(), O_RDWR)) < 0) {
    fIsOpen = false;
    return fFileNum; // Error
  }
  fIsOpen = true;
  return 0;
}

int32_t TUVMEDevice::WriteControlRegister()
{
  if (!fIsOpen) {
    return -1;
  }

  uint32_t ctlRegister = 0x0;

  switch (fDataWidth) {
    case TUVMEDevice::kD8:
      ctlRegister |= PCI_VDW_8BIT;
      break;
    case TUVMEDevice::kD16:
      ctlRegister |= PCI_VDW_16BIT;
      break;
    case TUVMEDevice::kD32:
      ctlRegister |= PCI_VDW_32BIT;
      break;
    case TUVMEDevice::kD64:
      ctlRegister |= PCI_VDW_64BIT;
      break;
    default:
      return -1;
  }
 

  switch (fAddressSpace) {
    case kA16:
      ctlRegister |= PCI_VAS_A16;
      break;
    case kA24:
      ctlRegister |= PCI_VAS_A24;
      break;
    case kA32:
      ctlRegister |= PCI_VAS_A32;
      break;
    case kCRCSR:
      ctlRegister |= PCI_VAS_CRCSR;
      break;
    case kUser1:
      ctlRegister |= PCI_VAS_USER1;
      break;
    case kUser2:
      ctlRegister |= PCI_VAS_USER2;
      break;
    default:
      return -1;
  }

 
  switch (fMode) {
    case kProgram:
      ctlRegister |= PCI_PGM_PROG;
      break;
    case kData:
      ctlRegister |= PCI_PGM_DATA;
      break;
    default:
      return -1;
  }

  switch (fType) {
    case kNonPrivileged:
      ctlRegister |= PCI_SUPER_NONP;
      break;
    case kSuper:
      ctlRegister |= PCI_SUPER_SUP;
      break;
    default:
      return -1;
  }
  
  ctlRegister |= (fUseBLTs) ? PCI_VCT_USE_BLT : PCI_VCT_NO_BLT;
  if (fAllowPostedWrites) {
    ctlRegister |= PCI_POSTED_WRITE;
  }
  /* Now enable the channel. */
  ctlRegister |= 0x80000000;

  if ( ioctl(fFileNum, UNIVERSE_IOCSET_CTL, ctlRegister) < 0 ) return -1;  
  return 0;

}

int32_t TUVMEDevice::SetWithAddressModifier(uint32_t addressModifier) 
{
  if ((addressModifier & 0xF) <= 0x7) return -1;
    /* We are not equipped to handle other types of AMs.*/
  switch ((addressModifier & 0x30) >> 4) { 
      case 3:
          fAddressSpace = kA24;
          break;
      case 2:
          fAddressSpace = kA16;
          break;
      case 1:
          /* User defined address space. */
          return -1;
          break;
      case 0:
          fAddressSpace = kA32;
          break;
  } 

  SetType(((addressModifier & 0x4) >> 2) ? kSuper : kNonPrivileged);
  SetUseBLTs(!(((addressModifier & 0x2) >> 1) ^ (addressModifier & 0x1))); 
  SetMode((addressModifier & 0x1) ? kData : kProgram); 
  return 0;

}

int32_t TUVMEDevice::Enable()
{
  if ( WriteControlRegister() < 0 ) return -1;


  /* Setting memory handling. */
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_IOREMAP, ((fUseIORemap) ? 1 : 0)) < 0 ) return -1;  
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_BS, fPCIOffset) < 0 ) return -1; 
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_BD, fSizeOfImage) < 0 ) return -1; 
  if ( ioctl(fFileNum, UNIVERSE_IOCSET_VME, fVMEAddress) < 0 ) return -1; 
  if ( !fUseIORemap ) {
    /* Attemp to memory map the space. */
    fMappedAddress = mmap(0, fSizeOfImage, PROT_READ | PROT_WRITE, MAP_SHARED, fFileNum, 0);
    if ( fMappedAddress == MAP_FAILED ) { 
      fMappedAddress = NULL;
      return -1;
    }
  }
  return 0;
}

void TUVMEDevice::SetVMEAddress(uint32_t vmeAddress) 
{
  fVMEAddress = vmeAddress; 
  if (fIsOpen) ioctl(fFileNum, UNIVERSE_IOCSET_VME, fVMEAddress);
}
void TUVMEDevice::Close() 
{
  if (fIsOpen) {
    close(fFileNum);
    fIsOpen = false;
  }
  if ( fMappedAddress ) munmap( (void*)fMappedAddress, fSizeOfImage ); 
}

int32_t TUVMEDevice::Read(char* buffer, uint32_t numBytes, uint32_t offset)
{
  if (!fIsOpen) return 0; 
  int32_t checker = 0;
  /* The following lock is to prevent any other device from accessing the
     universe chip and to ensure that the checked bus error corresponds
     to the previous read.  */
  fSystemLock.Lock();
  if ( fMappedAddress ) {
    memcpy(buffer, (char*)fMappedAddress + offset, numBytes); 
    checker = CheckBusError();
    fSystemLock.Unlock();
    if ( checker == 0 ) return numBytes;
    else return 0;
  } else {
    lseek(fFileNum, offset, SEEK_SET);
    checker = read(fFileNum, buffer, numBytes); 
    fSystemLock.Unlock();
    return checker;
  }
}

int32_t TUVMEDevice::Write(char* buffer, uint32_t numBytes, uint32_t offset)
{
  if (!fIsOpen) return 0; 
  int32_t checker = 0;
  /* The following lock is to prevent any other device from accessing the
     universe chip and to ensure that the checked bus error corresponds
     to the previous read.  */
  fSystemLock.Lock();
  if ( fMappedAddress ) {
    memcpy((char*)fMappedAddress + offset, buffer, numBytes); 
    checker = CheckBusError();
    fSystemLock.Unlock();
    if ( checker == 0 ) return numBytes;
    else return 0;
  } else {
    lseek(fFileNum, offset, SEEK_SET);
    checker = write(fFileNum, buffer, numBytes); 
    fSystemLock.Unlock();
    return checker;
  }
}

int32_t TUVMEDevice::CheckBusError() 
{
  return ioctl( fFileNum, UNIVERSE_IOCCHECK_BUS_ERROR ); 
}
