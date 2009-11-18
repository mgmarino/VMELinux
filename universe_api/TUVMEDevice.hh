#ifndef _TUVMEDevice_hh_
#define _TUVMEDevice_hh_

#include <stdint.h>
#ifdef __cplusplus
#include <string>
#include "universe.h"
#include <sys/mman.h>
#include "TUVMEDeviceLock.hh"

/*
 * This class represents a slave window in the Tundra Universe II chip.
 */

class TUVMEDevice {

  public:
    TUVMEDevice(uint32_t devNumber);
    virtual ~TUVMEDevice();

    enum ETUVMEDeviceEnum {kNumberOfDevices = 8};

    enum ETUVMEDeviceAddressSpace { kA16 = 0,
                                    kA24 = 1,
                                    kA32 = 2, 
                                    kCRCSR = 3, 
                                    kUser1 = 4, 
                                    kUser2 = 5};

    enum ETUVMEDeviceDataWidth { kD8 = 1,
                                 kD16 = 2,
                                 kD32 = 4,
                                 kD64 = 8};

    enum ETUVMEDeviceMode { kProgram = 0, kData };

    enum ETUVMEDeviceType { kNonPrivileged = 0, kSuper };

    inline void SetPCIOffset(uint32_t offset) {fPCIOffset = offset;}
    void SetVMEAddress(uint32_t vmeAddress); 

    inline void SetSizeOfImage(uint32_t sizeOfImage) 
      {fSizeOfImage = sizeOfImage; WriteControlRegister();}

    int32_t SetWithAddressModifier(uint32_t addressModifier);
    inline void SetAddressSpace(ETUVMEDeviceAddressSpace addressSpace) 
      {fAddressSpace = addressSpace; WriteControlRegister(); }

    inline void SetDataWidth(ETUVMEDeviceDataWidth dataWidth) 
      {fDataWidth = dataWidth; WriteControlRegister();}

    inline void SetMode(ETUVMEDeviceMode mode) 
      {fMode = mode; WriteControlRegister();}

    inline void SetType(ETUVMEDeviceType type) 
      {fType = type; WriteControlRegister();}

    inline void SetUseBLTs(bool useBLTs) 
      {fUseBLTs = useBLTs; WriteControlRegister();}

    inline void SetAllowPostedWrites(bool allowPostedWrites) 
      {fAllowPostedWrites = allowPostedWrites; WriteControlRegister();}

    inline void SetUseIORemap(bool useIORemap) 
      {fUseIORemap = useIORemap;}

    inline int32_t GetDevNumber() {return fDevNumber;}
    inline uint32_t GetVMEAddress() {return fVMEAddress;}
    inline uint32_t GetSizeOfImage() {return fSizeOfImage;}

    inline volatile void* GetMappedAddress() {return fMappedAddress;}

    virtual std::string GetDeviceStringName() {return "vme_m";}    
    int32_t CheckBusError();

    int32_t Open();
    /*Synchronize the control register of the chip with the state of this object. */
    virtual int32_t WriteControlRegister();

    /* Enable device */
    virtual int32_t Enable(); 
    void Close();

    /* Locking functions for thread safety. */
    virtual int32_t LockDevice() { return fLock.Lock(); }
    virtual int32_t UnlockDevice() { return fLock.Unlock(); }

    int32_t Read(char* buffer, uint32_t numBytes, uint32_t offset = 0);
    int32_t Write(char* buffer, uint32_t numBytes, uint32_t offset = 0);

  protected:
    uint32_t fPCIOffset;
    uint32_t fVMEAddress;
    uint32_t fSizeOfImage;
    ETUVMEDeviceAddressSpace fAddressSpace;
    ETUVMEDeviceDataWidth fDataWidth;
    ETUVMEDeviceMode fMode;
    ETUVMEDeviceType fType;
    bool fUseBLTs;
    bool fAllowPostedWrites;
    bool fUseIORemap;
   
    int32_t fFileNum;
    bool fIsOpen;
    int32_t fDevNumber;

    volatile void* fMappedAddress;

    TUVMEDeviceLock fLock; // lock for this particular slave window
    static TUVMEDeviceLock fSystemLock; // lock shared between all devices. 
     
    inline void Reset() 
      {fPCIOffset=0; fVMEAddress=0; fSizeOfImage=0; fAddressSpace=kA16; fDataWidth=kD8;
       fMode=kData; fType=kNonPrivileged; fUseBLTs=false; fAllowPostedWrites=false;
       fUseIORemap=false; fMappedAddress=NULL;}
};
#else
typedef struct TUVMEDevice TUVMEDevice;
#endif /* __cplusplus*/

#endif /* TUVMEDevice.hh */
