#ifndef _TUVMEDevice_hh_
#define _TUVMEDevice_hh_

#include <stdint.h>
#ifdef __cplusplus
#include <string>
#include "universe.h"
#include <pthread.h>


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

    inline void Reset() 
      {fPCIOffset=0; fVMEAddress=0; fSizeOfImage=0; fAddressSpace=kA16; fDataWidth=kD8;
       fMode=kData; fType=kNonPrivileged; fUseBLTs=false; fAllowPostedWrites=false;
       fUseIORemap=false;}
    inline void SetPCIOffset(uint32_t offset) {fPCIOffset = offset;}
    void SetVMEAddress(uint32_t vmeAddress); 
    inline void SetSizeOfImage(uint32_t sizeOfImage) {fSizeOfImage = sizeOfImage;}
    int SetWithAddressModifier(uint32_t addressModifier);
    inline void SetAddressSpace(ETUVMEDeviceAddressSpace addressSpace) 
      {fAddressSpace = addressSpace;}
    inline void SetDataWidth(ETUVMEDeviceDataWidth dataWidth) 
      {fDataWidth = dataWidth;}
    inline void SetMode(ETUVMEDeviceMode mode) {fMode = mode;}
    inline void SetType(ETUVMEDeviceType type) {fType = type;}
    inline void SetUseBLTs(bool useBLTs) {fUseBLTs = useBLTs;}
    inline void SetAllowPostedWrites(bool allowPostedWrites) 
      {fAllowPostedWrites = allowPostedWrites;}
    inline void SetUseIORemap(bool useIORemap) 
      {fUseIORemap = useIORemap;}

    inline int32_t GetDevNumber() {return fDevNumber;}
    inline uint32_t GetVMEAddress() {return fVMEAddress;}
    inline uint32_t GetSizeOfImage() {return fSizeOfImage;}

    virtual std::string GetDeviceStringName() {return "vme_m";}    

    int Open();
    virtual int Enable(); 
    void Close();

    /* Locking functions for thread safety. */
    virtual int32_t LockDevice() { return pthread_mutex_lock( &fLock ); }
    virtual int32_t UnlockDevice() { return pthread_mutex_unlock( &fLock ); }

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

    pthread_mutex_t fLock;
     
};
#else
typedef struct TUVMEDevice TUVMEDevice;
#endif /* __cplusplus*/

#endif /* TUVMEDevice.hh */
