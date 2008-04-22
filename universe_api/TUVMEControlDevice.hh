#ifndef _TUVMEControlDevice_hh_
#define _TUVMEControlDevice_hh_

#ifdef __cplusplus
#include "TUVMEDevice.hh"
#include <string>
#include "universe.h"


class TUVMEControlDevice: public TUVMEDevice {

  public:
    TUVMEControlDevice();
    virtual ~TUVMEControlDevice();

    enum ECycleSpeeds { kNormal  = 0,
                        kFaster  = 1,
                        kFastest = 2};
    
    virtual std::string GetDeviceStringName() {return "vme_ctl";}    
    virtual int Enable() {return 0;} 
    virtual int Open();

    void SetHWByteSwap(bool doByteSwap = true);
    void SetDSNegationSpeed(ECycleSpeeds speed = kNormal);
    void SetDSHighTimeBLTs(ECycleSpeeds speed = kNormal);
    size_t GetPCIMemorySize();

  protected:
    
    uint32_t fRevisionID;
};

#endif /* __cplusplus */
#endif /* TUVMEControlDevice.hh */
