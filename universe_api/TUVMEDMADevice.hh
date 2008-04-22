#ifndef _TUVMEDMADevice_hh_
#define _TUVMEDMADevice_hh_

#ifdef __cplusplus
#include "TUVMEDevice.hh"
#include <string>
#include "universe.h"


class TUVMEDMADevice: public TUVMEDevice {

  public:
    TUVMEDMADevice();
    virtual ~TUVMEDMADevice();

    virtual std::string GetDeviceStringName() {return "vme_dma";}    
    virtual int Enable();

    void SetNoIncrement(bool noInc = true) {fUseNoIncrement = noInc;}

  protected:
    bool fUseNoIncrement;

};

#endif /* __cplusplus */
#endif /* TUVMEDMADevice.hh */
