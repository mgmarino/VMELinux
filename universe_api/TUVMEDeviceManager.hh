#ifndef _TUVMEDeviceManager_hh_
#define _TUVMEDeviceManager_hh_


#include "TUVMEDevice.hh"
#include "TUVMEControlDevice.hh"
#include "TUVMEDMADevice.hh"

#ifdef __cplusplus

#include <set> 
#include <vector> 

class TUVMEDeviceManager {
  public:
    TUVMEDeviceManager();
    ~TUVMEDeviceManager();

    enum ETUVMEDevMgrEnum {kA16Dev1 = 0, kA16Dev2 = 4};

    TUVMEDevice* GetDevice(uint32_t vmeAddress, 
      uint32_t addressModifier, uint32_t dataWidth, uint32_t sizeOfImage = 0);
    TUVMEDevice* GetControlDevice() {return &fControlDevice;}
    TUVMEDevice* GetDMADevice(uint32_t vmeAddress, 
      uint32_t addressModifier, uint32_t dataWidth);
    int32_t CloseDevice(TUVMEDevice* device);

    inline uint32_t GetSizePerImage() {return fSizePerImage;}
    
    void SetUsePostedWrites(bool usePostedWrites = true) 
      {fUsePostedWrites = usePostedWrites;}
    void SetUseNonIncremented(bool useNoInc = true) 
      {fUseNonIncremented = useNoInc;}
  protected:
    bool fUsePostedWrites;
    bool fUseNonIncremented;

    std::set<TUVMEDevice*> fAllDevices; 
    std::set<uint32_t> fDevicesRemaining;
    TUVMEControlDevice fControlDevice;
    TUVMEDMADevice fDMADevice;
    uint32_t fSizePerImage;
    
};
#endif /*__cplusplus*/

/* The following defines the c function interface. */

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif
extern TUVMEDevice* get_new_device(uint32_t vmeAddress, uint32_t addressModifier, uint32_t dataWidth, uint32_t sizeOfImage);
  /* This function grabs a new device with the given specifications. It will return NULL if there is any error. */
  /* An error can be caused if there are no more available devices, or if something is wrong with the input parameters. */
extern int32_t close_device(TUVMEDevice* device);
  /* Closes a device and releases it back into the available pool.  */
extern TUVMEDevice* get_dma_device(uint32_t vmeAddress, uint32_t addressModifier, uint32_t dataWidth);
  /* Grabs the dma device and sets up the transfer.  If NULL, this means that DMA device is busy. */
  /* A transfer from the DMA is initiated with the read_device function. */
extern TUVMEDevice* get_ctl_device(void);
  /* Grabs the control device.  If NULL, this means that control device is busy. */
extern void set_dma_no_increment(bool noInc);
  /* This specifies that the dma device should not increment a VME address. This is useful if a dma read
   * of x bytes is required at one particular address.  
   * It should be called before get_dma_device.*/
extern void set_hw_byte_swap(bool doSwap);
  /* Sets byte swap in the hardware.  This only works on the VX 40x/04x cpu boards and has undefined behavior for other boards. */
extern uint32_t get_max_size_of_image(void);
  /* Returns the maximum size of an image. */
extern void set_ds_negation_speed(uint32_t speed);
  /* Sets the ds negation speed during a read cycle. 
   * speed:  0 - normal 
   *         1 - faster 
   *         2 - fastest
   *
   * Usage of this in a non-normal state could violate the VME64 standard. 
   * However, it is also possible to speed up bus cycles. */
extern void set_ds_high_time_blts(uint32_t speed);
  /* Sets the ds high time during a block transfer. 
   * speed:  0 - normal 
   *         1 - faster 
   *
   * Usage of this in a non-normal state could violate the VME64 standard. 
   * However, it is possible to speed up block transfers. */ 
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern int32_t read_device(TUVMEDevice*, char* buffer, uint32_t numBytes, uint32_t offset);
  /* reads numBytes bytes from a device into a buffer at an offset on the device. */
extern int32_t write_device(TUVMEDevice*, char* buffer, uint32_t numBytes, uint32_t offset);
  /* writes numBytes bytes into a device from a buffer at an offset on the device. */
#ifdef __cplusplus
}
#endif

#endif