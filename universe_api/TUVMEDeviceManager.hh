#ifndef _TUVMEDeviceManager_hh_
#define _TUVMEDeviceManager_hh_


#include "TUVMEDevice.hh"
#include "TUVMEControlDevice.hh"
#include "TUVMEDMADevice.hh"
#include "TUVMEDeviceLock.hh"

#ifdef __cplusplus

#include <set> 
#include <vector> 

class TUVMEDeviceManager {
  public:

    static TUVMEDeviceManager* GetDeviceManager();
    enum ETUVMEDevMgrEnum {kA16Dev1 = 0, kA16Dev2 = 4};

    TUVMEDevice* GetDevice(uint32_t vmeAddress, 
      uint32_t addressModifier, uint32_t dataWidth, uint32_t sizeOfImage = 0);
    TUVMEDevice* GetControlDevice() 
      {return (fControlDeviceIsOpen) ? &fControlDevice : NULL;}

    /* DMA read device. */
    TUVMEDevice* GetDMADevice(uint32_t vmeAddress, 
      uint32_t addressModifier, uint32_t dataWidth, 
      bool autoIncrement);
    void ReleaseDMADevice() { fDMADevice.UnlockDevice(); }

    int32_t CloseDevice(TUVMEDevice* device);

    inline uint32_t GetSizePerImage() {return fSizePerImage;}
    
    void SetUsePostedWrites(bool usePostedWrites = true) 
      {fUsePostedWrites = usePostedWrites;}
    
    virtual int32_t LockDevice() { return fLock.Lock(); }
    virtual int32_t UnlockDevice() { return fLock.Unlock(); }

  protected:
    TUVMEDeviceManager();
    TUVMEDeviceManager(const TUVMEDeviceManager&);
    TUVMEDeviceManager& operator=(const TUVMEDeviceManager&);
    virtual ~TUVMEDeviceManager();
    bool fUsePostedWrites;
    bool fControlDeviceIsOpen;
    bool fDMADeviceIsOpen;

    std::set<TUVMEDevice*> fAllDevices; 
    std::set<uint32_t> fDevicesRemaining;
    TUVMEControlDevice fControlDevice;
    TUVMEDMADevice fDMADevice;
    uint32_t fSizePerImage;

    TUVMEDeviceLock fLock;

  private:
    static TUVMEDeviceManager* gUniverseDeviceManager;
    
};
#endif /*__cplusplus*/

/* The following defines the c function interface. */
/* It is important to note that the following functions are not generally thread-safe. */

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif
extern TUVMEDevice* get_new_device(uint32_t vmeAddress, uint32_t addressModifier, uint32_t dataWidth, uint32_t sizeOfImage);
  /* This function grabs a new device with the given specifications. It will return NULL if there is any error. */
  /* An error can be caused if there are no more available devices, or if something is wrong with the input parameters. */
  /* Not thread safe. */
extern int32_t close_device(TUVMEDevice* device);
  /* Closes a device and releases it back into the available pool.  */
  /* Calling on DMA or CTL devices has no effect. */
  /* Not thread safe. */
extern TUVMEDevice* get_dma_device(uint32_t vmeAddress, uint32_t addressModifier, uint32_t dataWidth, bool autoIncrement);
  /* Grabs the dma device and sets up the transfer.  If NULL, this means that DMA device is busy. */
  /* A transfer from the DMA is initiated with the read_device function. */
  /* A call to this function locks the DMA device to the calling thread.  *
   * It *must* be released by calling release_dma_device(). */
extern void release_dma_device(void);
  /* Returns the DMA device to the available pool. */
  /* This must not be called if the dma device is not owned. */
extern TUVMEDevice* get_ctl_device(void);
  /* Grabs the control device.  If NULL, this means that control device is busy. */
extern void set_hw_byte_swap(bool doSwap);
  /* Sets byte swap in the hardware.  This only works on the VX 40x/04x cpu boards and has undefined behavior for other boards. */
extern uint32_t get_max_size_of_image(void);
  /* Returns the maximum size of an image. */
//extern void set_ds_negation_speed(uint32_t speed);
  /* Sets the ds negation speed during a read cycle. 
   * speed:  0 - normal 
   *         1 - faster 
   *         2 - fastest
   *
   * Usage of this in a non-normal state could violate the VME64 standard. 
   * However, it is also possible to speed up bus cycles. */
//extern void set_ds_high_time_blts(uint32_t speed);
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
/* Locking functions for use with threads.  This is important when a device 
   could be called across different threads. */
extern void lock_device(TUVMEDevice*);
extern void unlock_device(TUVMEDevice*);
extern int32_t setup_device(TUVMEDevice* dev, uint32_t vme_address, uint32_t address_modifier, uint32_t data_width);
extern int32_t read_device(TUVMEDevice*, char* buffer, uint32_t numBytes, uint32_t offset);
  /* reads numBytes bytes from a device into a buffer at an offset on the device. */
extern int32_t write_device(TUVMEDevice*, char* buffer, uint32_t numBytes, uint32_t offset);
  /* writes numBytes bytes into a device from a buffer at an offset on the device. */
#ifdef __cplusplus
}
#endif

#endif
