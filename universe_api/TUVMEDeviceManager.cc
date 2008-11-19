#include "TUVMEDeviceManager.hh"

TUVMEDeviceManager::TUVMEDeviceManager()
{
  for (uint32_t i=0;i<TUVMEDevice::kNumberOfDevices;i++) {
    fDevicesRemaining.insert(i);
  }
  fUsePostedWrites = false;
  if (fControlDevice.Open() < 0) {
    /* Error*/
  } else {
    /* Setup device. */
    uint32_t buffer = 0;
    if (fControlDevice.Read((char*)&buffer, sizeof(uint32_t), PCI_CSR)==sizeof(uint32_t)) {
      /* This sets the Bus Master bit in the PCI_CSR register of the Universe II. 
       * This is needed to complete DMA transfers. */
      buffer |= 0x4;
      fControlDevice.Write((char*)&buffer, sizeof(uint32_t), PCI_CSR);
    }
  }
  if (fDMADevice.Open() < 0) {
    /* Error */
  } else {
    /* Setup device. */
  }
  fSizePerImage = fControlDevice.GetPCIMemorySize()/TUVMEDevice::kNumberOfDevices;

  pthread_rwlock_init( &fReadWriteLock, NULL );
}

TUVMEDeviceManager::~TUVMEDeviceManager()
{
  std::set<TUVMEDevice*>::iterator iter; 
  pthread_rwlock_wrlock( &fReadWriteLock );
  for (iter = fAllDevices.begin();iter != fAllDevices.end(); iter++) {  
    delete *iter;
  }
  pthread_rwlock_unlock( &fReadWriteLock );
  pthread_rwlock_destroy( &fReadWriteLock );
}

TUVMEDevice* TUVMEDeviceManager::GetDevice(uint32_t vmeAddress, 
  uint32_t addressModifier, uint32_t dataWidth, uint32_t sizeOfImage)
{
  pthread_rwlock_rdlock( &fReadWriteLock );
  TUVMEDevice* device = NULL;
  /* First check if there is something in the map that can handle this*/
  /* If not, we have to insert it into the set.*/
  /* Check to see if the available devices is 0. */
  if (fDevicesRemaining.empty()) {
    pthread_rwlock_unlock( &fReadWriteLock );
    /* We have to delete a member. */
    return NULL;
  }
  if (sizeOfImage > fSizePerImage) {
    pthread_rwlock_unlock( &fReadWriteLock );
    return NULL; 
  }
  /* Size is too big too handle, try increasing the size of memory allocated
   * to the driver. */

  /* There are some empty devices. */
  /* If the request is for an A16 device, we'll use the 
   * higher granularity images preferentially. */
  uint32_t label;
  while (!fDevicesRemaining.empty()) {
    if (((addressModifier & 0x30) >> 4) == 2) {
      /* A16 */
      if (fDevicesRemaining.find(kA16Dev1) != fDevicesRemaining.end()) {
        label = kA16Dev1;
      } else if (fDevicesRemaining.find(kA16Dev2) != fDevicesRemaining.end()) {
        label = kA16Dev2;
      } else {
        /* Just grab anything. */
        label = *fDevicesRemaining.begin();
      }
    } else {
      /* All other addresses */
      std::set<uint32_t>::iterator setIter;
      for (setIter=fDevicesRemaining.begin();setIter!=fDevicesRemaining.end();setIter++) {
        /* Loops through and tries to find a label that isn't kA16Dev*. */
        label = *setIter; 
        if ((label != kA16Dev1) && (label != kA16Dev2)) break; 
      }
    }

    pthread_rwlock_unlock( &fReadWriteLock );
    pthread_rwlock_wrlock( &fReadWriteLock );
    fDevicesRemaining.erase(label);
    pthread_rwlock_unlock( &fReadWriteLock );

    device = new TUVMEDevice(label);
    if (device->Open() >= 0) {
      break;
    }
    /* We couldn't open, try to loop through to find one to work. */
    pthread_rwlock_rdlock( &fReadWriteLock );
    delete device; 
    device = NULL;
  }
  if (!device) {
    return NULL;
  }
  device->SetWithAddressModifier(addressModifier);
  device->SetDataWidth((TUVMEDevice::ETUVMEDeviceDataWidth)dataWidth);
  device->SetVMEAddress(vmeAddress);
  device->SetAllowPostedWrites(fUsePostedWrites);
  device->SetUseIORemap(true); // we don't remap things yet
  device->SetPCIOffset(label*fSizePerImage);
  device->SetSizeOfImage(((sizeOfImage==0) ? fSizePerImage : sizeOfImage));
  if (device->Enable() < 0) {
    /* Hmmm, this isn't good, not sure what to do? */
    /* Generally this happens if an ioremap fails, (e.g. vmalloc isn't set high enough) */
    /* Keep the device out of the Devices remaining since it could've been in use
     * by someone else. */
    delete device;
    return NULL;
  }
  pthread_rwlock_wrlock( &fReadWriteLock );
  fAllDevices.insert(device);
  pthread_rwlock_unlock( &fReadWriteLock );
  return device;
}

int32_t TUVMEDeviceManager::CloseDevice(TUVMEDevice* device)
{
  /* First make sure it exists in here. */
  pthread_rwlock_rdlock( &fReadWriteLock );
  if (fAllDevices.find(device) == fAllDevices.end()) {
    /* Doesn't exist?  This is odd, some dev file is not being 
     * handled by the manager.*/
    pthread_rwlock_unlock( &fReadWriteLock );
    return -1;
  }
  pthread_rwlock_unlock( &fReadWriteLock );
  pthread_rwlock_wrlock( &fReadWriteLock );
  fAllDevices.erase(device);
  fDevicesRemaining.insert(device->GetDevNumber());
  /* Return it to the available pool. */
  pthread_rwlock_unlock( &fReadWriteLock );

  delete device;
  return 0;
}

TUVMEDevice* TUVMEDeviceManager::GetDMADevice(uint32_t vmeAddress, 
  uint32_t addressModifier, uint32_t dataWidth, bool autoIncrement)
{
  /* We have to setup the DMA device. */
  fDMADevice.LockDevice();
  fDMADevice.SetWithAddressModifier(addressModifier);
  fDMADevice.SetDataWidth((TUVMEDevice::ETUVMEDeviceDataWidth)dataWidth);
  fDMADevice.SetVMEAddress(vmeAddress);
  fDMADevice.SetNoIncrement(!autoIncrement);
  if (fDMADevice.Enable() < 0) return NULL;
  return &fDMADevice;
}
    
/************************************************************************/
/************************************************************************/
/***************** Define c-wrapped functions. **************************/
/************************************************************************/
/************************************************************************/

static TUVMEDeviceManager gUniverseDevMgr;
/* We use a global device manager.  This means that two programs cannot be open at the same time. */

TUVMEDevice* get_new_device(uint32_t vmeAddress, uint32_t addressModifier, 
  uint32_t dataWidth, uint32_t sizeOfImage)
{
  return gUniverseDevMgr.GetDevice(vmeAddress, addressModifier, dataWidth, sizeOfImage);
}

int32_t close_device(TUVMEDevice* device)
{
  return gUniverseDevMgr.CloseDevice(device);
}

TUVMEDevice* get_dma_device(uint32_t vmeAddress, uint32_t addressModifier, uint32_t dataWidth, bool autoIncrement)
{
  return gUniverseDevMgr.GetDMADevice(vmeAddress, addressModifier, dataWidth, autoIncrement);
}

void release_dma_device(void)
{
  gUniverseDevMgr.ReleaseDMADevice();
}

TUVMEDevice* get_ctl_device()
{
  return gUniverseDevMgr.GetControlDevice();
}

void set_hw_byte_swap(bool doSwap)
{
  dynamic_cast<TUVMEControlDevice*>(gUniverseDevMgr.GetControlDevice())->SetHWByteSwap(doSwap);
}

uint32_t get_max_size_of_image()
{
  return gUniverseDevMgr.GetSizePerImage();
}

/*void set_ds_negation_speed(uint32_t speed)
{
  dynamic_cast<TUVMEControlDevice*>(gUniverseDevMgr.GetControlDevice())->SetDSNegationSpeed((TUVMEControlDevice::ECycleSpeeds)speed);
}

void set_ds_high_time_blts(uint32_t speed)
{
  dynamic_cast<TUVMEControlDevice*>(gUniverseDevMgr.GetControlDevice())->SetDSHighTimeBLTs((TUVMEControlDevice::ECycleSpeeds)speed);
}*/

void lock_device(TUVMEDevice* dev)
{
  dev->LockDevice();
}

void unlock_device(TUVMEDevice* dev)
{
  dev->UnlockDevice();
}

int32_t read_device(TUVMEDevice* dev, char* buffer, uint32_t numBytes, uint32_t offset)
{
  return dev->Read(buffer, numBytes, offset);
}
int32_t write_device(TUVMEDevice* dev, char* buffer, uint32_t numBytes, uint32_t offset)
{
  return dev->Write(buffer, numBytes, offset);
}
