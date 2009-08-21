#ifndef _TUVMEDeviceLock_hh_
#define _TUVMEDeviceLock_hh_

/*
 * This class provides a simple interface to pthread mutexes.
 */

#ifdef __cplusplus
#include <pthread.h>
class TUVMEDeviceLock
{
  public:
    TUVMEDeviceLock() { pthread_mutex_init(&fLock, NULL); }
    virtual ~TUVMEDeviceLock() { pthread_mutex_destroy(&fLock); }

    virtual int32_t Lock() { return pthread_mutex_lock( &fLock ); }
    virtual int32_t Unlock() { return pthread_mutex_unlock( &fLock ); }

  protected:
    pthread_mutex_t fLock;
    
};
#endif /*__cplusplus*/
#endif /* TUVMEDeviceLock*/
