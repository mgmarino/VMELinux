//------------------------------------------------------------------------------  
//title: Tundra Universe PCI-VME Kernel Driver
//version: Linux 1.1
//date: March 1999                                                                
//designer: Michael Wyrick                                                      
//programmer: Michael Wyrick                                                    
//platform: Linux 2.4.x
//language: GCC 2.95 and 3.0
//module: universe
//------------------------------------------------------------------------------  
// some major code fixes/updates to linux kernel 2.6.  Also added functionality, 
// fixed bugs.  DMA moved to its own device.  Control moved to its own device and
// limited opening.  Added much more error checking, memory reservation.
//
// Michael Marino (Jan 2008)
//
//------------------------------------------------------------------------------  
//  Purpose: Provide a Kernel Driver to Linux for the Universe I and II 
//           Universe model number universe
//  Docs:                                  
//    This driver supports both the Universe and Universe II chips                                     
//------------------------------------------------------------------------------  
// RCS:
// $Id: universe.c,v 1.5 2001/10/27 03:50:07 jhuggins Exp $
// $Log: universe.c,v $
// Revision 1.5  2001/10/27 03:50:07  jhuggins
// These changes add support for the extra images offered by the Universe II.
// CVS : ----------------------------------------------------------------------
//
// Revision 1.6  2001/10/16 15:16:53  wyrick
// Minor Cleanup of Comments
//
//
//-----------------------------------------------------------------------------

static char Version[] = "1.3.r1 2008Jan07";

#ifndef __KERNEL__
#define __KERNEL__
#endif


#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
#include <linux/utsrelease.h>
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif

#ifdef CONFIG_MODVERSIONS
  #define MODVERSIONS
  #include <linux/modversions.h>
#endif

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>

#include "universe.h"

/* Why are these not included?!?*/
#ifndef PCI_VENDOR_ID_TUNDRA
  #define PCI_VENDOR_ID_TUNDRA 0x10e3
#endif

#ifndef PCI_DEVICE_ID_TUNDRA_CA91C042
  #define PCI_DEVICE_ID_TUNDRA_CA91C042 0x0000
#endif
#define VX407_CSR0   0x210


//----------------------------------------------------------------------------
// Module parameters 
//----------------------------------------------------------------------------
static unsigned long sizeToReserve = 0x10000000; // 256 MB reserved
static unsigned long reserveFromAddress = 0xC0000000; // reserve from high mem
#ifdef MODULE
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
  MODULE_PARM(sizeToReserve, "i");
  MODULE_PARM(reserveFromAddress, "i");
#else
  module_param(sizeToReserve, long, 0);
  module_param(reserveFromAddress, long, 0);
#endif
  MODULE_PARM_DESC(sizeToReserve, "Give the size of pci space to reserve. ");
  MODULE_PARM_DESC(reserveFromAddress, "Give a starting pci address from which to reserve.");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,10))
  MODULE_LICENSE("GPL");
#endif
#endif



//----------------------------------------------------------------------------
// Prototypes
//----------------------------------------------------------------------------
static int uni_open(struct inode *, struct file *);
static int uni_release(struct inode *, struct file *);
static ssize_t uni_read(struct file *,char *, size_t, loff_t *);
static ssize_t uni_write(struct file *,const char *, size_t, loff_t *);
static unsigned int uni_poll(struct file *, poll_table *);
static int uni_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static long long uni_llseek(struct file *,loff_t,int);
static int uni_mmap(struct file *,struct vm_area_struct *);
//static int uni_mmap(struct inode *inode,struct file *file,struct vm_area_struct *vma);
//static int uni_select(struct inode *inode,struct file *file,int mode, select_table *table);
//
static void uni_vma_open(struct vm_area_struct *);
static void uni_vma_close(struct vm_area_struct *);
static int uni_procinfo(char *, char **, off_t, int, int *,void *);
static void register_proc(void);
static void unregister_proc(void);


//----------------------------------------------------------------------------
// Types
//----------------------------------------------------------------------------
static struct proc_dir_entry *uni_procdir;

static struct file_operations uni_fops = 
{
  llseek:   uni_llseek,
  read:     uni_read,
  write:    uni_write,
  poll:     uni_poll,     // uni_poll   
  ioctl:    uni_ioctl,
  open:     uni_open,
  release:  uni_release, 
  mmap:     uni_mmap 
};

static struct vm_operations_struct uni_vma_remap_ops = 
{
  .open = uni_vma_open,
  .close = uni_vma_close
};

static int aCTL[] = {LSI0_CTL, LSI1_CTL, LSI2_CTL, LSI3_CTL,
                     LSI4_CTL, LSI5_CTL, LSI6_CTL, LSI7_CTL};
                     
static int aBS[]  = {LSI0_BS,  LSI1_BS,  LSI2_BS,  LSI3_BS,
                     LSI4_BS,  LSI5_BS,  LSI6_BS,  LSI7_BS}; 
                     
static int aBD[]  = {LSI0_BD,  LSI1_BD,  LSI2_BD,  LSI3_BD,
                     LSI4_BD,  LSI5_BD,  LSI6_BD,  LSI7_BD}; 

static int aTO[]  = {LSI0_TO,  LSI1_TO,  LSI2_TO,  LSI3_TO,
                     LSI4_TO,  LSI5_TO,  LSI6_TO,  LSI7_TO}; 

//----------------------------------------------------------------------------
// Vars and Defines
//----------------------------------------------------------------------------
//#define UNI_MAJOR      221
#define MAX_MINOR       9 // This includes the DMA_MINOA
#define CONTROL_MINOR   8
#define DMA_MINOR      9

//#define MODE_UNDEFINED  0
//#define MODE_PROGRAMMED 1
//#define MODE_DMA        2

#define DMATYPE_SINGLE  1
#define DMATYPE_LLIST   2

static int OkToWrite[MAX_MINOR + 1];        // Can I write to the Hardware

static int opened[MAX_MINOR + 1];
//static int mode[MAX_MINOR + 1];             // DMA or Programmed I/O

static unsigned long DMA;     // DMA Control Reg
static unsigned long DMA_vme; // DMA VME address

static int status;
static int DMAType = 0;
static int irq = 0;
static char *baseaddr  = 0;
static char *image_ba[MAX_MINOR+1];         // Base PCI Address

static unsigned int image_ptr[MAX_MINOR+1];
//static unsigned int image_va[MAX_MINOR+1];
static unsigned int image_perform_ioremap[MAX_MINOR+1];  // array to let us know if the memory should be ioremapped
static unsigned int image_is_ioremapped[MAX_MINOR+1];  // array to let us know if the image is ioremapped. 

// Hold the VME Irq Handlers
static TirqHandler vmeirqs[7] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static short vmeirqbit[7]     = {IRQ_VIRQ1, IRQ_VIRQ2, IRQ_VIRQ3, IRQ_VIRQ4,
                                 IRQ_VIRQ5, IRQ_VIRQ6, IRQ_VIRQ7};               
static int vmevec[7]          = {V1_STATID, V2_STATID, V3_STATID, V4_STATID,
                                 V5_STATID, V6_STATID, V7_STATID};

// Status Vars
static unsigned int reads  = 0;
static unsigned int writes = 0;
static unsigned int ioctls = 0;

static TDMAcallback DMACallBackFunc = NULL;
static void *debugptr = NULL;

// Timers
struct timer_list DMA_timer;                // This is a timer for returning status

// Hold onto the major Number as it is dynamically given
static int majorNumber = 0;
/* the number shows up in /proc/devices and can be taken from there by a script which 
 * generates the dev files. */
static int memoryIsReserved = 0;



//----------------------------------------------------------------------------
//  uni_procinfo()
//----------------------------------------------------------------------------
static int uni_procinfo(char *buf, char **start, off_t fpos, int lenght, int *eof, void *data)
{
  char *p;
  unsigned int temp1,temp2,x;

  p = buf;
  p += sprintf(p,"Universe driver %s\n",Version);

  p += sprintf(p,"  baseaddr = %08X\n",(int)baseaddr);
  p += sprintf(p,"  Stats  reads = %i  writes = %i  ioctls = %i\n",
                 reads,writes,ioctls);

  for (x=0;x<8;x+=2) {
    temp1 = readl(baseaddr+aCTL[x]);
    temp2 = readl(baseaddr+aCTL[x+1]);
    p += sprintf(p,"  LSI%i_CTL = %08X    LSI%i_CTL = %08X\n",x,temp1,x+1,temp2);
    temp1 = readl(baseaddr+aBS[x]);
    temp2 = readl(baseaddr+aBS[x+1]);
    p += sprintf(p,"  LSI%i_BS  = %08X    LSI%i_BS  = %08X\n",x,temp1,x+1,temp2);
    temp1 = readl(baseaddr+aBD[x]);
    temp2 = readl(baseaddr+aBD[x+1]);
    p += sprintf(p,"  LSI%i_BD  = %08X    LSI%i_BD  = %08X\n",x,temp1,x+1,temp2);
    temp1 = readl(baseaddr+aTO[x]);
    temp2 = readl(baseaddr+aTO[x+1]);
    p += sprintf(p,"  LSI%i_TO  = %08X    LSI%i_TO  = %08X\n",x,temp1,x+1,temp2);
  }  

 /* for (x=0;x<8;x+=2)
    p += sprintf(p,"  image_va%i   = %08X     image_va%i   = %08X\n",
                 x,image_va[x],x+1,image_va[x+1]);*/ 
  p += sprintf(p,"\nDriver Program Status:\n");  

  //for (x=0;x<8;x+=2)
    p += sprintf(p,"  DMACTL   = %08lX \n",
                 DMA);        
  for (x=0;x<8;x+=2)
    p += sprintf(p,"  OkToWrite %i = %1X        OkToWrite %i = %1X\n",
                 x,OkToWrite[x],x+1,OkToWrite[x+1]); 
  /*for (x=0;x<8;x+=2)
    p += sprintf(p,"  Mode %i      = %1X        Mode %i      = %1X\n",
                 x,mode[x],x+1,mode[x+1]); 
  */
  p += sprintf(p,"\n");  

  temp1 = 0;
  p += sprintf(p, "VMEIrqs Assigned: ");
  for (x=0;x<7;x++) {
    if (vmeirqs[x] != NULL) {
      p += sprintf(p, "%i ",x+1);
      temp1++;
    }
  }
  if (temp1 == 0)
    p += sprintf(p, "none\n");
  else
    p += sprintf(p,"\n");  

  *eof = 1;
  return p - buf;
}

//----------------------------------------------------------------------------
//  register_proc()
//----------------------------------------------------------------------------
static void register_proc()
{
  uni_procdir = create_proc_entry("universe", S_IFREG | S_IRUGO, 0);
  uni_procdir->read_proc = uni_procinfo;
}

//----------------------------------------------------------------------------
//  unregister_proc()
//----------------------------------------------------------------------------
static void unregister_proc()
{
  remove_proc_entry("universe",0);
}

//----------------------------------------------------------------------------
//
//  uni_poll()
//
//----------------------------------------------------------------------------
static unsigned int uni_poll(struct file* file, poll_table* wait)
{
  return 0;
}

//----------------------------------------------------------------------------
//
//  uni_open()
//
//----------------------------------------------------------------------------
static int uni_open(struct inode *inode,struct file *file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  if (minor > MAX_MINOR) {
    return(-ENODEV);
  }
/*
  if (minor == CONTROL_MINOR) {
    opened[minor]++;
    return(0);
  }
*/ // Multiple copies of ctl opened is asking for disaster.
  if (!opened[minor]) {
    opened[minor] = 1;
    /* increment the module usage*/
    try_module_get(THIS_MODULE);
    return(0);
  } else {
    return(-EBUSY);
  }
}

//----------------------------------------------------------------------------
//
//  uni_release()
//
//----------------------------------------------------------------------------
static int uni_release(struct inode *inode,struct file *file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  if (opened[minor] > 1) {
    printk("  Closing minor:%d, file open more than once?!\n", minor);
  }
  opened[minor]--;

  OkToWrite[minor] = 0;
  if (minor != CONTROL_MINOR && minor != DMA_MINOR) {
    /* Disable the image, reset the registers */
    writel(0x0, baseaddr+aCTL[minor]);
    writel(0x0, baseaddr+aBS[minor]);
    writel(0x0, baseaddr+aBD[minor]);
    writel(0x0, baseaddr+aTO[minor]);
    if (image_ba[minor] && image_is_ioremapped[minor] == 1) {
      /* Get rid of the ioremapped memory. */
      iounmap(image_ba[minor]);
      image_is_ioremapped[minor] = 0;
      image_ba[minor] = 0;
    }
  } 
  /* decrement the module usage*/
  module_put(THIS_MODULE);
  return 0;
}

//----------------------------------------------------------------------------
//
//  uni_lseek()
//
//----------------------------------------------------------------------------
static long long uni_llseek(struct file *file,loff_t offset,int whence)
{
  unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
  //unsigned int toffset,base,paddr;
  if (minor == DMA_MINOR) return -EPERM;
  if (whence == SEEK_SET) {
    if (minor == CONTROL_MINOR) {
      image_ptr[minor] = offset;
    } else {
      //toffset = readl(baseaddr+aTO[minor]);    
      //base    = readl(baseaddr+aBS[minor]);    
      //paddr   = offset-toffset;
      //image_ptr[minor] = (int)(image_ba[minor]+(paddr-base));
      image_ptr[minor] = (int)(image_ba[minor]+offset);
    }  
    return 0;
  } else if (whence == SEEK_CUR) {
    image_ptr[minor] += offset;
    return 0;
  } else {
    return -EPERM;  
  }
}

//----------------------------------------------------------------------------
//
//  uni_read()
//
//----------------------------------------------------------------------------
static ssize_t uni_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
  int x = 0;
  unsigned int v,numt,remain,tmp;
  char *temp = buf;
  unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);  
  int p; 

  unsigned char  vc;
  unsigned short vs;
  unsigned int   vl;

  char *DMA_Buffer;
  unsigned int DMA_Buffer_Size = 0, 
    order     = 0,
    a_size    = 0,
    dma_align = 0,
    timeout   = 0;                     
  int val     = 0,
    pci       = 0,
    okcount   = 0;

  if (minor == CONTROL_MINOR) {
    p = (int)image_ptr[minor];
    v = readl(baseaddr+p);    
    if (__copy_to_user(temp,&v,4) != 0 ) {
      return (-EIO);
    }
  } else {
    reads++;                
    if (OkToWrite[minor] == 1) {
      if (minor == DMA_MINOR) {
        // ------------------------------------------------------------------
        //
        // ------------------------------------------------------------------      
        // Wait for DMA to finish, This needs to be changed
        val = readl(baseaddr+DGCS);
        while ((val & 0x00008000) && (timeout++ < 1000000))
          val = readl(baseaddr+DGCS);

        // VME Address ( and PCI address must be 8-byte aligned with each other)				      
        dma_align = DMA_vme & 0x7;  

        // Setup DMA Buffer to read data into
        DMA_Buffer_Size = count + dma_align;         
        a_size = PAGE_SIZE;
        while (a_size < DMA_Buffer_Size) {
          order++;
          a_size <<= 1;
        }
        DMA_Buffer = (char *)__get_dma_pages(GFP_KERNEL,order);  
        //printk("DMA buffer: 0x%lx",DMA_Buffer);

        // PCI Address
        pci = virt_to_bus(DMA_Buffer) + dma_align;

        // Setup DMA regs
        // setup DMA for a *read*
        DMA &= 0x7FFFFFFF; // Sets L2V bit for a read
        writel(DMA,baseaddr+DCTL);       // Setup Control Reg
        writel(count,baseaddr+DTBC);     // Count	    
        writel(pci,baseaddr+DLA);        // PCI Address
        writel(DMA_vme,baseaddr+DVA);    // VME Address
        //printk("DMA CTL, count, pci address, VME Address: 0x%lx,0x%x,0x%x,0x%lx\n", 
        //  DMA, count, pci, DMA_vme);

        // Start DMA
        //printk("Starting DMA\n");
        writel(0x80006F00,baseaddr+DGCS);       // GO

        // Wait for DMA to finish, This needs to be changed
        val = readl(baseaddr+DGCS);
        //printk("Val: 0x%x\n",val);
        while ((val & 0x00008000) && (timeout++ < 100000))
          val = readl(baseaddr+DGCS);

        if (timeout == 100000)
          printk("<<universe DMA Timed out>>\n");

        if (! (val & 0x00000800)) {  // An Error Happened
          count -= readl(baseaddr+DTBC);    
          printk("Error happened, DTBC read back: 0x%x\n",val);
        }

        // Copy pages to User Memory
        if (__copy_to_user(temp,DMA_Buffer+dma_align,count) != 0 ) {
          return (-EIO);
        }

        free_pages((unsigned long)DMA_Buffer,order);
      } else { 
        numt = count;
        remain = count;

        // Calc the number of longs we need
        numt = count / 4;
        remain = count % 4;
        for (x=0;x<numt;x++) {
          vl = readl((char*)image_ptr[minor]);    

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount += 4;

         if ( __copy_to_user(temp,&vl,4) != 0) {
            return (-EIO);
          }
          image_ptr[minor]+=4;
          temp+=4;
        }  

        // Calc the number of Words we need
        numt = remain / 2;
        remain = remain % 2;
        for (x=0;x<numt;x++) {
          vs = readw((char*)image_ptr[minor]);    

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount += 2;

          if (__copy_to_user(temp,&vs,2) != 0) {
            return (-EIO);
          }
          image_ptr[minor]+=2;
          temp+=2;
        }  

        for (x=0;x<remain;x++) {
          vc = readb((char*)image_ptr[minor]);    

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount++;

          if (__copy_to_user(temp,&vc,1) != 0) {
            return (-EIO);
          }
          image_ptr[minor]+=1;
          temp+=1;
        }  
      }
    } else {            
      return -EPERM;
    }// end of OKtoWrite 
  }  
  *ppos += count;
  return(count);
}

//----------------------------------------------------------------------------
//
//  uni_write()
//
//----------------------------------------------------------------------------
static ssize_t uni_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
  int x,p;
  unsigned int numt,remain,tmp;
  char *temp = (char *)buf;
  unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);

  unsigned char  vc;
  unsigned short vs;
  unsigned int   vl;

  char *DMA_Buffer;
  unsigned int DMA_Buffer_Size = 0, 
    order     = 0,
    a_size    = 0,
    dma_align = 0,
    timeout   = 0;                     
  int val,
    pci       = 0,
    okcount   = 0;

  writes++;
  if (minor == CONTROL_MINOR) {
    __copy_from_user(&vl,temp,4);
    p = (int)image_ptr[minor];
    writel(vl,baseaddr+p);
  } else {
    if (OkToWrite[minor] == 1) {
      if (minor == DMA_MINOR) {
        // deal with the dma 
        // ------------------------------------------------------------------
        //
        // ------------------------------------------------------------------      
        // Wait for DMA to finish, This needs to be changed
        val = readl(baseaddr+DGCS);
        while ((val & 0x00008000) && (timeout++ < 100000))
          val = readl(baseaddr+DGCS);

        // Setup DMA Buffer to write data into
        // VME Address, which must be 8-byte aligned (the bottom 3 bits must be the same)				      
        dma_align = DMA_vme & 0x7;  

        DMA_Buffer_Size = count + dma_align;         
        a_size = PAGE_SIZE;
        while (a_size < DMA_Buffer_Size) {
          order++;
          a_size <<= 1;
        }
        DMA_Buffer = (char *)__get_dma_pages(GFP_KERNEL,order);  

        // Copy User Memory into buffer
        __copy_from_user(DMA_Buffer + dma_align,temp,count);

        // PCI Address
        pci = virt_to_bus(DMA_Buffer) + dma_align;

        // Setup DMA regs
        DMA |= 0x80000000; // Sets L2V bit for a write
        writel(DMA,baseaddr+DCTL);  // Setup Control Reg
        writel(count,baseaddr+DTBC);                  // Count	    
        writel(pci,baseaddr+DLA);                     // PCI Address
        writel(DMA_vme,baseaddr+DVA);                     // VME Address

        // Start DMA
        writel(0x80006F00,baseaddr+DGCS);             // GO

        // Wait for DMA to finish, This needs to be changed
        val = readl(baseaddr+DGCS);
        while ((val & 0x00008000) && (timeout++ < 100000))
          val = readl(baseaddr+DGCS);

        if (timeout == 100000)
          printk("<<universe DMA Timed out>>\n");

        if (! (val & 0x00000800)) {  // An Error Happened
          // The Universe Seems to return an invalid value in DTBC on
          // Bus Errors during DMA, so invalidate the count
          count = 0;
        }

        free_pages((unsigned long)DMA_Buffer,order);
      } else {          
        // dealing with an image minor
        // Calc the number of longs we need
        numt = count;
        remain = count;

        numt = count / 4;
        remain = count % 4;
        for (x=0;x<numt;x++) {
          __copy_from_user(&vl,temp,4);
          writel(vl,(char*)image_ptr[minor]);

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount += 4;

          image_ptr[minor]+=4;
          temp+=4;
        }  

        // Calc the number of Words we need
        numt = remain / 2;
        remain = remain % 2;

        for (x=0;x<numt;x++) {
          __copy_from_user(&vs,temp,2);
          writew(vs,(char*)image_ptr[minor]);

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount += 2;

          image_ptr[minor]+=2;
          temp+=2;
        }  

        for (x=0;x<remain;x++) {
          __copy_from_user(&vc,temp,1);
          writeb(vc,(char*)image_ptr[minor]);

          // Lets Check for a Bus Error
          tmp = readl(baseaddr+PCI_CSR);
          if (tmp & 0x08000000) {
            writel(tmp,baseaddr+PCI_CSR);
            return(-EIO);
          } else
            okcount += 2;

          image_ptr[minor]+=1;
          temp+=1;
        }  

        // Lets Check for a Bus Error
        tmp = readl(baseaddr+PCI_CSR);
        if (tmp & 0x08000000) {  // S_TA is Set
          writel(0x08000000,baseaddr+PCI_CSR);
          return(-EIO);
        }
      }
    }  else { //OKtoWrite
      return -EPERM;
    }
  }  
  *ppos += count;
  return(count);
}

//----------------------------------------------------------------------------
//  uni_ioctl()
//----------------------------------------------------------------------------
//  For a normal image minor, initialize it by (in this order):
//    IOCTL_SET_CTL: Set control register (address space, data space, etc.)
//    IOCTL_SET_IOREMAP: set to ioremap pci mem to kernel memory.
//    IOCTL_SET_BS:  Set base (offset) of window.  The driver will automatically
//      determine where this is within it's allowed PCI space.
//    IOCTL_SET_BD:  Set bound (size) of window.
//    IOCTL_SET_VME: Use this to set the desired VME address the base
//      of the window will point to.
//
//  For a DMA minor, initialize it by (in this order):
//    IOCTL_SET_CTL: Set control register (address space, data space, etc.)
//    IOCTL_SET_VME: Use this to set the desired VME address from which the 
//      DMA will start. 
//
//  For a control minor, the following functions are allowed:
//    IOCTL_SET_HW_BYTESWAP: set hardware byteswap
//    IOCTL_GET_MEM_SIZE:    get size of pci memory set aside by the driver
//    default: write ard to the register given by the cmd number

static int uni_ioctl(struct inode *inode,struct file *file,unsigned int cmd, unsigned long arg)
{
  unsigned int minor = MINOR(inode->i_rdev);
  unsigned long sizetomap = 0, bs = 0;
  unsigned char readBack = 0;
  unsigned long *tempPtr;

  ioctls++;
  switch (cmd) {
  case IOCTL_SET_CTL:
    if (minor == CONTROL_MINOR) return -EPERM;
    if (minor == DMA_MINOR) {
      // Lets compute and save the DMA CTL Register
      DMA = arg & 0x80C7F380; //crazy bitmask, a bunch of reserved areas
      OkToWrite[minor] = 1;
    } else {
      arg &= 0xFFFFFFFE; // *always* use memory space, not i/o 
      writel(arg,baseaddr+aCTL[minor]);
    }
    break;

  case IOCTL_SET_BS:
    if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
    if (arg >= sizeToReserve ) {
    /* we want to make sure that the base offset doesn't overshoot our reserved memory. */
      return -EFAULT;
    }
    writel(arg+reserveFromAddress,baseaddr+aBS[minor]);
    break;

  case IOCTL_SET_BD:
    if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
    bs = readl(baseaddr+aBS[minor]);
    if (arg+bs < sizeToReserve+reserveFromAddress) {
      /* we want to make sure that the bound doesn't overshoot our reserved memory. */
      
      writel(arg+bs,baseaddr+aBD[minor]);
      if (image_ba[minor] && image_is_ioremapped[minor] == 1) {
        iounmap(image_ba[minor]);
        image_is_ioremapped[minor] = 0;
        image_ba[minor] = 0;
      }
  
      // This uses the BS Register to Find the size of the Image Mapping		
  
      if (image_perform_ioremap[minor] == 1) {
        sizetomap = arg; 
        image_ba[minor] = (char *)ioremap(bs, sizetomap);
        if (!image_ba[minor]) {
          OkToWrite[minor] = 0;     
          printk( KERN_ERR "Error in ioremap, address: %lx, size: %lx\n", arg + reserveFromAddress, sizetomap);
          return -EFAULT;
        }
        image_ptr[minor] = (int)image_ba[minor];
        OkToWrite[minor] = 1;
        image_is_ioremapped[minor] = 1;
      } else {
        /* This disallows the use of the read/write functions of the device. */
        /* If a device is not ioremmapped it must not be read out with these 
         * functions. */
        OkToWrite[minor] = 0;
      }
  
  
    } else {
      return -EFAULT;
    }
    break;  
  case IOCTL_SET_VME:
    /* We get a VME address and insert the offset. */
    /* BS needs to be set before this. */
    if (minor == CONTROL_MINOR) return -EPERM;
    // Calculate the VME Address
    if (minor == DMA_MINOR) {
      DMA_vme = arg;
    } else {
      bs = readl(baseaddr+aBS[minor]);
      writel(arg-bs,baseaddr+aTO[minor]);
      bs = readl(baseaddr+aTO[minor]);
    }
    break;  
  case IOCTL_SET_IOREMAP:
    if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
    image_perform_ioremap[minor] = (arg == 0) ? 0 : 1;
    break;
  case IOCTL_SET_HW_BYTESWAP:
    if (minor != CONTROL_MINOR) return -EPERM;
    readBack = (0x38 & arg);
    outb(readBack, VX407_CSR0);  
    if ((0x38 & arg) != (readBack = (0x38 & inb(VX407_CSR0)))) { 
      printk(  "  Hardware swap not set at address 0x%x. Set: 0x%x, Readback: 0x%x", 
        VX407_CSR0, (unsigned int)(0x38 & arg), readBack);
      return -EIO;
    }
    break;
  case IOCTL_GET_MEM_SIZE:
    if (minor != CONTROL_MINOR) return -EPERM;
    tempPtr = (unsigned long*) arg;
    /* only returning the size.  The mem location is handled internally. */
    /* It is the responsibility of the calling API to appropriately 
     * setup how the memory exists in the chunk (i.e. by setting up offsets).*/
    return __copy_to_user(tempPtr, &sizeToReserve, sizeof(sizeToReserve));
    break; 
  default:
    if (minor != CONTROL_MINOR) return -EPERM;
    /* only let the control minor write to this. */
    if (cmd < 0x1000) {  // This is a Register value so write to it.
      writel(arg,baseaddr+cmd);
    } else return -EFAULT;
    break;  
  }
  return(0);
}

//----------------------------------------------------------------------------
//  uni_mmap()
//----------------------------------------------------------------------------
static int uni_mmap(struct file *file,struct vm_area_struct *vma)
{
  // grab the minor 
  unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
  unsigned long bs = 0, bd = 0;
  unsigned long offset, physical_address, virtual_size, physical_size;
  if (image_is_ioremapped[minor] == 1) {
    // Image is ioremmaped, this is disallowed!
    // return operation not permitted.
    printk("  mmap failure:Image is already ioremapped.");
    return -EPERM;
  }

  bs = readl(baseaddr+aBS[minor]);
  bd = readl(baseaddr+aBD[minor]);

  offset = vma->vm_pgoff << PAGE_SHIFT;
  physical_address = bs + offset;
  virtual_size = vma->vm_end - vma->vm_start;

  if ((bd - bs) < offset) {
    /* offset is too large! */
    printk("  mmap failure:offset (%lx) is too large.", offset);
    return -EFAULT;
  }
  physical_size = (bd - bs) - offset;  
  if (virtual_size > physical_size) {
    /* Range spans too much */
    printk("  mmap failure:range (%lx) spans too much.", virtual_size);
    return -EINVAL;
  }

  //vma->vm_flags |= VM_RESERVED;
  // we do not want to have this area swapped out, lock it
  //vma->vm_flags |= VM_IO;
  /* The cpu CANNOT cache these pages.  This might not work. */
  //vma->vm_page_prot = 
  //  pgprot_noncached(vma->vm_page_prot);
  /* OK, now remap. */
  printk("  Mapping physical address (0x%lx), size (0x%lx) to user space", 
    physical_address, virtual_size);
  if (remap_pfn_range(vma, vma->vm_start, physical_address, 
        virtual_size, vma->vm_page_prot)) {
    /* Something failed, maybe try again? */
    return -EAGAIN;
  }
  vma->vm_ops = &uni_vma_remap_ops;
  vma->vm_pgoff = physical_address >> PAGE_SHIFT;
  uni_vma_open(vma);
  return 0;
}


//----------------------------------------------------------------------------
//  uni_vma_open()
//----------------------------------------------------------------------------
static void uni_vma_open(struct vm_area_struct *vma)
{
  printk("Opening Universe II VMA, virt: %lx, phys: %lx\n", 
    vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}
//----------------------------------------------------------------------------
//  uni_vma_close()
//----------------------------------------------------------------------------
static void uni_vma_close(struct vm_area_struct *vma)
{
  printk("Closing Universe II VMA, virt: %lx, phys: %lx\n", 
    vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}
//-----------------------------------------------------------------------------
// Function   : PrintCmdPacketList
// Inputs     : TDMA_Cmd_Packet* cpl (PCI Phys Address)
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void PrintCmdPacketList(TDMA_Cmd_Packet* cpl)
{
  TDMA_Cmd_Packet *p = bus_to_virt((int)cpl);
  char buff[100];
  int x = 0;
  int done = 0;

  while (!done && (x < 10)) {
    x++;
    sprintf(buff,"<universe> (%i) dctl=%08X, dtbc=%08X\n",x,p->dctl,p->dtbc);
    printk(buff);
    sprintf(buff,"<universe> dlv=%08X, dva=%08X\n",p->dlv,p->dva);
    printk(buff);
    sprintf(buff,"<universe> dcpp=%08X\n",p->dcpp);
    printk(buff);
    done = (p->dcpp & 0x00000001);
    p = bus_to_virt((int)(TDMA_Cmd_Packet*)(p->dcpp & 0xFFFFFFF8));  // Next in Line Please
  }
}

//-----------------------------------------------------------------------------
// Function   : void DMA_status
// Inputs     : unsigned long __data
// Outputs    : static
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void DMA_status(unsigned long __data)
{
  printk("<<universe>> DMA Timed out\n");

  if (DMACallBackFunc)
    DMACallBackFunc(1);
  DMACallBackFunc = NULL;
}

//-----------------------------------------------------------------------------
// Function   : DMA_irq_handler
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void DMA_irq_handler(void)
{
  int val;
  int cbv=0;
  char buf[100];
  TDMA_Cmd_Packet *cpl;

  del_timer(&DMA_timer);  // Cancel DMA Time Out

  val = readl(baseaddr+DGCS);

  if (!(val & 0x00000800)) {
    sprintf(buf,"<<universe>> DMA Error in DMA_irq_handler DGCS=%08X\n",val);  
    printk(buf);
    val = readl(baseaddr+DCPP);
    sprintf(buf,"<<universe>> DCPP=%08X\n",val);  
    printk(buf);
    val = readl(baseaddr+DCTL);
    sprintf(buf,"<<universe>> DCTL=%08X\n",val);  
    printk(buf);
    val = readl(baseaddr+DTBC);
    sprintf(buf,"<<universe>> DTBC=%08X\n",val);  
    printk(buf);
    val = readl(baseaddr+DLA);
    sprintf(buf,"<<universe>> DLA=%08X\n",val);  
    printk(buf);
    val = readl(baseaddr+DVA);
    sprintf(buf,"<<universe>> DVA=%08X\n",val);  
    printk(buf);

    if (DMAType == DMATYPE_LLIST) {
      printk("<<universe>> CmdPacketList sent to DMARead\n");
      PrintCmdPacketList(debugptr);

      printk("<<universe>> CmdPacketList as seen by DCPP\n");
      cpl = (TDMA_Cmd_Packet*)readl(baseaddr+DCPP);   // Command Packet Pointer
      cpl = (TDMA_Cmd_Packet*)((int)cpl & ~0x03);

      PrintCmdPacketList(cpl);
    }
    cbv = 1;  // Call Back value is set to 1 to show error condition
  }

  if (DMACallBackFunc)
    DMACallBackFunc(cbv);

  // We are done with the Call Back so clear it
  DMACallBackFunc = NULL;
}

//-----------------------------------------------------------------------------
// Function   : LERR_irq_handler
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void LERR_irq_handler(void)
{
  int val;
  char buf[100];
  TDMA_Cmd_Packet *cpl;

  del_timer(&DMA_timer);  // Cancel DMA Time Out

  val = readl(baseaddr+DGCS);

  if (!(val & 0x00000800)) {
    sprintf(buf,"<<universe>> LERR_irq_handler DMA Read Error DGCS=%08X\n",val);  
    printk(buf);

    if (DMAType == DMATYPE_LLIST) {
      cpl = (TDMA_Cmd_Packet*)readl(baseaddr+DCPP);   // Command Packet Pointer
      cpl = (TDMA_Cmd_Packet*)((int)cpl & ~0x03);
      PrintCmdPacketList(cpl);

      sprintf(buf,"<<universe>> DMAReadCallBack Request of cmdpack=0x%08X\n",(int)cpl);
      printk(buf);
    }
  }

  if (DMACallBackFunc)
    DMACallBackFunc(1);
  DMACallBackFunc = NULL;
}

//-----------------------------------------------------------------------------
// Function   : VERR_irq_handler
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void VERR_irq_handler(void)
{
  int val;
  char buf[100];
  TDMA_Cmd_Packet *cpl;

  del_timer(&DMA_timer);  // Cancel DMA Time Out

  val = readl(baseaddr+DGCS);

  if (!(val & 0x00000800)) {
    sprintf(buf,"<<universe>> VERR_irq_handler DMA Read Error DGCS=%08X\n",val);  
    printk(buf);

    if (DMAType == DMATYPE_LLIST) {
      cpl = (TDMA_Cmd_Packet*)readl(baseaddr+DCPP);   // Command Packet Pointer
      cpl = (TDMA_Cmd_Packet*)((int)cpl & ~0x03);

      PrintCmdPacketList(cpl);

      sprintf(buf,"<<universe>> DMAReadCallBack Request of cmdpack=0x%08X\n",(int)cpl);
      printk(buf);
    }
  }

  if (DMACallBackFunc)
    DMACallBackFunc(1);
  DMACallBackFunc = NULL;
}

//-----------------------------------------------------------------------------
// Function   : irq_handler
// Inputs     : int irq, struct pt_regs *regs
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
irqreturn_t irq_handler(int irq, void *ptr)
{
  long stat, enable, i, vector;

  enable = readl(baseaddr+LINT_EN);
  stat = readl(baseaddr+LINT_STAT);

  if (stat & 0x0100)
    DMA_irq_handler();
  if (stat & 0x0200)
    LERR_irq_handler();
  if (stat & 0x0400)
    VERR_irq_handler();

  for (i=0;i<7;i++) {
    vector = readl(baseaddr+vmevec[i]);
    if (stat & vmeirqbit[i] & enable) {
      if (vmeirqs[i] != NULL) {
        vmeirqs[i](i+1, vector, ptr);
      }
    }
  }

  writel(stat, baseaddr+LINT_STAT);                // Clear all pending ints
  return IRQ_HANDLED;
}

//-----------------------------------------------------------------------------
// Function   : cleanup_module
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void cleanup_module(void)
{          
  int x;
  int pcivector;

  writel(0,baseaddr+LINT_EN);                   // Turn off Ints
  pcivector = readl(baseaddr+PCI_MISC1) & 0x000000FF; 
  free_irq(pcivector,NULL);             // Free Vector

  for (x=1;x<MAX_MINOR+1;x++) {
    if (image_ba[x] && image_is_ioremapped[x] == 1)
      iounmap(image_ba[x]);
  }    
  iounmap(baseaddr);
  if (memoryIsReserved == 1) {
    release_mem_region(reserveFromAddress, sizeToReserve); 
  }
  unregister_proc();
  unregister_chrdev(majorNumber, "universe");
}

//----------------------------------------------------------------------------
//  init_module()
//----------------------------------------------------------------------------
int init_module(void)
{
  int x, result;
  unsigned int temp, ba;
  char vstr[80];

  struct pci_dev *uni_pci_dev = NULL;

  sprintf(vstr,"Tundra Universe PCI-VME Bridge Driver %s\n",Version);
  printk(vstr);
  printk("  Copyright 1997-2001, Michael J. Wyrick\n");
  printk("  Copyright 2008, Michael G. Marino\n");

  if ((uni_pci_dev = pci_get_device(PCI_VENDOR_ID_TUNDRA,
                                     PCI_DEVICE_ID_TUNDRA_CA91C042, 
                                     uni_pci_dev))) {
    printk("  Universe device found.");

    // Lets turn Latency off
    pci_write_config_dword(uni_pci_dev, PCI_MISC0, 0);

    // Display PCI Registers  
    pci_read_config_dword(uni_pci_dev, PCI_CSR, &status);
    printk("  Vendor = %04X  Device = %04X  Status = %08X\n",
           uni_pci_dev->vendor,uni_pci_dev->device,status);
    printk("  Class = %08X\n",uni_pci_dev->class);

    pci_read_config_dword(uni_pci_dev, PCI_MISC0, &temp);
    printk("  Misc0 = %08X\n",temp);      

    // Setup Universe Config Space
    // This is a 4k wide memory area that need to be mapped into the kernel
    // virtual memory space so we can access it.
    pci_write_config_dword(uni_pci_dev, PCI_BS, CONFIG_REG_SPACE);
    pci_read_config_dword(uni_pci_dev, PCI_BS, &ba);        
    baseaddr = (char *)ioremap(ba,4096);
    if (!baseaddr) {
      printk("  vremap failed to map Universe to Kernel Space.\r");
      return 1;
    }

    // Check to see if the Mapping Worked out
    printk("  baseaddr read in as: %02x\n", (unsigned int)baseaddr);
    temp = readl(baseaddr);
    printk("  Read via mapping, PCI_ID = %08X\n",temp);       
    if (temp != 0x000010E3) {
      printk("  Universe Chip Failed to Return PCI_ID in Memory Map.\n");
      return 1;
    }

    // OK, Every this is ok so lets turn off the windows
    writel(0x00800000,baseaddr+LSI0_CTL);     
    writel(0x00800000,baseaddr+LSI1_CTL);     
    writel(0x00800000,baseaddr+LSI2_CTL);     
    writel(0x00800000,baseaddr+LSI3_CTL);     

    // Write to Misc Register
    // Set VME Bus Time-out
    //   Arbitration Mode
    //   DTACK Enable
    writel(0x15060000,baseaddr+MISC_CTL);     

    // Lets turn off interrupts
    writel(0x00000000,baseaddr+LINT_EN);   // Disable interrupts in the Universe first
    writel(0x0000FFFF,baseaddr+LINT_STAT); // Clear Any Pending Interrupts

    pci_read_config_dword(uni_pci_dev, PCI_INTERRUPT_LINE, &irq);
    irq &= 0x000000FF;                    // Only a byte in size
    result = request_irq(irq, irq_handler, IRQF_DISABLED, "VMEBus (universe)", NULL);
    if (result) {
      printk("  universe: can't get assigned pci irq vector %02X\n", irq);
    } else {
      writel(0x0000, baseaddr+LINT_MAP0);    // Map all ints to 0
      writel(0x0000, baseaddr+LINT_MAP1);    // Map all ints to 0
    }

  } else {
    printk("  Universe device not found on PCI Bus.\n");
    return 1;
  }

  if ((majorNumber = register_chrdev(majorNumber, "universe", &uni_fops)) < 0) {
    printk("  Error getting Major Number for Drivers\n");
    iounmap(baseaddr);
    return(1);
  } else {
    printk("  Device files major number: %d \n", majorNumber);
    printk("  Tundra Loaded.\n"); 
  }

  register_proc();

  for (x=0;x<MAX_MINOR+1;x++) {
    image_ba[x]  = 0;               // Not defined
    image_ptr[x] = 0;               // Not defined
    opened[x]    = 0;               // Closed
    OkToWrite[x] = 0;               // Not OK
    //mode[x]      = MODE_UNDEFINED;  // Undefined
    image_perform_ioremap[x] = 1;   // Default is to ioremap windows
    image_is_ioremapped[x] = 0;     // Nothing is ioremapped yet 
  }  

  // Setup the DMA Timer
  init_timer(&DMA_timer);
  DMA_timer.function = DMA_status;

  // Enable DMA Interrupts
  writel(0x0700, baseaddr+LINT_EN);
  
  // let's try to find a large chunk of memory defined with the inputs
  if (request_mem_region(reserveFromAddress, sizeToReserve, "universe") == NULL) {
    /* Failed to get memory. */
    printk(" Error reserving memory from %lx to %lx", reserveFromAddress, reserveFromAddress + sizeToReserve - 1);
    memoryIsReserved = 0;
    cleanup_module();
    return -EBUSY;
  }
  memoryIsReserved = 1;

  return 0;
}

//-----------------------------------------------------------------------------
//
//  Public Interface
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Function   : Universe_BaseAddr
// Inputs     : void
// Outputs    : char*
// Description: Returns the Base Address of the Universe
// Remarks    : Used for direct access to the Universe for unsupported functions
// History    : 
//-----------------------------------------------------------------------------
char* Universe_BaseAddr(void) 
{
  return baseaddr;
}

//-----------------------------------------------------------------------------
// Function   : Universe_IRQ
// Inputs     : void
// Outputs    : int
// Description: Returns the PCI IRQ that the Universe is on
// Remarks    : Used mostly for Status and Debugging
// History    : 
//-----------------------------------------------------------------------------
int Universe_IRQ(void) 
{
  return irq;
}

//-----------------------------------------------------------------------------
// Function   : enable_vmeirq
// Inputs     : int irq
// Outputs    : int
// Description: Enable a VME IRQ
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void enable_vmeirq(unsigned int irq)
{
  int enable;

  enable = readl(baseaddr+LINT_EN);
  enable |= vmeirqbit[irq-1];
  writel(enable, baseaddr+LINT_EN);
}

//-----------------------------------------------------------------------------
// Function   : disable_vmeirq
// Inputs     : unsigned int irq
// Outputs    : void
// Description: Disable a VME IRQ
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void disable_vmeirq(unsigned int irq)
{
  int enable;

  enable = readl(baseaddr+LINT_EN);
  enable &= ~vmeirqbit[irq-1];
  writel(enable, baseaddr+LINT_EN);
}

//-----------------------------------------------------------------------------
// Function   : request_vmeirq
// Inputs     : unsigned int irq, TirqHandler handler
// Outputs    : int, 0 if successful
// Description: assign a handler to a vme interrupt
// Remarks    : uses a simple check to see if the interrupt is free, then
//              assigns a handler for the Interrupt
// History    : 
//-----------------------------------------------------------------------------
int request_vmeirq(unsigned int irq, TirqHandler handler)
{
  if ((irq >= 1) && (irq <= 7)) {
    if (vmeirqs[irq-1] == NULL) {
      vmeirqs[irq-1] = handler;
      return 0;
    }
  }
  return -1;                        // IRQ is already assigned or invalid
}

//-----------------------------------------------------------------------------
// Function   : free_vmeirq
// Inputs     : unsigned int irq
// Outputs    : void
// Description: release the vmeirq
// Remarks    : This does not check that a module is freeing an interrupt it
//              owns.  It is up to the programmer to make sure he knows what
//              what he is doing
// History    : 
//-----------------------------------------------------------------------------
void free_vmeirq(unsigned int irq)
{
  if ((irq >= 1) && (irq <= 7))
    vmeirqs[irq-1] = NULL;
}

//-----------------------------------------------------------------------------
// Function   : mapvme
// Inputs     : unsigned int pci  - PCI Address of Image Map (See howto)
//              unsigned int vme  - VME Address of Image (See howto)
//              unsigned int size - Size of Image
//              int image         - Image Number to Use (See howto)
//              int ctl           - VME Address Space (See howto)
// Outputs    : char*
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
char* mapvme(unsigned int pci,unsigned int vme, unsigned int size, 
             int image, int ctl)
{
  char *ptr;
  unsigned int to;

  if ((image >=0) && (image <=3)) {
    if ((pci >= 0xB0000000) && (pci <= 0xE0000000)) {
      to = vme-pci;
      // Setup the Mapping in the Universe
      writel(pci, baseaddr + aBS[image]);
      writel(pci + size, baseaddr + aBD[image]);
      writel(to, baseaddr + aTO[image]);
      writel(ctl | 0x80000000, baseaddr + aCTL[image]);

      // Get the Virtual address of the PCI Address map
      ptr = (char *)ioremap(pci,size);
      if (ptr == NULL) {
        printk("<universe> ioremap failed in mapvme\n");
        return NULL;
      } else {
        return ptr;        // Everything went ok, return the address pointer
      }
    }
  }
  return NULL;
}

//-----------------------------------------------------------------------------
// Function   : unmapvme
// Inputs     : char *ptr, int image
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
void unmapvme(char *ptr, int image)
{
  if ((image >= 0) && (image <=3)) {
    // Clear the Mapping in the Universe
    writel(0x00000000, baseaddr + aCTL[image]);
    writel(0x00000000, baseaddr + aBS[image]);
    writel(0x00000000, baseaddr + aBD[image]);
    writel(0x00000000, baseaddr + aTO[image]);
  }

  // Clear Mapping of PCI Memory
  iounmap(ptr);
}

//-----------------------------------------------------------------------------
// Function   : VME_DMA
// Inputs     : unsigned int pci   - PCI Address of Image Map (See howto)
//              unsigned int vme   - VME Address of Image (See howto)
//              unsigned int count - Size of Image
//              int image          - Image Number to Use (See howto)
//              int ctl            - VME Address Space (See howto)
// Outputs    : void
// Description: 
// Remarks    : This is Like the CMD Packet Version but will do only one transfer
// History    : 
//-----------------------------------------------------------------------------
void VME_DMA(void* pci, void* vme, unsigned int count, int ctl, TDMAcallback cback)
{
#ifdef __DEBUG__
  char buf[256];
  sprintf(buf,"<<universe>> DMA Request of virtpci=0x%08X vme=0x%08X cnt=%i ctl=0x%08X\n",
          pci,vme,count,ctl);
  printk(buf);
#endif

  // Setup the DMA Transfer in the Universe
  writel(ctl, baseaddr + DCTL);
  writel(count, baseaddr + DTBC);
  writel(virt_to_bus(pci), baseaddr + DLA);
  writel((int)vme, baseaddr + DVA);

  // We need to build a timer to timeout DMA access
  DMA_timer.expires  = jiffies + DMATIMEOUT;     
  add_timer(&DMA_timer);

  // Setup CallBack Function
  DMACallBackFunc = cback;

  // Start the Transfer,
  // Interrupt on Stop,Halt,Done,LERR,VERR,Prot Error
  DMAType = DMATYPE_SINGLE;
  writel(0x80216F6F,baseaddr+DGCS);           

  // We are now all setup, so lets return to the calling function so it can
  // sleep or what ever until the DMA is done
}

//-----------------------------------------------------------------------------
// Function   : VME_DMA_LinkedList
// Inputs     : void* CmdPacketList,TDMAcallback cback
// Outputs    : int
// Description: 
// Remarks    : CmdPacketList is a Phys Address
// History    : 
//-----------------------------------------------------------------------------
void VME_DMA_LinkedList(void* CmdPacketList,TDMAcallback cback)
{
  debugptr = CmdPacketList;

  // Setup a DMA Transfer and once the Transfer is complete call cback
  writel(0x08316F00,baseaddr+DGCS);
  writel(0x00000000,baseaddr+DTBC);
  writel((int)CmdPacketList,baseaddr+DCPP);   // Command Packet Pointer

  // We need to build a timer to timeout DMA access
  DMA_timer.expires  = jiffies + DMATIMEOUT;     
  add_timer(&DMA_timer);

  // Setup CallBack Function
  DMACallBackFunc = cback;

  // Start the Transfer,
  // Interrupt on Stop,Halt,Done,LERR,VERR,Prot Error
  DMAType = DMATYPE_LLIST;
  writel(0x88216F6F,baseaddr+DGCS);           

  // We are now all setup, so lets return to the calling function so it can
  // sleep or what ever until the DMA is done
}

//-----------------------------------------------------------------------------
// Function   : VME_Bus_Error
// Inputs     : 
// Outputs    : 
// Description: Returns 1 if a Bus Error is Pending
// Remarks    : 
// History    : 10/30/2000 mjw Started
//-------------------------------------------------------------------------mjw-
int VME_Bus_Error(void)
{
  int pci_csr;

  pci_csr = readl(baseaddr+PCI_CSR);           // Check for Bus Error
  writel(pci_csr,baseaddr+PCI_CSR);            // Clear the Error
  return((pci_csr & 0x08000000) ? 1 : 0);
}

