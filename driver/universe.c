//------------------------------------------------------------------------------	
//title: Tundra Universe PCI-VME Kernel Driver
//version: Linux 1.1
//date: March 1999																																
//designer: Michael Marino, Michael Wyrick 
//programmer: Michael Wyrick, Michael Marino 
//platform: Linux 2.6
//language: GCC 4.x
//module: universe
//
//------------------------------------------------------------------------------	
// more major code fixes, updates.  Basically, this code is brand new, but I'm
// keeping the huggins and wyrick on here because they got me started.  
// All the APIs have been updated to more appropriately deal with a pci
// device.  The DMA is now interrupt driven.  mmap is available and working.
//
// This means we now move to a new version 2.0.b1
//
// When this is fully tested, we will move it to 2.0
//
//------------------------------------------------------------------------------	
// some major code fixes/updates to linux kernel 2.6.	Also added functionality, 
// fixed bugs.	DMA moved to its own device.	Control moved to its own device and
// limited opening.	Added much more error checking, memory reservation.
//
// Michael Marino (Jan 2008)
//
//------------------------------------------------------------------------------	
//	Purpose: Provide a Kernel Driver to Linux for the Universe I and II 
//	 Universe model number universe
//	Docs:								
//	This driver supports both the Universe and Universe II chips	
//------------------------------------------------------------------------------
// RCS:
//IOCTL_SET_VME $Id: universe.c,v 1.5 2001/10/27 03:50:07 jhuggins Exp $
// $Log: universe.c,v $
// Revision 1.5	2001/10/27 03:50:07	jhuggins
// These changes add support for the extra images offered by the Universe II.
// CVS : ----------------------------------------------------------------------
//
// Revision 1.6	2001/10/16 15:16:53	wyrick
// Minor Cleanup of Comments
//
//
//-----------------------------------------------------------------------------

static char Version[] = "2.0.b1 2008Nov05";

#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/dmi.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include "universe.h"
#include "ConcurrentVX40x.h"

//----------------------------------------------------------------------------
// Module parameters 
//----------------------------------------------------------------------------
static unsigned long size_to_reserve		= 0x10000000; // 256 MB reserved
static unsigned long reserve_from_address	= 0xC0000000; // reserve from high mem

module_param(size_to_reserve, long, 0);
module_param(reserve_from_address, long, 0);
MODULE_PARM_DESC(size_to_reserve, "Give the size of pci space to reserve. ");
MODULE_PARM_DESC(reserve_from_address, "Give a starting pci address from which to reserve.");
MODULE_LICENSE("GPL");

//----------------------------------------------------------------------------
// Vars and Defines
//----------------------------------------------------------------------------
//#define UNIVERSE_DEBUG
#define MAX_MINOR	9 // This includes the DMA_MINOR
#define CONTROL_MINOR	8
#define DMA_MINOR	9
#define DMA_TIMEOUT	200 // jiffies
// Including Vendor tags if the linux kernel hasn't defined them
#ifndef PCI_VENDOR_ID_TUNDRA
	#define PCI_VENDOR_ID_TUNDRA 0x10e3
#endif

#ifndef PCI_DEVICE_ID_TUNDRA_CA91C042
	#define PCI_DEVICE_ID_TUNDRA_CA91C042 0x0000
#endif

#define UNIVERSE_PREFIX  "universe: "


//----------------------------------------------------------------------------
// structs 
//----------------------------------------------------------------------------
struct universe_dev {
	int ok_to_write;		/* Can I write to the hardware */
	atomic_t opened;		/* Is it open? */
	int image_perform_ioremap;	/* Should we ioremap? */ 
	int ctl_address;		/* Offset of the ctl for a device */	
	int bs_address;			/* Offset where bs is stored for a device */	
	int bd_address;			/* Offset where bd is stored for a device */	
	int to_address;			/* Offset where to is stored for a device */	
	int vme_address;		/* Base vme address */	
	void *image_ba;			/* Base virtual address for this device */
	void *image_ptr;		/* Pointer to the current place in image */
	unsigned int ctl_register;	/* Control register value for the image */	
	struct semaphore sem;		/* Mutex semaphore */
	struct cdev cdev;		/* Character device structure */
	void *buffer;			/* Buffer for read/write. */
	size_t buffer_length;		/* Length of buffer. */
};

struct universe_dma {
	dma_addr_t dma_transfer_addr;		/* DMA transfer address. */
	size_t dma_size;			/* Size of DMA transfer. */
	enum dma_data_direction dma_direction;	/* Direction of transfer. */
	int dma_align;				/* Alignment of dma. */
	int timeouts;				/* Number of timeouts. */
	struct timer_list dma_timer;		/* Timer for the dma. */
	spinlock_t lock;			/* Spin lock for the dma. */
	struct completion dma_completion;	/* Completion flag. */
};

static struct universe_driv {
	struct pci_dev *pci_dev;	/* PCI device. */
	struct universe_dma dma;	/* DMA xfer info. */
	void *baseaddr;			/* Base address of the universe device. */
	int irq;			/* IRQ of universe driver. */
	int reads;			/* Reads performed on the driver. */
	int writes;			/* Writes performed on the driver. */
	int ioctls;			/* ioctls performed on the driver. */
} universe_driver;

//----------------------------------------------------------------------------
// Prototypes
//----------------------------------------------------------------------------
static int	universe_open(struct inode *, struct file *);
static int	universe_release(struct inode *, struct file *);
static loff_t	universe_llseek(struct file *,loff_t,int);
static ssize_t	universe_read(struct file *,char *, size_t, loff_t *);
static ssize_t	universe_write(struct file *,const char *, size_t, loff_t *);
static int	universe_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static int	universe_mmap(struct file *,struct vm_area_struct *);

static int	universe_init_module(void);
static void	universe_exit_module(void);

static void	universe_vma_open(struct vm_area_struct *);
static void	universe_vma_close(struct vm_area_struct *);

static void	universe_setup_universe_dev(struct universe_dev *, int index); 
static int	universe_check_bus_error(void);
static void 	universe_dma_timeout(unsigned long);
static irqreturn_t universe_irq_handler(int irq, void *dev_id);
static int	universe_ioport_default_permissions(uint16_t);

//----------------------------------------------------------------------------
// Static variables 
//----------------------------------------------------------------------------

static struct file_operations universe_fops = 
{
	.owner	 	= THIS_MODULE,
	.llseek		= universe_llseek,
	.read		= universe_read,
	.write	 	= universe_write,
	.ioctl	 	= universe_ioctl,
	.open		= universe_open,
	.release	= universe_release, 
	.mmap		= universe_mmap 
};

static struct vm_operations_struct universe_vma_remap_ops = 
{
	.open	= universe_vma_open,
	.close	= universe_vma_close
};



static int aCTL[]	= {	LSI0_CTL, LSI1_CTL, LSI2_CTL, LSI3_CTL,
				LSI4_CTL, LSI5_CTL, LSI6_CTL, LSI7_CTL	};
										 
static int aBS[]	= {	LSI0_BS, LSI1_BS, LSI2_BS, LSI3_BS,
				LSI4_BS, LSI5_BS, LSI6_BS, LSI7_BS	}; 
										 
static int aBD[]	= {	LSI0_BD, LSI1_BD, LSI2_BD, LSI3_BD,
				LSI4_BD, LSI5_BD, LSI6_BD, LSI7_BD	}; 

static int aTO[]	= {	LSI0_TO, LSI1_TO, LSI2_TO, LSI3_TO,
				LSI4_TO, LSI5_TO, LSI6_TO, LSI7_TO	}; 


// Hold onto the major Number as it is dynamically given.
// The number shows up in /proc/devices and can be taken from 
// there by a script which generates the dev files. 
static int universe_major	= 0;
static int universe_minor	= 0;
static int universe_nr_devs 	= MAX_MINOR + 1;
static struct universe_dev universe_devices[MAX_MINOR + 1];
static int universe_board_type  = UNIVERSE_BOARD_TYPE_UNKNOWN;
 
static int (*universe_ioport_permissions)(uint16_t portnumber) = NULL;


//-----------------------------------------------------------------------------
//
// Function	: universe_exit_module
//
//-----------------------------------------------------------------------------
static void __exit
universe_exit_module(void)
{					
	int i;
	dev_t dev_number = MKDEV(universe_major, universe_minor);

	// ---------------------------------------------------------
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Exiting universe module\n");
#endif
	release_mem_region(reserve_from_address, size_to_reserve); 
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Released memory region.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Turn off interrupts
	iowrite32(0x0,universe_driver.baseaddr + LINT_EN);
	free_irq(universe_driver.irq,&universe_driver);	
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Turned off irq for Universe.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Free iomapped resources if these are still open.  
	for (i=0;i<universe_nr_devs;i++) {
		if (universe_devices[i].image_ba) {
#ifdef UNIVERSE_DEBUG
			printk( KERN_DEBUG UNIVERSE_PREFIX "Unmapping minor: %d\n", i );
#endif
			iounmap(universe_devices[i].image_ba);
			universe_devices[i].image_ba = 0;
		}
	}		
	// ---------------------------------------------------------
	
	// ---------------------------------------------------------
	// Unmapping the configuration space
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Unmapping baseaddr.\n");
#endif
	if(universe_driver.baseaddr) iounmap(universe_driver.baseaddr);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "baseaddr unmapped.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Releasing the pci memory region.
	pci_release_region(universe_driver.pci_dev, 0);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Release region.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Disabling the devices, this clears the irq routing.
	pci_disable_device(universe_driver.pci_dev);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Disabled device.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// We are done with the pci device.
	pci_dev_put(universe_driver.pci_dev);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Returned pci_dev.\n");
#endif
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Remove registration of char drivers.
	unregister_chrdev_region(dev_number, universe_nr_devs);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Released char driver region.\n");
#endif
	// ---------------------------------------------------------
}
module_exit(universe_exit_module);

//----------------------------------------------------------------------------
//
//	universe_init_module()
//
//----------------------------------------------------------------------------
static int __init
universe_init_module(void)
{
	int result,i;
	unsigned int temp;
	dev_t dev_number;

	printk(KERN_INFO UNIVERSE_PREFIX "Tundra Universe II PCI-VME Driver %s\n", Version);
	printk(KERN_INFO UNIVERSE_PREFIX "Copyright 2008, Michael G. Marino\n");

	// ---------------------------------------------------------
	// Getting the device major and minor numbers
	result = alloc_chrdev_region(&dev_number, 0, universe_nr_devs,"universe");
	universe_major = MAJOR(dev_number);
	if ( result < 0 ) { 
		printk(KERN_ERR UNIVERSE_PREFIX "Error getting major: %d\n", universe_major);
		return result;
	} 

	printk(KERN_INFO UNIVERSE_PREFIX "Device files major number: %d \n", universe_major);
	// device major and minor numbers found
	// ---------------------------------------------------------

	// zero out the driver struct
	memset(&universe_driver, 0, sizeof(universe_driver));

	// ---------------------------------------------------------
	// Searching for the device and setting up.
	if (!(universe_driver.pci_dev = pci_get_device(	PCI_VENDOR_ID_TUNDRA,
						PCI_DEVICE_ID_TUNDRA_CA91C042, 
						NULL))) {
		printk(KERN_ERR UNIVERSE_PREFIX "Universe device not found on PCI Bus.\n");
		result = -ENXIO;
		goto err_unreg_chrdrv_region;
	}
	// The device was found on the PCI bus. 
	printk(KERN_INFO UNIVERSE_PREFIX "Universe device found.\n");
	if (pci_enable_device(universe_driver.pci_dev)) {
		// Could not enable the device, something is wrong.
		printk(KERN_ERR UNIVERSE_PREFIX "Failed to enable Universe device.\n");
		result = -ENODEV;
		goto err_devput;
	}
	printk(KERN_INFO UNIVERSE_PREFIX "Universe device enabled.\n");
	if (pci_request_region(universe_driver.pci_dev, 0, "universe")) {
		printk(KERN_ERR UNIVERSE_PREFIX "Request pci region failed.\n");
		result = -EBUSY;
		goto err_disable;
	}

	pci_set_master(universe_driver.pci_dev);
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Setup Universe Config Space
	// This is a 4k wide memory area that need to be mapped into the kernel
	// virtual memory space so we can access it.
	universe_driver.baseaddr = (void *)ioremap(pci_resource_start(universe_driver.pci_dev, 0), 
						   pci_resource_len(universe_driver.pci_dev, 0));
	if (!universe_driver.baseaddr) {
		printk(KERN_ERR UNIVERSE_PREFIX "ioremap failed to map Universe to Kernel Space.\r");
		result = -ENOMEM;
		goto err_release_region;
	}
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Check to see if the Mapping Worked out
#ifdef UNIVERSE_DEBUG
	printk(KERN_DEBUG UNIVERSE_PREFIX "baseaddr read in as: %02x\n", (unsigned int)universe_driver.baseaddr);
#endif
	temp = ioread32(universe_driver.baseaddr);
	if (temp != PCI_VENDOR_ID_TUNDRA) {
		printk(KERN_ERR UNIVERSE_PREFIX "Universe Chip Failed to Return PCI_ID in Memory Map.\n");
		result = -ENXIO;
		goto err_unmap;
	}
	// ---------------------------------------------------------
	// ---------------------------------------------------------
	// Setting defaults, these can be reset by ioctl
	// Set VME Bus Time-out: 16 mus
	// Arbitration Mode: Priority, Timeout, minimum (<16 mus, >8mus)
	// Set Bus Master	
	iowrite32(0x15020000,universe_driver.baseaddr+MISC_CTL);		 
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Setting up interrupts 
	// Disable interrupts in the Universe first
	iowrite32(0x00000000,universe_driver.baseaddr + LINT_EN);	 
	// Clear Any Pending Interrupts
	iowrite32(0x0000FFFF,universe_driver.baseaddr + LINT_STAT); 
	// Grabbing the interrupt request
	universe_driver.irq = universe_driver.pci_dev->irq; 
	// We have to share or at least be able to handle sharing since the PCI bus automatically share the lines.
	result = request_irq(universe_driver.irq, universe_irq_handler, IRQF_SHARED, "universe", &universe_driver);
	if (result) {
		printk(KERN_ERR UNIVERSE_PREFIX "Can't get assigned irq %d\n", universe_driver.irq);
		result = -EBUSY;
		goto err_unmap;
	} 
	printk(KERN_INFO UNIVERSE_PREFIX "Assigned irq %d\n", universe_driver.irq);

	// The Universe II manual discusses 8 possible LINT lines.  These
	// are commonly referred to as LNK[A-H] lines in the PCI literature.
	// The PCI bus has at least 4 lines, but additional hardware
	// may have more lines.  Also, it is important to understand that
	// the 8 lines coming from the Universe II are not fully PCI compliant.
	// From the manual:
	//
	// The Universe II provides eight local bus interrupts, only one of which has drive strength that is fully
	// PCI compliant. If any of the other seven interrupts are to be used as interrupt outputs to the local bus
	// (all eight may be defined as either input or output), an analysis must be done on the design to determine
	// whether the 4 mA of drive that the Universe II provides on these lines is sufficient for the design. If
	// more drive is required, the lines may simply be buffered.
	//
	// We assume that the fully compliant line is LINT0, or LNKA.  This should be true for most setups.
	// Therefore, we map *all* interrupts to this line.  
	//
	iowrite32(0x00000000, universe_driver.baseaddr+LINT_MAP0);  
	iowrite32(0x00000000, universe_driver.baseaddr+LINT_MAP1);  
	// We don't enable any interrupts from the Universe yet, we wait until the DMA is opened.
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Finding a large chunk of memory defined with the inputs
	if (request_mem_region(reserve_from_address, size_to_reserve, "universe") == NULL) {
		/* Failed to get memory. */
		printk(	KERN_WARNING UNIVERSE_PREFIX " Error reserving memory from 0x%lx to 0x%lx\n", 
			reserve_from_address, reserve_from_address + size_to_reserve - 1);
		result = -EBUSY;
		goto err_free_irq;
	}
	// ---------------------------------------------------------

	// ---------------------------------------------------------
	// Setting up the universe devices
	for(i=0;i<universe_nr_devs;i++) universe_setup_universe_dev(&universe_devices[i], i);	
	// ---------------------------------------------------------

	// Success 
	return 0; 

err_free_irq:
	free_irq(universe_driver.irq, &universe_driver);
err_unmap:
	iounmap(universe_driver.baseaddr);
err_release_region:
	pci_release_region(universe_driver.pci_dev, 0);
err_disable:
	pci_disable_device(universe_driver.pci_dev);
err_devput:
	pci_dev_put(universe_driver.pci_dev);
err_unreg_chrdrv_region:
	unregister_chrdev_region(dev_number, universe_nr_devs);
	return result;
}
module_init(universe_init_module);

//-----------------------------------------------------------------------------
//
// universe_ioport_default_permissions()
//
//-----------------------------------------------------------------------------
static int universe_ioport_default_permissions(uint16_t port)
{
	/* function returns -1 indicating failure. */
	return -1;
}
//-----------------------------------------------------------------------------
//
// universe_setup_universe_dev()
//
//-----------------------------------------------------------------------------
static void universe_setup_universe_dev(struct universe_dev *dev, int index)
{
	int err;
	int devno = MKDEV(universe_major, universe_minor + index);

	// First zero everything
	memset(dev, 0, sizeof(struct universe_dev));
	
	// Setup the file operations. 
	cdev_init(&(dev->cdev), &universe_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &universe_fops;
	err = cdev_add(&(dev->cdev), devno, 1);
	if (err) {
		printk(	KERN_ERR UNIVERSE_PREFIX "Error (%d) addding universe dev: %d\n",
			err, index );
	}

	// Initializing semaphore
	init_MUTEX(&dev->sem);
	// Default is to ioremap so set that flag.
	dev->image_perform_ioremap = 1;
	// we only allow one process at a time to access
	// each device.  
	dev->opened.counter = 1;//ATOMIC_INIT(1);

	// The following only deals with the *normal* minors
	if (index != CONTROL_MINOR && index != DMA_MINOR) {
		dev->ctl_address = aCTL[index];
		dev->bs_address = aBS[index];
		dev->bd_address = aBD[index];
		dev->to_address = aTO[index];
		iowrite32(0x0, universe_driver.baseaddr + dev->ctl_address);
		iowrite32(0x0, universe_driver.baseaddr + dev->bs_address);
		iowrite32(0x0, universe_driver.baseaddr + dev->bd_address);
		iowrite32(0x0, universe_driver.baseaddr + dev->to_address);
	} else if (index == DMA_MINOR) {
		init_timer(&universe_driver.dma.dma_timer);
	}
}

//----------------------------------------------------------------------------
//
//	universe_open()
//
//----------------------------------------------------------------------------
static int universe_open(struct inode *inode,struct file *file)
{
	struct universe_dev *dev; 
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);	
	dev = container_of(inode->i_cdev, struct universe_dev, cdev);

	// We only allow a device to be open once.
	if (!atomic_dec_and_test(&dev->opened)) {
		/* Someone else has opened, this is not allowed. */
		atomic_inc(&dev->opened);
		return -EBUSY;
	} 
	file->private_data = dev;
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Opening universe file: %d\n", minor);
#endif
	if (minor == CONTROL_MINOR) {
		universe_ioport_permissions = universe_ioport_default_permissions;
#ifndef CONFIG_DMI
		printk( KERN_ERR UNIVERSE_PREFIX "ioctl error: kernel not compiled with CONFIG_DMI.\n"); 
		printk( KERN_ERR UNIVERSE_PREFIX "ioctl error: Driver unable to determine SBC type.\n"); 
		universe_board_type = UNIVERSE_BOARD_TYPE_UNKNOWN;
#else
		if ( dmi_name_in_vendors("Concurrent Technologies") ) {
			universe_board_type = UNIVERSE_BOARD_TYPE_CCT;	
			universe_ioport_permissions = concurrent_ioports_permissions;
		} else {	
			universe_board_type = UNIVERSE_BOARD_TYPE_UNKNOWN;	
		}
#endif 
		return 0;
	}
	if (minor == DMA_MINOR) {
		/* get free pages for the dma device. */
		/* We request a lot, but we should be ok. */
		dev->buffer = (void *)__get_free_pages(GFP_KERNEL, MAX_ORDER-1);
		if (!dev->buffer) {
			atomic_inc(&dev->opened);
			printk( KERN_ERR UNIVERSE_PREFIX "Unable to get free pages for DMA.\n");
			return -EFAULT;
		}
		dev->buffer_length = PAGE_SIZE << (MAX_ORDER-1);
		universe_driver.dma.dma_timer.data = 0;
		universe_driver.dma.dma_timer.function = universe_dma_timeout;
		spin_lock_init(&universe_driver.dma.lock);
		init_completion(&universe_driver.dma.dma_completion);
		// FixME: Currently, the interrupt handler only handles DMA interrupts.
		// Enabling DMA interrupts, this can be reset by ioctl 
		iowrite32(0x00000100,universe_driver.baseaddr + LINT_EN);	 
	} else {
		/* malloc some area for the other minors, just 1 page */
		/* Zero it, not necessary but could save us some trouble.*/
		dev->buffer = (void *)get_zeroed_page(GFP_KERNEL);
		if (!dev->buffer) {
			atomic_inc(&dev->opened);
			printk( KERN_ERR UNIVERSE_PREFIX "Unable to get free pages for device.\n");
			return -EFAULT;
		}
		dev->buffer_length = PAGE_SIZE;
	}
	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_release()
//
//----------------------------------------------------------------------------
static int universe_release(struct inode *inode,struct file *file)
{
	struct universe_dev *dev = file->private_data; 
#ifdef UNIVERSE_DEBUG
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);	
	printk( KERN_DEBUG UNIVERSE_PREFIX "Closing universe file: %d\n", minor);
#endif

	dev->ok_to_write = 0;
	/* Disable the image, reset the registers */
	if (dev->ctl_address) iowrite32(0x0, universe_driver.baseaddr + dev->ctl_address);
	if (dev->bs_address) iowrite32(0x0, universe_driver.baseaddr + dev->bs_address);
	if (dev->bd_address) iowrite32(0x0, universe_driver.baseaddr + dev->bd_address);
	if (dev->to_address) iowrite32(0x0, universe_driver.baseaddr + dev->to_address);

	if (dev->image_ba) {
		/* Get rid of the ioremapped memory. */
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Unmapping file: %d, at address: 0x%lx\n", minor, (unsigned long) dev->image_ba);
#endif
		iounmap(dev->image_ba);
		dev->image_ba = 0;
	}
	if (dev->buffer) {
		/* Free the pages we made. */
		free_pages((unsigned long) dev->buffer, get_order(dev->buffer_length));
		dev->buffer = 0;
		dev->buffer_length = 0;
	}
	/* releasing the device. */
	atomic_inc(&dev->opened);
	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_lseek()
//
//----------------------------------------------------------------------------
static long long universe_llseek(struct file *file,loff_t offset,int whence)
{
	/* So we don't care what the minor number is.  Remember that dev *
	 * was completely zeroed out and we only set the image_ba for    *
	 * the PCI target devices so that image_ba for a CONTROL         *
	 * or DMA minor is 0.  DMA minor doesn't use this anyways.       */

	struct universe_dev *dev = file->private_data; 

	if (whence == SEEK_SET) dev->image_ptr = dev->image_ba + offset;
	else if (whence == SEEK_CUR) dev->image_ptr += offset;
	else return -EPERM;	

	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_read()
//
//----------------------------------------------------------------------------
static ssize_t universe_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	unsigned int v,numt,remain,tmp,done_reading;
	int 	val	= 0;

	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);	
	struct universe_dev *dev = file->private_data; 

	universe_driver.reads++;
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Initiating a read.\n");
#endif
	if (minor == CONTROL_MINOR) {
		v = ioread32(universe_driver.baseaddr + (unsigned long)dev->image_ptr);		
		if (copy_to_user(buf,&v,sizeof(v))) return -EFAULT;
		return sizeof(v);
	} 
	if (dev->ok_to_write != 1) return -EPERM;
	if (minor != DMA_MINOR) goto readout_minor;

#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "DMA Transfer requested. \n");
#endif
	// Check the semaphore_lock
	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
	}
	// Check to see if the DMA is running, this should never happen
	// If it is, kick back to the calling function, let it handle.
	// This should never happen!
	val = ioread32(universe_driver.baseaddr + DGCS);
	if (val & 0x00008000) {
		printk( KERN_WARNING UNIVERSE_PREFIX "DMA is busy, but lock was obtained!\n");
		up(&dev->sem);
		return -EBUSY;
	}
        
	// VME Address ( and PCI address must be 8-byte aligned with each other)							
	universe_driver.dma.dma_align = dev->vme_address & 0x7;	
	universe_driver.dma.dma_size = count;
	universe_driver.dma.dma_direction = DMA_FROM_DEVICE;
        if (universe_driver.dma.dma_size + 
	    universe_driver.dma.dma_align > dev->buffer_length) {
		// We've requested the max size of contiguous pages
		// but we still can't manage this size of transfer.
		// This means the calling function will have to
		// break up the dma into smaller pieces.
#ifdef UNIVERSE_DEBUG
		printk(KERN_WARNING UNIVERSE_PREFIX "Requested DMA transfer is larger than 0x%lx\n", 
			(long unsigned int) dev->buffer_length);
#endif
		up(&dev->sem);
		return -ENXIO;
	}				 
        
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "DMA Transfer, getting map. \n");
#endif
	// PCI Address
	universe_driver.dma.dma_transfer_addr = dma_map_single(&universe_driver.pci_dev->dev, dev->buffer, 
						universe_driver.dma.dma_size + universe_driver.dma.dma_align,
						universe_driver.dma.dma_direction);
	// Setup DMA regs
	// setup DMA for a *read*
	// First we setup the interrupts, and clear the other statuses.
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "DMA Transfer, setting registers. \n");
#endif
	iowrite32(0x00006F6F,universe_driver.baseaddr+DGCS);
	dev->ctl_register &= 0x7FFFFFFF; // Sets L2V bit for a read
	iowrite32(dev->ctl_register,universe_driver.baseaddr + DCTL); // Setup Control Reg
	iowrite32(count,universe_driver.baseaddr + DTBC); // Count			
	iowrite32(universe_driver.dma.dma_transfer_addr+universe_driver.dma.dma_align,
			universe_driver.baseaddr + DLA); // PCI Address
	iowrite32(dev->vme_address,universe_driver.baseaddr+DVA);	// VME Address
        
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "DMA Transfer, setting up timer. \n");
#endif
	universe_driver.dma.dma_timer.expires = jiffies + DMA_TIMEOUT;
	add_timer(&universe_driver.dma.dma_timer);
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "DMA Transfer, GO. \n");
#endif
	iowrite32(0x80006F6F,universe_driver.baseaddr+DGCS);	 // GO
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Waiting for completion. \n");
#endif
	if(wait_for_completion_interruptible(&universe_driver.dma.dma_completion)) {
		// Huh, this is a problem, we will lose the data.
		printk(KERN_WARNING UNIVERSE_PREFIX "Requested DMA transfer interrupted.\n");
		up(&dev->sem);
		return -ERESTARTSYS;
	}
	// OK, so we waited and it apparently completed.  Check for errors.
	val = ioread32(universe_driver.baseaddr+DGCS);
	if (!(val & 0x00000800)) {
#ifdef UNIVERSE_DEBUG
		printk(KERN_WARNING UNIVERSE_PREFIX "Requested DMA read transfer failed.\n");
#endif
		up(&dev->sem);
		return 0;
	}
	if(copy_to_user(buf, dev->buffer + universe_driver.dma.dma_align, 
			universe_driver.dma.dma_size)) {
		up(&dev->sem);
		return -EFAULT; 
	}
	up(&dev->sem);
	return count;

readout_minor:
	// Normal minor
	remain = count;
	while (remain > 0) {
		numt = (count > dev->buffer_length) ? dev->buffer_length :
					    	      count;
		done_reading = 0;	
		/* Copying from i/o memory.  */
		// We use coupled read/writes exclusively in the image programming. 
		// When the Universe II encounters a BERR, it generates Target-Abort
		// on the PCI bus and sets the S_TA bit in the PCI_CSR register.
		// This should happen immediately so that the following function
		// will exit immediately following a BERR.  We always check if there
		// was an error on the bus.
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Reading from: 0x%lx\n", (unsigned long) dev->image_ptr);
#endif
		while( numt >= sizeof(u32) ) {
			// long copy  
			*(u32 *)(dev->buffer + done_reading) = ioread32(dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_reading += sizeof(u32);
			dev->image_ptr += sizeof(u32);
			numt -= sizeof(u32);
		}
		while( numt >= sizeof(u16) ) {
			// word copy  
			*(u16 *)(dev->buffer + done_reading) = ioread16(dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_reading += sizeof(u16);
			dev->image_ptr += sizeof(u16);
			numt -= sizeof(u16);
		}
		while( numt >= sizeof(u8) ) {
			// byte copy  
			*(u8 *)(dev->buffer + done_reading)= ioread8(dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_reading += sizeof(u8);
			dev->image_ptr += sizeof(u8);
			numt -= sizeof(u8);
		}
		if(copy_to_user(buf, dev->buffer, done_reading)) return -EFAULT;
		remain -= done_reading;
	}
	return count;
	
}

//----------------------------------------------------------------------------
//
//	universe_write()
//
//----------------------------------------------------------------------------
static ssize_t universe_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	unsigned int numt,remain,tmp,done_writing;
	unsigned int	vl;
	int 	val 	= 0;
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct universe_dev *dev = file->private_data; 

	universe_driver.writes++;

	if (minor == CONTROL_MINOR) {
		if (copy_from_user(&vl,buf,sizeof(vl))) return -EFAULT;
		iowrite32(vl,universe_driver.baseaddr + (unsigned long)dev->image_ptr);
		return sizeof(vl);
	} 

	if (dev->ok_to_write != 1) return -EPERM;
	if (minor != DMA_MINOR) goto write_minor;
	// deal with the dma 
	// ------------------------------------------------------------------
	//
	// ------------------------------------------------------------------			
	// Wait for DMA to finish, This needs to be changed
	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
	}
	// We now have the semaphore lock.
	//start_dma:
	// Check to see if the DMA is running, this should never happen
	// If it is, kick back to the calling function, let it handle.
	// This should never happen!
	val = ioread32(universe_driver.baseaddr + DGCS);
	if (val & 0x00008000) {
		printk( KERN_WARNING UNIVERSE_PREFIX "DMA is busy, but lock was obtained!\n");
		up(&dev->sem);
		return -EBUSY;
	}
        
	// VME Address ( and PCI address must be 8-byte aligned with each other)							
	universe_driver.dma.dma_align = dev->vme_address & 0x7;	
	universe_driver.dma.dma_size = count;
	universe_driver.dma.dma_direction = DMA_TO_DEVICE;
        if (universe_driver.dma.dma_size + 
	    universe_driver.dma.dma_align > dev->buffer_length) {
		// We've requested the max size of contiguous pages
		// but we still can't manage this size of transfer.
		// This means the calling function will have to
		// break up the dma into smaller pieces.
#ifdef UNIVERSE_DEBUG
		printk(KERN_WARNING UNIVERSE_PREFIX "Requested DMA transfer is larger than 0x%lx\n", 
			(long unsigned int)dev->buffer_length);
#endif
		up(&dev->sem);
		return -ENXIO;
	}				 
        
	// PCI Address
	universe_driver.dma.dma_transfer_addr = dma_map_single(&universe_driver.pci_dev->dev, dev->buffer, 
						universe_driver.dma.dma_size + universe_driver.dma.dma_align,
						universe_driver.dma.dma_direction);
	// Setup DMA regs
	dev->ctl_register |= 0x80000000; // Sets L2V bit for a write
	iowrite32(dev->ctl_register,universe_driver.baseaddr + DCTL); // Setup Control Reg
	iowrite32(count,universe_driver.baseaddr + DTBC); // Count			
	iowrite32(universe_driver.dma.dma_transfer_addr+universe_driver.dma.dma_align,universe_driver.baseaddr + DLA); // PCI Address
	iowrite32(dev->vme_address,universe_driver.baseaddr+DVA);	// VME Address
        
	universe_driver.dma.dma_timer.expires = jiffies + DMA_TIMEOUT;
	add_timer(&universe_driver.dma.dma_timer);
	// Start DMA
	iowrite32(0x80006F6F,universe_driver.baseaddr+DGCS);	 // GO
	// Now, the write also waits for completion.  Fix in the future?
	if(wait_for_completion_interruptible(&universe_driver.dma.dma_completion)) {
		// Huh, this is a problem, we will lose the data.
		printk(KERN_ERR UNIVERSE_PREFIX "Requested DMA transfer interrupted.\n");
		up(&dev->sem);
		return -ERESTARTSYS;
	}
	val = ioread32(universe_driver.baseaddr+DGCS);
	if (!(val & 0x00000800)) {
#ifdef UNIVERSE_DEBUG
		printk(KERN_ERR UNIVERSE_PREFIX "Requested DMA write transfer failed.\n");
#endif
		up(&dev->sem);
		return 0;
	}
	up(&dev->sem);
	return count;

write_minor:
	// dealing with an image minor
	remain = count;
	while (remain > 0) {
		/* Copying to i/o memory.  */
		// We use coupled read/writes exclusively in the image programming. 
		// When the Universe II encounters a BERR, it generates Target-Abort
		// on the PCI bus and sets the S_TA bit in the PCI_CSR register.
		// This should happen immediately so that the following function
		// will exit immediately following a BERR.  We always check if there
		// was an error on the bus.
		numt = (count > dev->buffer_length) ? dev->buffer_length :
					    	      count;
		if(copy_from_user(dev->buffer, buf, numt)) return -EFAULT;
		done_writing = 0;	
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Writing to: 0x%lx\n", (unsigned long) dev->image_ptr);
#endif
		while( numt >= sizeof(u32) ) {
			// long copy
			iowrite32(*(u32 *)(dev->buffer + done_writing), dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_writing += sizeof(u32);
			dev->image_ptr += sizeof(u32);
			numt -= sizeof(u32);
		}
		while( numt >= sizeof(u16) ) {
			// word copy
			iowrite16(*(u16 *)(dev->buffer + done_writing), dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_writing += sizeof(u16);
			dev->image_ptr += sizeof(u16);
			numt -= sizeof(u16);
		}
		while( numt >= sizeof(u8) ) {
			// byte copy 
			iowrite8(*(u8 *)(dev->buffer + done_writing), dev->image_ptr); 
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			done_writing += sizeof(u8);
			dev->image_ptr += sizeof(u8);
			numt -= sizeof(u8);
		}
		remain -= done_writing;
		dev->image_ptr += done_writing;
	}
	return count;
}

//----------------------------------------------------------------------------
//
//	universe_ioctl()
//
//----------------------------------------------------------------------------
//	For a normal image minor, initialize it by (in this order):
//		UNIVERSE_IOCSET_CTL: Set control register (address space, data space, etc.)
//		UNIVERSE_IOCSET_IOREMAP: set to ioremap pci mem to kernel memory.
//		UNIVERSE_IOCSET_BS:	Set base (offset) of window.	The driver will automatically
//			determine where this is within it's allowed PCI space.
//		UNIVERSE_IOCSET_BD:	Set bound (size) of window.
//		UNIVERSE_IOCSET_VME: Use this to set the desired VME address the base
//			of the window will point to.
//
//	For a DMA minor, initialize it by (in this order):
//		UNIVERSE_IOCSET_CTL: Set control register (address space, data space, etc.)
//		UNIVERSE_IOCSET_VME: Use this to set the desired VME address from which the 
//			DMA will start. 
//
//	For a control minor, the following functions are allowed:
//		UNIVERSE_IOCSET_HW_BYTESWAP: set hardware byteswap
//		UNIVERSE_IOCGET_MEM_SIZE:		get size of pci memory set aside by the driver
//		default: write ard to the register given by the cmd number

static int universe_ioctl(struct inode *inode,struct file *file,unsigned int cmd, unsigned long arg)
{
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned long sizetomap = 0, bs = 0;
	struct universe_ioport_ioctl ioport_str;
	int err  = 0;
	struct universe_dev *dev;

	// Make sure we are getting the right commands
	if (_IOC_TYPE(cmd) != UNIVERSE_MAGIC_NUMBER) return - ENOTTY;
	if (_IOC_NR(cmd) > UNIVERSE_IOC_MAXNR) return - ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}
	if (err) return -EFAULT;

	dev = file->private_data; 
	universe_driver.ioctls++;
	switch (cmd) {

	case UNIVERSE_IOCSET_CTL:
		if (minor == CONTROL_MINOR) return -EPERM;
		if (minor == DMA_MINOR) {
			// Lets compute and save the DMA CTL Register
			dev->ctl_register = arg & 0x80C7F380; //crazy bitmask, a bunch of reserved areas
			dev->ok_to_write = 1;
		} else {
			arg &= 0xFFFFFFFE; // *always* use memory space, not i/o 
#ifdef UNIVERSE_DEBUG
			printk( KERN_DEBUG UNIVERSE_PREFIX "Writing 0x%lx at 0x%lx\n", arg, (unsigned long) universe_driver.baseaddr + dev->ctl_address );
#endif
			iowrite32(arg,universe_driver.baseaddr + dev->ctl_address);
			arg = ioread32(universe_driver.baseaddr + dev->ctl_address);
#ifdef UNIVERSE_DEBUG
			printk( KERN_DEBUG UNIVERSE_PREFIX "Readback: 0x%lx\n", arg);
#endif
		}
		break;

	case UNIVERSE_IOCSET_BS:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		if (arg >= size_to_reserve ) {
		/* we want to make sure that the base offset doesn't overshoot our reserved memory. */
			return -EFAULT;
		}
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Setting BS: 0x%lx\n", (unsigned long)(arg + reserve_from_address));
#endif
		iowrite32(arg+reserve_from_address,universe_driver.baseaddr+dev->bs_address);
		arg = ioread32(universe_driver.baseaddr+dev->bs_address);
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Readback: 0x%lx\n", arg);
#endif
		break;

	case UNIVERSE_IOCSET_BD:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "reading at: 0x%lx\n", (unsigned long)(universe_driver.baseaddr + dev->bs_address));
#endif
		bs = ioread32(universe_driver.baseaddr + dev->bs_address);
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "bs read in as: 0x%lx\n", bs);
		printk( KERN_DEBUG UNIVERSE_PREFIX "arg read in as: 0x%lx\n", arg);
#endif
		if (arg+bs <= size_to_reserve+reserve_from_address) {
			/* we want to make sure that the bound doesn't overshoot our reserved memory. */
			
#ifdef UNIVERSE_DEBUG
			printk( KERN_DEBUG UNIVERSE_PREFIX "Writing 0x%lx at 0x%lx\n", arg+bs, (unsigned long) universe_driver.baseaddr + dev->bd_address );
#endif
			iowrite32(arg+bs,universe_driver.baseaddr+dev->bd_address);
			if (dev->image_ba) {
#ifdef UNIVERSE_DEBUG
				printk( KERN_DEBUG UNIVERSE_PREFIX "Unmapping minor: %d\n", minor);
#endif
				iounmap(dev->image_ba);
				dev->image_ba = 0;
			}
	
			// This uses the BS Register to Find the size of the Image Mapping		
	
			if (dev->image_perform_ioremap == 1) {
				sizetomap = arg; 
#ifdef UNIVERSE_DEBUG
				printk( KERN_DEBUG UNIVERSE_PREFIX "Mapping minor: %d\n", minor);
#endif
				dev->image_ba = (char *)ioremap(bs, sizetomap);
#ifdef UNIVERSE_DEBUG
				printk( KERN_DEBUG UNIVERSE_PREFIX "Mapping minor to: 0x%lx, at 0x%lx\n", (unsigned long) dev->image_ba, bs);
#endif
				if (!dev->image_ba) {
					dev->ok_to_write = 0;		 
					printk( KERN_ERR UNIVERSE_PREFIX "Error in ioremap, address: 0x%lx, size: 0x%lx\n", 
						arg + reserve_from_address, sizetomap);
					return -EFAULT;
				}
				dev->image_ptr = dev->image_ba;
				dev->ok_to_write = 1;
			} else {
				/* This disallows the use of the read/write functions of the device. */
				/* If a device is not ioremmapped it must not be read out with these 
				 * functions. */
				dev->ok_to_write = 0;
			}
	
	
		} else return -EFAULT;
		break;	

	case UNIVERSE_IOCSET_VME:
		/* We get a VME address and insert the offset. */
		/* BS needs to be set before this. */
		if (minor == CONTROL_MINOR) return -EPERM;
		dev->vme_address = arg;
		if (minor != DMA_MINOR) {
			// Calculate the VME Address
			bs = ioread32(universe_driver.baseaddr + dev->bs_address);
			iowrite32(arg-bs,universe_driver.baseaddr + dev->to_address);
			arg = ioread32(universe_driver.baseaddr + dev->to_address);
#ifdef UNIVERSE_DEBUG
			printk( KERN_DEBUG UNIVERSE_PREFIX "Readback VME to address: 0x%lx\n", arg);
#endif
		}
		break;	

	case UNIVERSE_IOCSET_IOREMAP:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		dev->image_perform_ioremap = (arg == 0) ? 0 : 1;
		break;

	case UNIVERSE_IOCIO_PORT_READ:
		/* This driver essentially circumvents ioperm by allowing processes to 
		 * read and write io ports without being the superuser.  This is necessary
		 * to get at board registers which affect VME transactions. */
		if (minor != CONTROL_MINOR) return -EPERM;
		if (__copy_from_user(&ioport_str, (void __user *)arg, sizeof(ioport_str)) !=0 ) {
			return -EIO;
		}
		/* Performing a permissions check. */
		if ( (*universe_ioport_permissions)(ioport_str.address) < 0 ) return -EPERM;
		ioport_str.value = inb(ioport_str.address); 
		if (__copy_to_user((void __user *) arg, &ioport_str, sizeof(ioport_str))!= 0 ) {
			return -EIO;
		}
		break;
		//readBack = (0x38 & arg);
		//outb(readBack, VX407_CSR0);	
		//if ((0x38 & arg) != (readBack = (0x38 & inb(VX407_CSR0)))) { 
		//	printk(	KERN_WARNING UNIVERSE_PREFIX " Hardware swap not set at address 0x%x. Set: 0x%x, Readback: 0x%x\n", 
		//		VX407_CSR0, (unsigned int)(0x38 & arg), readBack);
		//	return -EIO;
		//}

	case UNIVERSE_IOCIO_PORT_WRITE:
		/* This driver essentially circumvents ioperm by allowing processes to 
		 * read and write io ports without being the superuser.  This is necessary
		 * to get at board registers which affect VME transactions. */
		if (minor != CONTROL_MINOR) return -EPERM;
		if (__copy_from_user(&ioport_str, (void __user *)arg, sizeof(ioport_str)) !=0 ) {
			return -EIO;
		}
		/* Performing a permissions check. */
		if ( (*universe_ioport_permissions)(ioport_str.address) < 0 ) return -EPERM;
		outb(ioport_str.value, ioport_str.address); 
		break;

	case UNIVERSE_IOCGET_MEM_SIZE:
		if (minor != CONTROL_MINOR) return -EPERM;
		/* only returning the size.  The mem location is handled internally. */
		/* It is the responsibility of the calling API to appropriately 
		 * setup how the memory exists in the chunk (i.e. by setting up offsets).*/
		return __put_user(size_to_reserve, (unsigned long __user *)arg );

	case UNIVERSE_IOCGET_BOARD_TYPE:
		if (minor != CONTROL_MINOR) return -EPERM;
		/* only returning the board type if we know. */
		/* It is the responsibility of the calling API to appropriately 
		 * to handle this board type correctly. */

		return __put_user(universe_board_type, (unsigned int __user *)arg );


	case UNIVERSE_IOCCHECK_BUS_ERROR:
		return universe_check_bus_error();

	}
	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_mmap()
//
//----------------------------------------------------------------------------
static int universe_mmap(struct file *file,struct vm_area_struct *vma)
{
	unsigned long bs, bd;
	unsigned long offset, physical_address, virtual_size, physical_size;
	struct universe_dev *dev = file->private_data; 

	bs = ioread32(universe_driver.baseaddr + dev->bs_address);
	bd = ioread32(universe_driver.baseaddr + dev->bd_address);

	offset = vma->vm_pgoff << PAGE_SHIFT;
	physical_address = bs + offset;
	virtual_size = vma->vm_end - vma->vm_start;

	if ((bd - bs) < offset) {
		/* offset is too large! */
		printk(KERN_WARNING UNIVERSE_PREFIX "  mmap failure:offset (0x%lx) is too large.\n", offset);
		return -EFAULT;
	}
	physical_size = (bd - bs) - offset;	
	if (virtual_size > physical_size) {
		/* Range spans too much */
		printk(KERN_WARNING UNIVERSE_PREFIX "  mmap failure:range (0x%lx) spans too much, must be below (0x%lx).\n", virtual_size, physical_size);
		return -EINVAL;
	}

	// we do not want to have this area swapped out, lock it
	vma->vm_flags |= VM_RESERVED;
	/* The cpu CANNOT cache these pages.  Also, this will keep a core dump from reading this mem. */
	vma->vm_flags |= VM_IO;
	/* OK, now remap. */
#ifdef UNIVERSE_DEBUG
	printk(KERN_DEBUG UNIVERSE_PREFIX "  Mapping physical address (0x%lx), size (0x%lx) to user space\n", 
		physical_address, virtual_size);
#endif
	if (remap_pfn_range( 	vma, 
				vma->vm_start, 
				physical_address >> PAGE_SHIFT, 
				virtual_size, 
				vma->vm_page_prot)) {
		/* Something failed, maybe try again? */
		return -EAGAIN;
	}
	vma->vm_ops = &universe_vma_remap_ops;
	universe_vma_open(vma);
	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_vma_open()
//
//----------------------------------------------------------------------------
static void universe_vma_open(struct vm_area_struct *vma)
{
#ifdef UNIVERSE_DEBUG
	printk(KERN_DEBUG UNIVERSE_PREFIX "  Opening Universe II VMA, virt: 0x%lx, phys: 0x%lx\n", 
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
#endif
}

//----------------------------------------------------------------------------
//
//	universe_vma_close()
//
//----------------------------------------------------------------------------
static void universe_vma_close(struct vm_area_struct *vma)
{
#ifdef UNIVERSE_DEBUG
	printk(KERN_DEBUG UNIVERSE_PREFIX "  Closing Universe II VMA, virt: 0x%lx, phys: 0x%lx\n", 
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
#endif
}

//----------------------------------------------------------------------------
//
//	universe_check_bus_error()
//
//----------------------------------------------------------------------------
static int universe_check_bus_error(void)
{
	unsigned long tmp = ioread32(universe_driver.baseaddr + PCI_CSR);
	if (tmp & 0x08000000) {	// S_TA is Set
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "PCI_CSR reg, S_TA set: 0x%lx\n", tmp);
#endif
		iowrite32(tmp, universe_driver.baseaddr + PCI_CSR);
		tmp = ioread32(universe_driver.baseaddr + PCI_CSR);
#ifdef UNIVERSE_DEBUG
		printk( KERN_DEBUG UNIVERSE_PREFIX "Bus Error Readback: 0x%lx\n", tmp);
#endif
		return -EIO ;
	}
	return 0;
}

//----------------------------------------------------------------------------
//
//	universe_irq_handler()
//
//----------------------------------------------------------------------------
static irqreturn_t universe_irq_handler(int irq, void *dev_id)
{
	long stat;
	/* OK check to see if this is the correct interrupt. */
	if (dev_id == NULL) {
		/* This means we were called by the timer. */
		spin_lock(&universe_driver.dma.lock);	
		goto setup_readout;
	}
	stat = ioread32(universe_driver.baseaddr+LINT_STAT);
	if (! (stat & 0x100) ) return IRQ_NONE;

#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "In the universe interrupt.\n");
#endif
	spin_lock(&universe_driver.dma.lock);	
	/* Once we obtain the lock, let us check again.  Another interrupt may have handled it. */
	stat = ioread32(universe_driver.baseaddr+LINT_STAT);
	if (! (stat & 0x100) ) {
		spin_unlock(&universe_driver.dma.lock);	
		return IRQ_HANDLED;
	}
	stat = ioread32(universe_driver.baseaddr+DGCS);
	if (stat & 0x00008000) {
		/*It's still running, this should never happen */
		spin_unlock(&universe_driver.dma.lock);	
		return IRQ_HANDLED;
	}
	/* This means the dma has interrupted, so we so it completed,
	   so unmap the pages and get out. */
	del_timer(&universe_driver.dma.dma_timer);
	setup_readout:
	/* Unmap the dma transfer memory, we need to free this so we can read. */
	dma_unmap_single(&universe_driver.pci_dev->dev, 
		universe_driver.dma.dma_transfer_addr, 
		universe_driver.dma.dma_size + universe_driver.dma.dma_align,
		universe_driver.dma.dma_direction);
	/* Clear the bits. */
	iowrite32(0x0100,universe_driver.baseaddr+LINT_STAT);
	/* Wake up the people waiting. */
	complete(&universe_driver.dma.dma_completion);
	spin_unlock(&universe_driver.dma.lock);	
	return IRQ_HANDLED;
}

//----------------------------------------------------------------------------
//
//	universe_dma_timeout()
//		This functions is called when the dma timer runs out.  
//		It checks to see if the dma is still running: If so, reset timer, return;
//		If not call universe_irq_handler
//
//----------------------------------------------------------------------------
static void universe_dma_timeout(unsigned long notused)
{
	// We have to be careful about concurrency.
	unsigned long  val;

	// Do a spin lock.
	// This is because the universe could issue an interrupt
	// signifying it is done right at this time.  
	spin_lock(&universe_driver.dma.lock);
	universe_driver.dma.timeouts++;
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "Timeout.\n");
#endif
        val = ioread32(universe_driver.baseaddr+DGCS);
	if (val & 0x00008000) {
	        // DMA is still running, reset timer and get out.
		universe_driver.dma.dma_timer.expires = jiffies + DMA_TIMEOUT;
		add_timer(&universe_driver.dma.dma_timer);
		spin_unlock(&universe_driver.dma.lock);
		return;
	}
	/* OK, the dma isn't running, we must have missed an interrupt. */
#ifdef UNIVERSE_DEBUG
	printk( KERN_DEBUG UNIVERSE_PREFIX "In the timeout.\n");
#endif
	/* Call the interrupt handler. */
	spin_unlock(&universe_driver.dma.lock);
	universe_irq_handler(universe_driver.irq, NULL);
}
