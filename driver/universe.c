//------------------------------------------------------------------------------	
//title: Tundra Universe PCI-VME Kernel Driver
//version: Linux 1.1
//date: March 1999																																
//designer: Michael Marino, Michael Wyrick 
//programmer: Michael Wyrick, Michael Marino 
//platform: Linux 2.6
//language: GCC 4.x
//module: universe
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
// $Id: universe.c,v 1.5 2001/10/27 03:50:07 jhuggins Exp $
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

static char Version[] = "1.3.r1 2008Jan07";

#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/utsrelease.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/atomic.h>
#include <linux/spinlock.h>

#include "universe.h"

// Including Vendor tags if the linux kernel hasn't defined them
#ifndef PCI_VENDOR_ID_TUNDRA
	#define PCI_VENDOR_ID_TUNDRA 0x10e3
#endif

#ifndef PCI_DEVICE_ID_TUNDRA_CA91C042
	#define PCI_DEVICE_ID_TUNDRA_CA91C042 0x0000
#endif

#define VX407_CSR0	 0x210


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
#define MAX_MINOR	9 // This includes the DMA_MINOR
#define CONTROL_MINOR	8
#define DMA_MINOR	9
#define DMA_TIMEOUT	100000 // jiffies

struct universe_dev {
	int ok_to_write;		/* Can I write to the hardware */
	atomic_t opened;		/* Is it open? */
	int image_perform_ioremap;	/* Should we ioremap? */ 
	int image_is_ioremapped;	/* Device is ioremapped. */ 
	int ctl_address;		/* Offset of the ctl for a device */	
	int bs_address;			/* Offset where bs is stored for a device */	
	int bd_address;			/* Offset where bd is stored for a device */	
	int to_address;			/* Offset where to is stored for a device */	
	int vme_address;		/* Base vme address */	
	char *image_ba;			/* Base virtual address for this device */
	char *image_ptr;		/* Pointer to the current place in image */
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
	int memory_is_reserved;		/* Is memory reserved. */
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

static void	universe_setup_universe_dev(struct universe_dev *, int index); 

static void	universe_vma_open(struct vm_area_struct *);
static void	universe_vma_close(struct vm_area_struct *);

static int	universe_check_bus_error(void);
static void 	universe_dma_timeout(unsigned long);
static irqreturn_t universe_irq_handler(int irq, void *dev_id);

#ifdef UNIVERSE_DEBUG
static int	universe_procinfo(char *, char **, off_t, int, int *,void *);
static void	register_proc(void);
static void	unregister_proc(void);

static struct proc_dir_entry *universe_procdir;
#endif /* UNIVERSE_DEBUG */
//----------------------------------------------------------------------------
// Types
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

// Status Vars
//-----------------------------------------------------------------------------
// Function	: universe_setup_universe_dev
// Inputs	: void
// Outputs	: void
// Description	: This sets up the universe device struct.  
// Remarks	: 
// History	: 
//-----------------------------------------------------------------------------
static void universe_setup_universe_dev(struct universe_dev *dev, int index)
{
	int err;
	int devno = MKDEV(universe_major, universe_minor + index);

	// First zero everything
	memset(dev, 0, sizeof(struct universe_dev));
	
	cdev_init(&(dev->cdev), &universe_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &universe_fops;
	err = cdev_add(&(dev->cdev), devno, 1);
	if (err) {
		printk(	KERN_NOTICE "Error (%d) addding universe dev: %d\n",
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
	}
}
//-----------------------------------------------------------------------------
// Function	: universe_exit_module
// Inputs	: void
// Outputs	: void
// Description	: 
// Remarks	: 
// History	: 
//-----------------------------------------------------------------------------
static void __exit
universe_exit_module(void)
{					
	int i;
	int pcivector;
	dev_t dev_number = MKDEV(universe_major, universe_minor);

	iowrite32(0,universe_driver.baseaddr + LINT_EN);		 // Turn off Ints
	pcivector = ioread32(universe_driver.baseaddr+PCI_MISC1) & 0x000000FF; 
	free_irq(universe_driver.irq,&universe_driver);		 // Free Vector

	for (i=0;i<universe_nr_devs;i++) {
		if (	universe_devices[i].image_ba && 
			universe_devices[i].image_is_ioremapped == 1)
			iounmap(universe_devices[i].image_ba);
	}		
	iounmap(universe_driver.baseaddr);
	if (universe_driver.memory_is_reserved == 1) {
		release_mem_region(reserve_from_address, size_to_reserve); 
	}
#ifdef UNIVERSE_DEBUG
	unregister_proc();
#endif
	unregister_chrdev_region(dev_number, universe_nr_devs);
	pci_dev_put(universe_driver.pci_dev);
}
module_exit(universe_exit_module);

//----------------------------------------------------------------------------
//	universe_init_module()
//----------------------------------------------------------------------------
static int __init
universe_init_module(void)
{
	int result,i;
	unsigned int temp, ba;
	char vstr[80];
	dev_t dev_number;

	sprintf(vstr,"Tundra Universe PCI-VME Bridge Driver %s\n",Version);
	printk(KERN_INFO "%s", vstr);
	printk(KERN_INFO "Copyright 1997-2001, Michael J. Wyrick\n");
	printk(KERN_INFO "Copyright 2008, Michael G. Marino\n");

	// Getting the device major and minor numbers
	result = alloc_chrdev_region(&dev_number, 0, universe_nr_devs,"universe");
	if ( result < 0 ) { 
		printk(KERN_WARNING "Error getting major: %d\n", universe_major);
		return result;
	} else {
		printk(KERN_INFO "Device files major number: %d \n", universe_major);
	}
	universe_major = MAJOR(dev_number);
	// device major and minor numbers found

	// zero out the driver struct
	memset(&universe_driver, 0, sizeof(universe_driver));
	 
	if ((universe_driver.pci_dev = pci_get_device(	PCI_VENDOR_ID_TUNDRA,
						PCI_DEVICE_ID_TUNDRA_CA91C042, 
						NULL))) {
		// The device was found on the PCI bus. 
		printk(KERN_INFO "Universe device found.");
		if (!(result = pci_enable_device(universe_driver.pci_dev))) {
			printk(KERN_WARNING "Failed to enable Universe device.\n");
			return result;
		}
		printk(KERN_INFO "Universe device enabled.");

		// Turning latency off 
		pci_write_config_dword(universe_driver.pci_dev, PCI_MISC0, 0);
		// Display PCI Registers	
		pci_read_config_dword(universe_driver.pci_dev, PCI_CSR, &temp);

		printk(	KERN_INFO "Vendor = %04X Device = %04X Status = %08X\n",
			universe_driver.pci_dev->vendor,universe_driver.pci_dev->device,temp);
		printk(KERN_INFO "Class = %08X\n",universe_driver.pci_dev->class);

		pci_read_config_dword(universe_driver.pci_dev, PCI_MISC0, &temp);
		printk(KERN_INFO "Misc0 = %08X\n",temp);			

		// Setup Universe Config Space
		// This is a 4k wide memory area that need to be mapped into the kernel
		// virtual memory space so we can access it.
		pci_write_config_dword(universe_driver.pci_dev, PCI_BS, CONFIG_REG_SPACE);
		pci_read_config_dword(universe_driver.pci_dev, PCI_BS, &ba);				
		universe_driver.baseaddr = (void *)ioremap(ba, CONFIG_SPACE_SIZE);
		if (!universe_driver.baseaddr) {
			printk(KERN_WARNING "vremap failed to map Universe to Kernel Space.\r");
			return -ENOMEM;
		}

		// Check to see if the Mapping Worked out
		printk(KERN_INFO "baseaddr read in as: %02x\n", (unsigned int)universe_driver.baseaddr);
		temp = ioread32(universe_driver.baseaddr);
		printk(KERN_INFO "Read via mapping, PCI_ID = %08X\n",temp);			 
		if (temp != PCI_VENDOR_ID_TUNDRA) {
			printk(KERN_WARNING "Universe Chip Failed to Return PCI_ID in Memory Map.\n");
			return -ENXIO;
		}

		//FixME: Move all of this out of the init module.  This should be handled in IOCTL
		// Write to Misc Register
		// Set VME Bus Time-out
		//	 Arbitration Mode
		//	 DTACK Enable
		iowrite32(0x15060000,universe_driver.baseaddr+MISC_CTL);		 

		// Turn off interrupts
		// Disable interrupts in the Universe first
		iowrite32(0x00000000,universe_driver.baseaddr + LINT_EN);	 
		// Clear Any Pending Interrupts
		iowrite32(0x0000FFFF,universe_driver.baseaddr + LINT_STAT); 
		// Grabbing the interrupt request
		universe_driver.irq = universe_driver.pci_dev->irq;
		result = request_irq(universe_driver.irq, universe_irq_handler, IRQF_DISABLED, "VMEBus (universe)", &universe_driver);
		if (result) {
			printk(KERN_ERR "universe: can't get assigned irq %02X\n", universe_driver.irq);
			universe_exit_module();
			return -EBUSY;
		} else {
			iowrite32(0x00000000, universe_driver.baseaddr+LINT_MAP0);  
			iowrite32(0x00000000, universe_driver.baseaddr+LINT_MAP1);  
			iowrite32((universe_driver.irq & 0x7), universe_driver.baseaddr+LINT_MAP1);  
			// Enabling DMA interrupts
			iowrite32(0x00000100,universe_driver.baseaddr + LINT_EN);	 
			printk(KERN_INFO "universe: Assigned irq %02X\n", universe_driver.irq);
		}

	} else {
		// We didn't find the universe device, so get out. 
		printk(KERN_WARNING "Universe device not found on PCI Bus.\n");
		return -ENXIO;
	}
#ifdef UNIVERSE_DEBUG
	register_proc();
#endif
	// Finding a large chunk of memory defined with the inputs
	if (request_mem_region(reserve_from_address, size_to_reserve, "universe") == NULL) {
		/* Failed to get memory. */
		printk(	KERN_WARNING " Error reserving memory from %lx to %lx", 
			reserve_from_address, reserve_from_address + size_to_reserve - 1);
		universe_exit_module();
		return -EBUSY;
	}
	universe_driver.memory_is_reserved = 1;

	for(i=0;i<universe_nr_devs;i++) universe_setup_universe_dev(&universe_devices[i], i);	
	// Success 
	return 0; 
}
module_init(universe_init_module);


#ifdef UNIVERSE_DEBUG
//----------------------------------------------------------------------------
//	universe_procinfo()
//----------------------------------------------------------------------------
static int universe_procinfo(char *buf, char **start, off_t fpos, int lenght, int *eof, void *data)
{
	char *p;
	unsigned int temp1,temp2,x;

	p = buf;
	p += sprintf(p,"Universe driver %s\n",Version);

	p += sprintf(p,"baseaddr = %08X\n",(int)universe_driver.baseaddr);
	p += sprintf(p,"Stats reads = %i writes = %i ioctls = %i\n",
			 universe_driver.reads,universe_driver.writes,universe_driver.ioctls);
	for (x=0;x<8;x+=2) {
		temp1 = ioread32(universe_driver.baseaddr+aCTL[x]);
		temp2 = ioread32(universe_driver.baseaddr+aCTL[x+1]);
		p += sprintf(p,"	LSI%i_CTL = %08X		LSI%i_CTL = %08X\n",x,temp1,x+1,temp2);
		temp1 = ioread32(universe_driver.baseaddr+aBS[x]);
		temp2 = ioread32(universe_driver.baseaddr+aBS[x+1]);
		p += sprintf(p,"	LSI%i_BS	= %08X		LSI%i_BS	= %08X\n",x,temp1,x+1,temp2);
		temp1 = ioread32(universe_driver.baseaddr+aBD[x]);
		temp2 = ioread32(universe_driver.baseaddr+aBD[x+1]);
		p += sprintf(p,"	LSI%i_BD	= %08X		LSI%i_BD	= %08X\n",x,temp1,x+1,temp2);
		temp1 = ioread32(universe_driver.baseaddr+aTO[x]);
		temp2 = ioread32(universe_driver.baseaddr+aTO[x+1]);
		p += sprintf(p,"	LSI%i_TO	= %08X		LSI%i_TO	= %08X\n",x,temp1,x+1,temp2);
	}	

	p += sprintf(p,"\nDriver Program Status:\n");	

	p += sprintf(p,"	DMACTL	 = %08lX \n",
								 DMA);				
	for (x=0;x<8;x+=2)
		p += sprintf(p,"  OkToWrite %i = %1X  OkToWrite %i = %1X\n",x,OkToWrite[x],x+1,OkToWrite[x+1]); 
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
//	register_proc()
//----------------------------------------------------------------------------
static void register_proc()
{
	universe_procdir = create_proc_entry("universe", S_IFREG | S_IRUGO, 0);
	universe_procdir->read_proc = universe_procinfo;
}

//----------------------------------------------------------------------------
//	unregister_proc()
//----------------------------------------------------------------------------
static void unregister_proc()
{
	remove_proc_entry("universe",0);
}
#endif /* UNIVERSE_DEBUG */

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
	if (!atomic_dec_and_test(&dev->opened)) {
		/* Someone else has opened, this is not allowed. */
		atomic_inc(&dev->opened);
		return -EBUSY;
	} 
	file->private_data = dev;
	if (minor == CONTROL_MINOR) return 0;
	if (minor == DMA_MINOR) {
		/* get free pages for the dma device. */
		/* We request a lot, but we should be ok. */
		dev->buffer = (void *)__get_free_pages(GFP_KERNEL, MAX_ORDER);
		if (!dev->buffer) {
			atomic_inc(&dev->opened);
			return -EFAULT;
		}
		dev->buffer_length = PAGE_SIZE << MAX_ORDER;
		universe_driver.dma.dma_timer.data = 0;
		universe_driver.dma.dma_timer.function = universe_dma_timeout;
		spin_lock_init(&universe_driver.dma.lock);
		init_completion(&universe_driver.dma.dma_completion);
	} else {
		/* malloc some area for the other minors, just 1 page */
		/* Zero it, not necessary but could save us some trouble.*/
		dev->buffer = (void *)get_zeroed_page(GFP_KERNEL);
		if (!dev->buffer) {
			atomic_inc(&dev->opened);
			return -EFAULT;
		}
		dev->buffer_length = PAGE_SIZE;
	}
	return 0;
	// Enable DMA Interrupts
	//iowrite32(0x0700, universe_driver.baseaddr+LINT_EN);
}

//----------------------------------------------------------------------------
//
//	universe_release()
//
//----------------------------------------------------------------------------
static int universe_release(struct inode *inode,struct file *file)
{
	struct universe_dev *dev = file->private_data; 

	dev->ok_to_write = 0;
	/* Disable the image, reset the registers */
	if (dev->ctl_address) iowrite32(0x0, universe_driver.baseaddr 
						+ dev->ctl_address);
	if (dev->bs_address) iowrite32(0x0, universe_driver.baseaddr 
						+ dev->bs_address);
	if (dev->bd_address) iowrite32(0x0, universe_driver.baseaddr 
						+ dev->bd_address);
	if (dev->to_address) iowrite32(0x0, universe_driver.baseaddr 
						+ dev->to_address);

	if (dev->image_ba && dev->image_is_ioremapped == 1) {
		/* Get rid of the ioremapped memory. */
		iounmap(dev->image_ba);
		dev->image_is_ioremapped = 0;
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
	unsigned int v,numt,remain,tmp;
	int 	val	= 0;

	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);	
	struct universe_dev *dev = file->private_data; 

	universe_driver.reads++;
	if (minor == CONTROL_MINOR) {
		v = ioread32(universe_driver.baseaddr + (unsigned int)dev->image_ptr);		
		if (copy_to_user(buf,&v,sizeof(v))) return -EFAULT;
		return sizeof(v);
	} 
	if (dev->ok_to_write != 1) return -EPERM;
	if (minor == DMA_MINOR) {
		// Check the semaphore_lock
		// If this is locked, it means that we have called this
		// function and began the dma transfer so now we wait.
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
				printk( KERN_WARNING "DMA is busy, but lock was obtained!\n");
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
				printk(KERN_WARNING "Requested DMA transfer is larger than %lx\n", 
					(long unsigned int) dev->buffer_length);
				up(&dev->sem);
				return -EPERM;
			}				 
                        
			// PCI Address
			universe_driver.dma.dma_transfer_addr = dma_map_single(&universe_driver.pci_dev->dev, dev->buffer, 
								universe_driver.dma.dma_size + universe_driver.dma.dma_align,
								universe_driver.dma.dma_direction);
			// Setup DMA regs
			// setup DMA for a *read*
			dev->ctl_register &= 0x7FFFFFFF; // Sets L2V bit for a read
			iowrite32(dev->ctl_register,universe_driver.baseaddr + DCTL); // Setup Control Reg
			iowrite32(count,universe_driver.baseaddr + DTBC); // Count			
			iowrite32(universe_driver.dma.dma_transfer_addr+universe_driver.dma.dma_align,
					universe_driver.baseaddr + DLA); // PCI Address
			iowrite32(dev->vme_address,universe_driver.baseaddr+DVA);	// VME Address
                        
			universe_driver.dma.dma_timer.expires = jiffies + DMA_TIMEOUT;
			add_timer(&universe_driver.dma.dma_timer);
			// Start DMA
			iowrite32(0x80006F00,universe_driver.baseaddr+DGCS);	 // GO
			if(wait_for_completion_interruptible(&universe_driver.dma.dma_completion)) {
				// Huh, this is a problem, we will lose the data.
				printk(KERN_WARNING "Requested DMA transfer interrupted.\n");
				up(&dev->sem);
				return -ERESTARTSYS;
			}
		//read_dma:
			// OK, so we waited and it apparently completed.  Check for errors.
			val = ioread32(universe_driver.baseaddr+DGCS);
			if (!(val & 0x00000800)) {
				printk(KERN_WARNING "Requested DMA read transfer failed.\n");
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
			

	} else { 
		// Normal minor
		remain = count;
		while (remain > 0) {
			numt = (count > dev->buffer_length) ? dev->buffer_length :
						    	      count;
			/* Copying from i/o memory.  */
			// We use coupled read/writes exclusively in the image programming. 
			// When the Universe II encounters a BERR, it generates Target-Abort
			// on the PCI bus and sets the S_TA bit in the PCI_CSR register.
			// This should happen immediately so that the following function
			// will exit immediately following a BERR.  We always check if there
			// was an error on the bus.
			memcpy_fromio(dev->buffer,dev->image_ptr,numt);
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			if(copy_to_user(buf, dev->buffer, numt)) return -EFAULT;
			remain -= numt;
			dev->image_ptr += numt;
		}
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
	unsigned int numt,remain,tmp;
	unsigned int	vl;
	int 	val 	= 0;

	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct universe_dev *dev = file->private_data; 
	universe_driver.writes++;
	if (minor == CONTROL_MINOR) {
		if (copy_from_user(&vl,buf,sizeof(vl))) return -EFAULT;
		iowrite32(vl,universe_driver.baseaddr + (int)dev->image_ptr);
		return sizeof(vl);
	} 
	if (dev->ok_to_write != 1) return -EPERM;
	if (minor == DMA_MINOR) {
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
				printk( KERN_WARNING "DMA is busy, but lock was obtained!\n");
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
				printk(KERN_WARNING "Requested DMA transfer is larger than %lx\n", 
					(long unsigned int)dev->buffer_length);
				up(&dev->sem);
				return -EPERM;
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
			iowrite32(0x80006F00,universe_driver.baseaddr+DGCS);	 // GO
			// Now, the write also waits for completion.  Fix in the future?
			if(wait_for_completion_interruptible(&universe_driver.dma.dma_completion)) {
				// Huh, this is a problem, we will lose the data.
				printk(KERN_ERR "Requested DMA transfer interrupted.\n");
				up(&dev->sem);
				return -ERESTARTSYS;
			}
			val = ioread32(universe_driver.baseaddr+DGCS);
			if (!(val & 0x00000800)) {
				printk(KERN_WARNING "Requested DMA write transfer failed.\n");
				up(&dev->sem);
				return 0;
			}
			up(&dev->sem);
			return count;

	} else {					
		// dealing with an image minor
		remain = count;
		while (remain > 0) {
			// We use coupled read/writes exclusively in the image programming. 
			// When the Universe II encounters a BERR, it generates Target-Abort
			// on the PCI bus and sets the S_TA bit in the PCI_CSR register.
			// This should happen immediately so that the following function
			// will exit immediately following a BERR.  We always check if there
			// was an error on the bus.
			numt = (count > dev->buffer_length) ? dev->buffer_length :
						    	      count;
			if(copy_from_user(dev->buffer, buf, numt)) return -EFAULT;
			memcpy_toio(dev->buffer,dev->image_ptr,numt);
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			remain -= numt;
			dev->image_ptr += numt;
		}

	}	
	return count;
}

//----------------------------------------------------------------------------
//	universe_ioctl()
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
	unsigned char readBack = 0;
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
			dev->ctl_address = arg & 0x80C7F380; //crazy bitmask, a bunch of reserved areas
			dev->ok_to_write = 1;
		} else {
			arg &= 0xFFFFFFFE; // *always* use memory space, not i/o 
			iowrite32(arg,universe_driver.baseaddr + dev->ctl_address);
		}
		break;

	case UNIVERSE_IOCSET_BS:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		if (arg >= size_to_reserve ) {
		/* we want to make sure that the base offset doesn't overshoot our reserved memory. */
			return -EFAULT;
		}
		iowrite32(arg+reserve_from_address,universe_driver.baseaddr+dev->bs_address);
		break;

	case UNIVERSE_IOCSET_BD:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		bs = ioread32(universe_driver.baseaddr + dev->bs_address);
		if (arg+bs < size_to_reserve+reserve_from_address) {
			/* we want to make sure that the bound doesn't overshoot our reserved memory. */
			
			iowrite32(arg+bs,universe_driver.baseaddr+dev->bd_address);
			if (dev->image_ba && dev->image_is_ioremapped == 1) {
				iounmap(dev->image_ba);
				dev->image_is_ioremapped = 0;
				dev->image_ba = 0;
			}
	
			// This uses the BS Register to Find the size of the Image Mapping		
	
			if (dev->image_perform_ioremap == 1) {
				sizetomap = arg; 
				dev->image_ba = (char *)ioremap(bs, sizetomap);
				if (!dev->image_ba) {
					dev->ok_to_write = 0;		 
					printk( KERN_ERR "Error in ioremap, address: %lx, size: %lx\n", 
						arg + reserve_from_address, sizetomap);
					return -EFAULT;
				}
				dev->image_ptr = dev->image_ba;
				dev->ok_to_write = 1;
				dev->image_is_ioremapped = 1;
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
			bs = ioread32(universe_driver.baseaddr + dev->to_address);
		}
		break;	

	case UNIVERSE_IOCSET_IOREMAP:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		dev->image_perform_ioremap = (arg == 0) ? 0 : 1;
		break;

	case UNIVERSE_IOCSET_HW_BYTESWAP:
		if (minor != CONTROL_MINOR) return -EPERM;
		readBack = (0x38 & arg);
		outb(readBack, VX407_CSR0);	
		if ((0x38 & arg) != (readBack = (0x38 & inb(VX407_CSR0)))) { 
			printk(	KERN_WARNING " Hardware swap not set at address 0x%x. Set: 0x%x, Readback: 0x%x", 
				VX407_CSR0, (unsigned int)(0x38 & arg), readBack);
			return -EIO;
		}
		break;

	case UNIVERSE_IOCGET_MEM_SIZE:
		if (minor != CONTROL_MINOR) return -EPERM;
		/* only returning the size.	The mem location is handled internally. */
		/* It is the responsibility of the calling API to appropriately 
		 * setup how the memory exists in the chunk (i.e. by setting up offsets).*/
		return __put_user(size_to_reserve, (unsigned long __user *)arg );
		break; 

	}
	return 0;
}

//----------------------------------------------------------------------------
//	universe_mmap()
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
		printk(KERN_WARNING "  mmap failure:offset (%lx) is too large.", offset);
		return -EFAULT;
	}
	physical_size = (bd - bs) - offset;	
	if (virtual_size > physical_size) {
		/* Range spans too much */
		printk(KERN_WARNING "  mmap failure:range (%lx) spans too much.", virtual_size);
		return -EINVAL;
	}

	// we do not want to have this area swapped out, lock it
	vma->vm_flags |= VM_RESERVED;
	/* The cpu CANNOT cache these pages.  Also, this will keep a core dump from reading this mem. */
	vma->vm_flags |= VM_IO;
	/* OK, now remap. */
	printk(KERN_INFO "  Mapping physical address (0x%lx), size (0x%lx) to user space", 
		physical_address, virtual_size);
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
//	universe_vma_open()
//----------------------------------------------------------------------------
static void universe_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_INFO "  Opening Universe II VMA, virt: %lx, phys: %lx\n", 
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}
//----------------------------------------------------------------------------
//	universe_vma_close()
//----------------------------------------------------------------------------
static void universe_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_INFO "  Closing Universe II VMA, virt: %lx, phys: %lx\n", 
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}
//----------------------------------------------------------------------------
//	universe_check_bus_error()
//----------------------------------------------------------------------------
static int universe_check_bus_error(void)
{
	int tmp = ioread32(universe_driver.baseaddr + PCI_CSR);
	if (tmp & 0x08000000) {	// S_TA is Set
		iowrite32(0x08000000, universe_driver.baseaddr + PCI_CSR);
		return -EIO ;
	}
	return 0;
}
//----------------------------------------------------------------------------
//	universe_irq_handler()
//		This is a *fast* interrupt handler, that is interrupts are
//		disabled while this is running. 
//----------------------------------------------------------------------------
static irqreturn_t universe_irq_handler(int irq, void *dev_id)
{
	long stat;
	spin_lock(&universe_driver.dma.lock);	
	/* OK check to see if this is the correct interrupt. */
	stat = ioread32(universe_driver.baseaddr+LINT_STAT);
  	if (stat & 0x00000100) {
		stat = ioread32(universe_driver.baseaddr+DGCS);
		if (stat & 0x00008000) {
			/*It's still running, this should never happen */
			spin_unlock(&universe_driver.dma.lock);	
			return IRQ_HANDLED;
		}
		del_timer(&universe_driver.dma.dma_timer);
		/* This means the dma has interrupted, so we so it completed,
		   so unmap the pages and get out. */
		dma_unmap_single(&universe_driver.pci_dev->dev, 
			universe_driver.dma.dma_transfer_addr, 
			universe_driver.dma.dma_size + universe_driver.dma.dma_align,
			universe_driver.dma.dma_direction);
		/* Clear the bits. */
	 	iowrite32(0x0100,universe_driver.baseaddr+LINT_STAT);
		complete(&universe_driver.dma.dma_completion);
	}
	spin_unlock(&universe_driver.dma.lock);	
	return IRQ_HANDLED;
}
//----------------------------------------------------------------------------
//	universe_dma_timeout()
//		This functions is called when the dma timer runs out.  
//		It checks to see if the dma is still running: If so, reset timer, return;
//		If not call universe_irq_handler
//----------------------------------------------------------------------------
static void universe_dma_timeout(unsigned long notused)
{
	// We have to be careful about concurrency.
	unsigned long  val, flags;

	// Do a spin lock and disable interrupt requests.
	// This is because the universe could issue an interrupt
	// signifying it is done right at this time.  
	spin_lock_irqsave(&universe_driver.dma.lock, flags);
	universe_driver.dma.timeouts++;
	val = ioread32(universe_driver.baseaddr+DGCS);
	if (val & 0x00008000) {
		// DMA is still running, reset timer and get out.
		universe_driver.dma.dma_timer.expires = jiffies + DMA_TIMEOUT;
		add_timer(&universe_driver.dma.dma_timer);
		spin_unlock_irqrestore(&universe_driver.dma.lock, flags);
		return;
	}
	/* OK, the dma isn't running, we must have missed an interrupt. */
	/* Call the interrupt handler. */
	spin_unlock_irqrestore(&universe_driver.dma.lock, flags);
	universe_irq_handler(universe_driver.irq, NULL);
}
