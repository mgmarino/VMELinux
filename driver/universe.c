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

#include <linux/utsrelease.h>
#include <linux/autoconf.h>
#include <linux/init.h>
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
#define MAX_MINOR	 9 // This includes the DMA_MINOR
#define CONTROL_MINOR	 8
#define DMA_MINOR	 9

struct universe_dev {
	int ok_to_write;		/* Can I write to the hardware */
	int opened;			/* Is it open? */
	int image_ba;			/* Base virtual address for this device */
	int image_perform_ioremap;	/* Should we ioremap? */ 
	int image_is_ioremapped;	/* Device is ioremapped. */ 
	int ctl_address;		/* Offset of the ctl for a device */	
	int bs_address;			/* Offset where bs is stored for a device */	
	int bd_address;			/* Offset where bd is stored for a device */	
	int to_address;			/* Offset where to is stored for a device */	
	int vme_address;		/* Base vme address */	
	unsigned int image_ptr;		/* Pointer to the image */
	unsigned int ctl_register;	/* Control register value for the image */	
	struct semaphore sem;		/* Mutual exclusion semaphore */
	struct cdev dev;		/* Character device structure */
};


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

static void	universe_setup_cdev(struct universe_dev *, int index); 

static void	universe_vma_open(struct vm_area_struct *);
static void	universe_vma_close(struct vm_area_struct *);

static int	universe_check_bus_error(void);
#ifdef UNIVERSE_DEBUG
static int	universe_procinfo(char *, char **, off_t, int, int *,void *);
static void	register_proc(void);
static void	unregister_proc(void);
// Status Vars
static unsigned int read	= 0;
static unsigned int writes 	= 0;
static unsigned int ioctls 	= 0;
#endif /* UNIVERSE_DEBUG */
//----------------------------------------------------------------------------
// Types
//----------------------------------------------------------------------------
static struct proc_dir_entry *universe_procdir;

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

static struct universe_dev universe_devices[universe_nr_devs];

static int irq = 0;
static char *baseaddr	= 0;

// Hold onto the major Number as it is dynamically given.
// The number shows up in /proc/devices and can be taken from 
// there by a script which generates the dev files. 
int universe_major	= 0;
int universe_minor	= 0;
int universe_nr_devs 	= MAX_MINOR + 1;
int memory_is_reserved	= 0;

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
	err = dev_add(&(dev->cdev), devno, 1);
	if (err) {
		printk(	KERN_NOTICE "Error (%d) addding universe dev: %d",
			err, index );
	}

	// Default is to ioremap so set that.
	dev->image_perform_ioremap = 1;
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

	writel(0,baseaddr+LINT_EN);		 // Turn off Ints
	pcivector = readl(baseaddr+PCI_MISC1) & 0x000000FF; 
	free_irq(pcivector,NULL);		 // Free Vector

	for (i=0;i<universe_nr_devs;i++) {
		if (	universe_devices[i]->image_ba && 
			universe_devices[i]->image_is_ioremapped == 1)
			iounmap(universe_devices[i]->image_ba);
	}		
	iounmap(baseaddr);
	if (memory_is_reserved == 1) {
		release_mem_region(reserve_from_address, size_to_reserve); 
	}
#ifdef UNIVERSE_DEBUG
	unregister_proc();
#endif
	unregister_chrdev_region(dev_number, universe_nr_devs);
}
module_exit(universe_exit_module);

//----------------------------------------------------------------------------
//	universe_init_module()
//----------------------------------------------------------------------------
static int __init
universe_init_module(void)
{
	int result;
	unsigned int temp, ba;
	char vstr[80];
	struct pci_dev *universe_pci_dev = NULL;

	sprintf(vstr,"Tundra Universe PCI-VME Bridge Driver %s\n",Version);
	printk(KERN_INFO vstr);
	printk(KERN_INFO "Copyright 1997-2001, Michael J. Wyrick\n");
	printk(KERN_INFO "Copyright 2008, Michael G. Marino\n");

	// Getting the device major and minor numbers
	dev_t dev_number;
	result = alloc_chrdev_region(&dev_number, 0, universe_nr_devs,"universe");
	universe_major = MAJOR(dev_number);
	if ( result < 0 ); 
		printk(KERN_WARNING "Error getting major: %d\n", universe_major);
		return result ;
	} else {
		printk(KERN_INFO "Device files major number: %d \n", universe_major);
	}
	// device major and minor numbers found


	if ((universe_pci_dev = pci_get_device(	PCI_VENDOR_ID_TUNDRA,
						PCI_DEVICE_ID_TUNDRA_CA91C042, 
						universe_pci_dev))) {
		// The device was found on the PCI bus. 
		printk(KERN_INFO "Universe device found.");

		// Turning latency off 
		pci_write_config_dword(universe_pci_dev, PCI_MISC0, 0);

		// Display PCI Registers	
		pci_read_config_dword(universe_pci_dev, PCI_CSR, &temp);
		printk(	KERN_INFO "Vendor = %04X Device = %04X Status = %08X\n",
			universe_pci_dev->vendor,universe_pci_dev->device,temp);
		printk(KERN_INFO "Class = %08X\n",universe_pci_dev->class);

		pci_read_config_dword(universe_pci_dev, PCI_MISC0, &temp);
		printk(KERN_INFO "Misc0 = %08X\n",temp);			

		// Setup Universe Config Space
		// This is a 4k wide memory area that need to be mapped into the kernel
		// virtual memory space so we can access it.
		pci_write_config_dword(universe_pci_dev, PCI_BS, CONFIG_REG_SPACE);
		pci_read_config_dword(universe_pci_dev, PCI_BS, &ba);				
		baseaddr = (char *)ioremap(ba,4096);
		if (!baseaddr) {
			printk(KERN_WARNING "vremap failed to map Universe to Kernel Space.\r");
			return -ENOMEM;
		}

		// Check to see if the Mapping Worked out
		printk(KERN_INFO "baseaddr read in as: %02x\n", (unsigned int)baseaddr);
		temp = readl(baseaddr);
		printk(KERN_INFO "Read via mapping, PCI_ID = %08X\n",temp);			 
		if (temp != PCI_VENDOR_ID_TUNDRA) {
			printk(KERN_WARNING "Universe Chip Failed to Return PCI_ID in Memory Map.\n");
			return -ENXIO;
		}

		// OK, Every this is ok so lets turn off the windows
		writel(0x00800000,baseaddr+LSI0_CTL);		 
		writel(0x00800000,baseaddr+LSI1_CTL);		 
		writel(0x00800000,baseaddr+LSI2_CTL);		 
		writel(0x00800000,baseaddr+LSI3_CTL);		 

		// Write to Misc Register
		// Set VME Bus Time-out
		//	 Arbitration Mode
		//	 DTACK Enable
		writel(0x15060000,baseaddr+MISC_CTL);		 

		// Lets turn off interrupts
		// Disable interrupts in the Universe first
		writel(0x00000000,baseaddr+LINT_EN);	 
		// Clear Any Pending Interrupts
		writel(0x0000FFFF,baseaddr+LINT_STAT); 

		pci_read_config_dword(universe_pci_dev, PCI_INTERRUPT_LINE, &irq);
		irq &= 0x000000FF;  // Only a byte in size
		result = request_irq(irq, irq_handler, IRQF_DISABLED, "VMEBus (universe)", NULL);
		if (result) {
			printk(KERN_WARNING "universe: can't get assigned pci irq vector %02X\n", irq);
		} else {
			writel(0x0000, baseaddr+LINT_MAP0);  // Map all ints to 0
			writel(0x0000, baseaddr+LINT_MAP1);  // Map all ints to 0
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
		memory_is_reserved = 0;
		universe_exit_module();
		return -EBUSY;
	}
	memory_is_reserved = 1;

	// Enable DMA Interrupts
	writel(0x0700, baseaddr+LINT_EN);
	
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

	p += sprintf(p,"baseaddr = %08X\n",(int)baseaddr);
	p += sprintf(p,"Stats reads = %i writes = %i ioctls = %i\n",
			 reads,writes,ioctls);
	for (x=0;x<8;x+=2) {
		temp1 = readl(baseaddr+aCTL[x]);
		temp2 = readl(baseaddr+aCTL[x+1]);
		p += sprintf(p,"	LSI%i_CTL = %08X		LSI%i_CTL = %08X\n",x,temp1,x+1,temp2);
		temp1 = readl(baseaddr+aBS[x]);
		temp2 = readl(baseaddr+aBS[x+1]);
		p += sprintf(p,"	LSI%i_BS	= %08X		LSI%i_BS	= %08X\n",x,temp1,x+1,temp2);
		temp1 = readl(baseaddr+aBD[x]);
		temp2 = readl(baseaddr+aBD[x+1]);
		p += sprintf(p,"	LSI%i_BD	= %08X		LSI%i_BD	= %08X\n",x,temp1,x+1,temp2);
		temp1 = readl(baseaddr+aTO[x]);
		temp2 = readl(baseaddr+aTO[x+1]);
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
	dev = container_of(inode->c_dev, struct universe_dev, cdev);
	if (!dev->opened) {
		dev->opened = 1;
		return 0;
	} else return -EBUSY ;
}

//----------------------------------------------------------------------------
//
//	universe_release()
//
//----------------------------------------------------------------------------
static int universe_release(struct inode *inode,struct file *file)
{
	struct universe_dev *dev; 
	dev = container_of(inode->c_dev, struct universe_dev, cdev);
	file->private_data = dev;

	dev->ok_to_write = 0;
	/* Disable the image, reset the registers */
	if (dev->ctl_address) writel(0x0, dev->ctl_address);
	if (dev->bs_address) writel(0x0, dev->bs_address);
	if (dev->bd_address) writel(0x0, dev->bd_address);
	if (dev->to_address) writel(0x0, dev->to_address);

	if (dev->image_ba && dev->image_is_ioremapped == 1) {
		/* Get rid of the ioremapped memory. */
		iounmap(dev->image_ba);
		dev->image_is_ioremapped = 0;
		dev->image_ba = 0;
	}
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
	int x = 0;
	unsigned int v,numt,remain,tmp;
	char *temp = buf;

	unsigned char	vc;
	unsigned short	vs;
	unsigned int	vl;

	char *DMA_Buffer;
	unsigned int 	DMA_Buffer_Size = 0, 
			order		= 0,
			a_size		= 0,
			dma_align 	= 0,
			timeout	 	= 0;										 

	int 	val	= 0,
		pci	= 0,
		okcount	= 0;

	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);	
	struct universe_dev *dev = file->private_data; 

	if (minor == CONTROL_MINOR) {
		v = readl(baseaddr + dev->image_ptr);		
		if (__copy_to_user(temp,&v,4) != 0) return -EIO;
		return 4;
	} 
	reads++;								
	if (dev->ok_to_write != 1) return -EPERM;
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

		// PCI Address
		pci = virt_to_bus(DMA_Buffer) + dma_align;

		// Setup DMA regs
		// setup DMA for a *read*
		DMA &= 0x7FFFFFFF; // Sets L2V bit for a read
		writel(dev->ctl_register,baseaddr+DCTL);	 // Setup Control Reg
		writel(count,baseaddr+DTBC);	 // Count			
		writel(pci,baseaddr+DLA);	// PCI Address
		writel(dev->vme_address,baseaddr+DVA);	// VME Address

		// Start DMA
		writel(0x80006F00,baseaddr+DGCS);	 // GO

		// Wait for DMA to finish, This needs to be changed
		val = readl(baseaddr+DGCS);
		while ((val & 0x00008000) && (timeout++ < 100000))
			val = readl(baseaddr+DGCS);

		if (timeout == 100000)
			printk(KERN_WARNING "<<universe DMA Timed out>>\n");

		if (! (val & 0x00000800)) {	// An Error Happened
			count -= readl(baseaddr+DTBC);		
			printk(KERN_WARNING "Error happened, DTBC read back: 0x%x\n",val);
		}

		// Copy pages to User Memory
		if (__copy_to_user(temp,DMA_Buffer+dma_align,count) != 0 ) {
			return -EIO;
		}
		free_pages((unsigned long)DMA_Buffer,order);

	} else { 
		numt = count;
		remain = count;

		// First we read as many longs as we can. 
		numt = count / 4;
		remain = count % 4;
		for (x=0;x<numt;x++) {
			vl = readl((char*)dev->image_ptr);		

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			okcount += 4;

		 	if ( __copy_to_user(temp,&vl,4) != 0) return -EIO;

			dev->image_ptr += 4;
			temp+=4;
		}	

		// Next we read the words (should be max 1) 
		numt = remain / 2;
		remain = remain % 2;
		for (x=0;x<numt;x++) {
			vs = readw((char*)dev->image_ptr);		

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;

			okcount += 2;

			if (__copy_to_user(temp,&vs,2) != 0) return -EIO;
			dev->image_ptr += 2;
			temp +=2 ;
		}	

		// Next we read the bytes (should be max 1) 
		for (x=0;x<remain;x++) {
			vc = readb((char*)dev->image_ptr);		

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;

			okcount++;

			if (__copy_to_user(temp,&vc,1) != 0) return -EIO;

			dev->image_ptr += 1;
			temp+=1;
		}	
	}
	*ppos += count;

	return count;
	
}

//----------------------------------------------------------------------------
//
//	universe_write()
//
//----------------------------------------------------------------------------
static ssize_t universe_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int x,p;
	unsigned int numt,remain,tmp;
	char *temp = (char *)buf;

	unsigned char	vc;
	unsigned short 	vs;
	unsigned int	vl;

	char *DMA_Buffer;
	unsigned int 	DMA_Buffer_Size	= 0, 
			order		= 0,
			a_size		= 0,
			dma_align 	= 0,
			timeout	 	= 0;										 
	int 	val 	= 0,
		pci	 = 0,
		okcount	 = 0;

	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct universe_dev *dev = file->private_data; 
	writes++;
	if (minor == CONTROL_MINOR) {
		if (__copy_from_user(&vl,temp,4) != 0) return -EIO;
		writel(vl,baseaddr + (int)dev->image_ptr);
	} 
	if (dev->ok_to_write != 1) return -EPERM;
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
		dma_align = dev->vme_address & 0x7;	

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
		dev->ctl_register |= 0x80000000; // Sets L2V bit for a write
		writel(dev->ctl_register, baseaddr + DCTL);	// Setup Control Reg
		writel(count, baseaddr + DTBC);		// Count			
		writel(pci, baseaddr + DLA);		 // PCI Address
		writel(dev->vme_address, baseaddr + DVA);	 // VME Address

		// Start DMA
		writel(0x80006F00, baseaddr + DGCS);	 // GO

		// Wait for DMA to finish, This needs to be changed
		val = readl(baseaddr + DGCS);
		while ((val & 0x00008000) && (timeout++ < 100000))
			val = readl(baseaddr+DGCS);

		if (timeout == 100000)
			printk(KERN_WARNING "<<universe DMA Timed out>>\n");

		if (! (val & 0x00000800)) {	// An Error Happened
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
			writel(vl,(char*)dev->image_ptr);

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			okcount += 4;

			dev->image_ptr += 4;
			temp += 4;
		}	

		// Calc the number of Words we need
		numt = remain / 2;
		remain = remain % 2;

		for (x=0;x<numt;x++) {
			__copy_from_user(&vs,temp,2);
			writew(vs,(char*)image_ptr[minor]);

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			okcount += 2;
			dev->image_ptr += 2;
			temp += 2;
		}	

		// Now read out the bytes remaining (should be 1) 
		for (x=0;x<remain;x++) {
			__copy_from_user(&vc,temp,1);
			writeb(vc,(char*)dev->image_ptr);

			// Check for a Bus Error
			tmp = universe_check_bus_error(); 
			if ( tmp != 0 ) return tmp;
			okcount += 2;

			dev->image_ptr += 1;
			temp += 1;
		}	

	}	
	*ppos += count;
	return count;
}

//----------------------------------------------------------------------------
//	universe_ioctl()
//----------------------------------------------------------------------------
//	For a normal image minor, initialize it by (in this order):
//		IOCTL_SET_CTL: Set control register (address space, data space, etc.)
//		IOCTL_SET_IOREMAP: set to ioremap pci mem to kernel memory.
//		IOCTL_SET_BS:	Set base (offset) of window.	The driver will automatically
//			determine where this is within it's allowed PCI space.
//		IOCTL_SET_BD:	Set bound (size) of window.
//		IOCTL_SET_VME: Use this to set the desired VME address the base
//			of the window will point to.
//
//	For a DMA minor, initialize it by (in this order):
//		IOCTL_SET_CTL: Set control register (address space, data space, etc.)
//		IOCTL_SET_VME: Use this to set the desired VME address from which the 
//			DMA will start. 
//
//	For a control minor, the following functions are allowed:
//		IOCTL_SET_HW_BYTESWAP: set hardware byteswap
//		IOCTL_GET_MEM_SIZE:		get size of pci memory set aside by the driver
//		default: write ard to the register given by the cmd number

static int universe_ioctl(struct inode *inode,struct file *file,unsigned int cmd, unsigned long arg)
{
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned long sizetomap = 0, bs = 0;
	unsigned char readBack = 0;
	unsigned long *tempPtr;

	struct universe_dev *dev = file->private_data; 

	ioctls++;
	switch (cmd) {

	case IOCTL_SET_CTL:
		if (minor == CONTROL_MINOR) return -EPERM;
		if (minor == DMA_MINOR) {
			// Lets compute and save the DMA CTL Register
			dev->ctl_address = arg & 0x80C7F380; //crazy bitmask, a bunch of reserved areas
			dev->ok_to_write = 1;
		} else {
			arg &= 0xFFFFFFFE; // *always* use memory space, not i/o 
			writel(arg,dev->ctl_address);
		}
		break;

	case IOCTL_SET_BS:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		if (arg >= size_to_reserve ) {
		/* we want to make sure that the base offset doesn't overshoot our reserved memory. */
			return -EFAULT;
		}
		writel(arg+reserve_from_address,baseaddr+dev->bs_address);
		break;

	case IOCTL_SET_BD:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		bs = readl(baseaddr + dev->bs_address);
		if (arg+bs < size_to_reserve+reserve_from_address) {
			/* we want to make sure that the bound doesn't overshoot our reserved memory. */
			
			writel(arg+bs,baseaddr+dev->bd_addressa);
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
				dev->image_ptr = (int)dev->image_ba;
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

	case IOCTL_SET_VME:
		/* We get a VME address and insert the offset. */
		/* BS needs to be set before this. */
		if (minor == CONTROL_MINOR) return -EPERM;
		dev->vme_address = arg;
		if (minor != DMA_MINOR) {
			// Calculate the VME Address
			bs = readl(baseaddr + dev->bs_address);
			writel(arg-bs,baseaddr + dev->to_address);
			bs = readl(baseaddr + dev->to_address);
		}
		break;	

	case IOCTL_SET_IOREMAP:
		if (minor == CONTROL_MINOR || minor == DMA_MINOR) return -EPERM;
		dev->image_perform_ioremap = (arg == 0) ? 0 : 1;
		break;

	case IOCTL_SET_HW_BYTESWAP:
		if (minor != CONTROL_MINOR) return -EPERM;
		readBack = (0x38 & arg);
		outb(readBack, VX407_CSR0);	
		if ((0x38 & arg) != (readBack = (0x38 & inb(VX407_CSR0)))) { 
			printk(	KERN_WARNING " Hardware swap not set at address 0x%x. Set: 0x%x, Readback: 0x%x", 
				VX407_CSR0, (unsigned int)(0x38 & arg), readBack);
			return -EIO;
		}
		break;

	case IOCTL_GET_MEM_SIZE:
		if (minor != CONTROL_MINOR) return -EPERM;
		tempPtr = (unsigned long*) arg;
		/* only returning the size.	The mem location is handled internally. */
		/* It is the responsibility of the calling API to appropriately 
		 * setup how the memory exists in the chunk (i.e. by setting up offsets).*/
		return __copy_to_user(tempPtr, &size_to_reserve, sizeof(size_to_reserve));
		break; 

	default:
		if (minor != CONTROL_MINOR) return -EPERM;
		/* only let the control minor write to this. */
		if (cmd < 0x1000) {	// This is a Register value so write to it.
			writel(arg,baseaddr+cmd);
		} else return -EFAULT;
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

	bs = readl(baseaddr + dev->bs_address);
	bd = readl(baseaddr + dev->bd_address);

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
				physical_address >> PAGE_SIZE, 
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
	int tmp = readl(baseaddr + PCI_CSR);
	if (tmp & 0x08000000) {	// S_TA is Set
		writel(0x08000000, baseaddr + PCI_CSR);
		return -EIO ;
	}
	return 0;
}

