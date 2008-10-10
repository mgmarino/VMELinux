#define DMATYPE_SINGLE  1
#define DMATYPE_LLIST   2


// Timers
struct timer_list DMA_timer; // This is a timer for returning status

	// Setup the DMA Timer
	init_timer(&DMA_timer);
	DMA_timer.function = DMA_status;

static int DMAType = 0;
static TDMAcallback DMACallBackFunc = NULL;
static void *debugptr = NULL;

// Hold the VME Irq Handlers
static TirqHandler vmeirqs[7] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static short vmeirqbit[7]     = {IRQ_VIRQ1, IRQ_VIRQ2, IRQ_VIRQ3, IRQ_VIRQ4,
                                 IRQ_VIRQ5, IRQ_VIRQ6, IRQ_VIRQ7};               
static int vmevec[7]          = {V1_STATID, V2_STATID, V3_STATID, V4_STATID,
                                 V5_STATID, V6_STATID, V7_STATID};

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

