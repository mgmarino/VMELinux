/* This header file defines where the control registers are in I/O port space
 * for Concurrent Technologies VX 40x SBC series. */

#define	CONCURRENT_VX_CSR0		0x210
#define	CONCURRENT_VX_CSR2		0x211
#define	CONCURRENT_VX_CSR1		0x212
#define	CONCURRENT_VX_VME_ADDR_CAPT	0x213
#define	CONCURRENT_VX_WATCHDOG_CSR	0x214
#define	CONCURRENT_VX_LONG_DUR_TMR_LS	0x218
#define	CONCURRENT_VX_LONG_DUR_TMR_ML	0x219
#define	CONCURRENT_VX_LONG_DUR_TMR_MH	0x21A
#define	CONCURRENT_VX_LONG_DUR_TMR_MS	0x21B
#define	CONCURRENT_VX_LONG_DUR_TMR_CSR	0x21C
#define	CONCURRENT_VX_CSR3		0x21D
#define	CONCURRENT_GPIO_CSR		0x31C
#define	CONCURRENT_SERIAL_IO_CSR	0x31D
#define	CONCURRENT_VME_SLOT_ID_REG	0x31E

#define CONCURRENT_HW_BYTE_SWAP_MASTER  0x8
#define CONCURRENT_HW_BYTE_SWAP_SLAVE   0x10
#define CONCURRENT_HW_BYTE_SWAP_FAST    0x20
	
#ifdef MODULE
static int concurrent_ioports_permissions(uint16_t port) 
{
	if ( ( port >= CONCURRENT_VX_CSR0 && port <= CONCURRENT_VX_CSR3 ) ||
	     ( port >= CONCURRENT_GPIO_CSR && port <= CONCURRENT_VME_SLOT_ID_REG ) ) {
		return 0;
	} else return -1;
}	
#endif
