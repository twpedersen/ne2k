/* NE2K driver for the TOS kernel
 * divya and thomas, csc 720 spring 2011
 */
#include "kernel.h"
#include "ndd.h"

#define NE2K_BASE_ADDR	0x300

/* register address assignments, a description of these can be found in
 * docs/DP8390D.pdf */
#define NE2K_REG_CR	0x00

/* page 0 (PS1,PS0) = (0,0) */
/* read: */
#define NE2K_REG_CLDA0	0x01
#define NE2K_REG_CLDA1	0x02
#define NE2K_REG_BNRY	0x03
#define NE2K_REG_TSR	0x04
#define NE2K_REG_NCR	0x05
#define NE2K_REG_FIFO	0x06
#define NE2K_REG_ISR	0x07
#define NE2K_REG_CRDA0	0x08
#define NE2K_REG_CRDA1	0x09
#define NE2K_REG_RSR	0x0C
#define NE2K_REG_CNTR0	0x0D
#define NE2K_REG_CNTR1	0x0E
#define NE2K_REG_CNTR2	0x0F

/* write: */
#define NE2K_REG_PSTART	0x01
#define NE2K_REG_PSTOP	0x02
#define NE2K_REG_TPSR	0x04
#define NE2K_REG_TBCR0	0x05
#define NE2K_REG_TBCR1	0x06
#define NE2K_REG_RSAR0	0x08
#define NE2K_REG_RSAR1	0x09
#define NE2K_REG_RBCR0	0x0A
#define NE2K_REG_RBCR1	0x0B
#define NE2K_REG_RCR	0x0C
#define NE2K_REG_TCR	0x0D
#define NE2K_REG_DCR	0x0E
#define NE2K_REG_IMR	0x0F

/* page 1 (PS1,PS0) = (0,1) */
#define NE2K_REG_PAR0	0x01
#define NE2K_REG_PAR1	0x02
#define NE2K_REG_PAR2	0x03
#define NE2K_REG_PAR3	0x04
#define NE2K_REG_PAR4	0x05
#define NE2K_REG_PAR5	0x06
#define NE2K_REG_CURR	0x07
#define NE2K_REG_MAR0	0x08
#define NE2K_REG_MAR1	0x09
#define NE2K_REG_MAR2	0x0A
#define NE2K_REG_MAR3	0x0B
#define NE2K_REG_MAR4	0x0C
#define NE2K_REG_MAR5	0x0D
#define NE2K_REG_MAR6	0x0E
#define NE2K_REG_MAR7	0x0F

/* register command sets */
/* command register */
#define NE2K_CR_STP	0x01
#define NE2K_CR_STA	0x02
#define NE2K_CR_TXP	0x04
#define NE2K_CR_RD0	0x08
#define NE2K_CR_RD1	0x10
#define NE2K_CR_RD2	0x20
#define NE2K_CR_PS0	0x40
#define NE2K_CR_PS1	0x80

/* data configuration */
#define NE2K_DCR_WTS	0x01
#define NE2K_DCR_LS		0x08
#define NE2K_DCR_FT1	0x40

/* receive configuration register */
#define NE2K_RCR_SEP	0x01
#define NE2K_RCR_AR		0x02
#define NE2K_RCR_AB		0x04
#define NE2K_RCR_AM		0x08
#define NE2K_RCR_PRO	0x10
#define NE2K_RCR_MON	0x20

/* transmission control */
#define NE2K_TCR_LB0	0x02
#define NE2K_TCR_LB1	0x04

//
// Interrupt Status Register (ISR)
//

#define NE2K_ISR_PRX      0x01           // Packet Received
#define NE2K_ISR_PTX      0x02           // Packet Transmitted
#define NE2K_ISR_RXE      0x04           // Receive Error
#define NE2K_ISR_TXE      0x08           // Transmission Error
#define NE2K_ISR_OVW      0x10           // Overwrite
#define NE2K_ISR_CNT      0x20           // Counter Overflow
#define NE2K_ISR_RDC      0x40           // Remote Data Complete
#define NE2K_ISR_RST      0x80           // Reset status

//
// Interrupt Mask Register (IMR)

#define NE2K_IMR_PRXE     0x01           // Packet Received Interrupt Enable
#define NE2K_IMR_PTXE     0x02           // Packet Transmit Interrupt Enable
#define NE2K_IMR_RXEE     0x04           // Receive Error Interrupt Enable
#define NE2K_IMR_TXEE     0x08           // Transmit Error Interrupt Enable
#define NE2K_IMR_OVWE     0x10           // Overwrite Error Interrupt Enable
#define NE2K_IMR_CNTE     0x20           // Counter Overflow Interrupt Enable
#define NE2K_IMR_RDCE     0x40           // Remote DMA Complete Interrupt Enable

/* check these values */
#define NE2K_ASIC_OFFSET	0x10
#define NE2K_NOVELL_RESET	0x0F
#define NE2K_NOVELL_DATA	0x00

/* local RAM usage (high bytes),
 * assume 32KB of local ram */
#define NE2K_PAGE_SIZE		0x100
/* our pages start here */
#define NE2K_MEMBASE		16 * 1024
#define NE2K_MEMSIZE		16 * 1024
#define NE2K_TX_SIZE		6
#define NE2K_TX_BUFS		2
#define NE2K_PSTART			NE2K_MEMBASE / NE2K_PAGE_SIZE
#define NE2K_PSTOP			NE2K_PSTART + NE2K_MEMSIZE / NE2K_PAGE_SIZE - NE2K_TX_SIZE + NE2K_TX_BUFS
#define NE2K_TXPSTART		NE2K_PSTOP


unsigned short htons(unsigned short s) {
	int r;
	r = s << 8;
	r |= s >> 8;
	return r;
}

/* these functions seem sort of redundant.. */

/* write byte to given register, switch page before calling this */
int ne2k_reg_write(struct ne2k_phy *phy, unsigned char reg,
				                        unsigned char byte) {

	outportb(phy->nicaddr + reg, byte);
	return 0;
}

/* read register contents */
unsigned char ne2k_reg_read(struct ne2k_phy *phy,
							unsigned char reg) {

	return inportb(phy->nicaddr + reg);
}

/* read len number of bytes from NIC buffer memory at src,
 * NIC should be initialized and running before calling this.
 * stolen almost verbatim from sanos, src should be word-aligned */
void ne2k_read_mem(struct ne2k_phy *phy, unsigned short src, void *dst,
										 unsigned short len) {
	/* align words */
	if (len & 1) len++;

	/* finish DMA */
	ne2k_reg_write(phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STA);

	/* start reading at src */
	ne2k_reg_write(phy, NE2K_REG_RSAR0, src);
	ne2k_reg_write(phy, NE2K_REG_RSAR1, src >> 8);

	/* for len bytes */
	ne2k_reg_write(phy, NE2K_REG_RBCR0, len);
	ne2k_reg_write(phy, NE2K_REG_RBCR1, len >> 8);

	/* select remote DMA read */
	ne2k_reg_write(phy, NE2K_REG_CR, NE2K_CR_RD0 | NE2K_CR_STA);

	/* do 16-bit DMA read */
	insw(phy->asicaddr, dst, len >> 1);
}

/* return current page ne2k is on */
unsigned char ne2k_reg_get_page(struct ne2k_phy *phy) {

	unsigned char page = ne2k_reg_read(phy, NE2K_REG_CR);
	return page >> 6;
}

/* Performs a safe page switch, card is always in page 0 under normal operation  */
int ne2k_reg_sw_page(struct ne2k_phy *phy, int pagenum) {

	unsigned char page = ne2k_reg_read(phy, NE2K_REG_CR);
	int err;

	switch (pagenum) {
		case 0:
			page &= ~NE2K_CR_PS0;
			page &= ~NE2K_CR_PS1;
			break;
		case 1:
			page |= NE2K_CR_PS0;
			page &= ~NE2K_CR_PS1;
			break;
		case 2:
			page &= ~NE2K_CR_PS0;
			page |= NE2K_CR_PS1;
			break;
		default:
			kprintf("ne2k: page not implemented\n");
			goto err_out;
	}

	if (err = ne2k_reg_write(phy, NE2K_REG_CR, page))
		goto err_out;

	return 0;
err_out:
	return err;
}

/* print registers for current page */
void ne2k_reg_hexdump(struct ne2k_phy *phy) {

	int i;
	int page = ne2k_reg_get_page(phy);
	kprintf("\nne2k page #%d: ", page);
	for (i = 0; i <= 0x0F; i++) {
		kprintf("\n%02X %02X ", i, ne2k_reg_read(phy, i));
	}
}

/* read pending packets off rx ring */
void ne2k_rx() {

	struct recv_ring_desc rx_hdr;
	unsigned short len;
	unsigned short pkt_ptr = ne2k_phy.next_pkt * NE2K_PAGE_SIZE;

	/* all the info we need is in the first 4 bytes of packet */
	kprintf("reading at: %02X\n", pkt_ptr);
	ne2k_read_mem(&ne2k_phy, pkt_ptr, &rx_hdr, sizeof(rx_hdr));
	kprintf("next_pkt: %02X\n", rx_hdr.next_pkt);
	kprintf("count: %02X\n", rx_hdr.count);
	kprintf("rsr: %02X\n", rx_hdr.rsr);
	len = rx_hdr.count - 4;

	/* finish DMA..? */
	//ne2k_reg_write(&ne2k_phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STA);

	/* should be getting length of packet first and then calculate head,*/
	nlb.head = 0;
	nlb.len = len;
	kprintf("LEN: %d\n", len);
	nlb.tail = nlb.head + nlb.len;
	ne2k_read_mem(&ne2k_phy, pkt_ptr, (void *)&(nlb.payload[nlb.head]), len);

	/* update pointers */
	/*
	bndry = ne2k_phy.next_pkt - 1;
	if (bndry < NE2K_PSTART)
		ne2k_reg_write(&ne2k_phy, NE2K_REG_BNRY, NE2K_PSTOP - 1);
	else
		ne2k_reg_write(&ne2k_phy, NE2K_REG_BNRY, ne2k_phy.next_pkt - 1);
		*/
}

void ne2k_handle_irq() {

	volatile unsigned char isr;

	do {
		kprintf("handling irqs\n");

		/* packet ready for rx */
		if (isr & NE2K_ISR_PRX) {
			kprintf("packet received");

			/* reset interrupt */
			ne2k_reg_write(&ne2k_phy, NE2K_REG_ISR, NE2K_ISR_PRX);
		    ne2k_rx();
		}

		/* packet txed */
		if (isr & NE2K_ISR_PTX) {
			kprintf("Packet transmitted");

			/* reset interrupt */
			ne2k_reg_write(&ne2k_phy, NE2K_REG_ISR, NE2K_ISR_PTX);
		}

		/* pkt tx error */
		if (isr & NE2K_ISR_TXE) {
			kprintf("packet tx error");

			/* reset interrupt */
			ne2k_reg_write(&ne2k_phy, NE2K_REG_ISR, NE2K_ISR_TXE);
		}

		/* remote DMA complete */
		if (isr & NE2K_ISR_RDC) {
			kprintf("Remote DMA Completed");
			int i;
			for(i = nlb.head; i < nlb.tail; i++) {
				if (i % 16 == 0) kprintf("\n");
				kprintf("%02X", nlb.payload[i]);
			}

			/* reset interrupt and set DMA complete */
			ne2k_reg_write(&ne2k_phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STA);
			ne2k_reg_write(&ne2k_phy, NE2K_REG_ISR, NE2K_ISR_RDC);
		}

		isr = ne2k_reg_read(&ne2k_phy, NE2K_REG_ISR);
		kprintf("isr: %d", isr);
	} while (isr);

	kprintf("\n***********IRQ SERVICED!***********");
}

void ne2k_isr() {

	asm ("push %eax; push %ecx; push %edx");
	asm ("push %ebx; push %ebp; push %esi; push %edi");

	ne2k_handle_irq();

	asm ("movb $0x20,%al");
	asm ("outb %al,$0x20");
	asm ("pop %edi; pop %esi; pop %ebp; pop %ebx");
	asm ("pop %edx; pop %ecx; pop %eax");
	asm ("iret");
}

/* try to detect presence of ne2k at phy->nicaddr, copied from ne2k.c
 * in sanos (jbox.dk/sanos) */
static int ne2k_probe(struct ne2k_phy *phy)
{
	unsigned char byte;

	/* reset */
	byte = inportb(phy->asicaddr + NE2K_NOVELL_RESET);
	outportb(phy->asicaddr + NE2K_NOVELL_RESET, byte);
	outportb(phy->nicaddr + NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STP);

	sleep(50);

	// Test for a generic DP8390 NIC
	byte = inportb(phy->nicaddr + NE2K_REG_CR);
	byte &= NE2K_CR_RD2 | NE2K_CR_TXP | NE2K_CR_STA | NE2K_CR_STP;
	if (byte != (NE2K_CR_RD2 | NE2K_CR_STP)) return 1;

	byte = inportb(phy->nicaddr + NE2K_REG_ISR);
	byte &= NE2K_ISR_RST;
	if (byte != NE2K_ISR_RST) return 1;

	return 0;
}


/* the init procedure is described on p. 29 of the datasheet, as well as the
 * driver reference implementation in docs/writingdriversfortheDP8390.pdf */
int ne2k_start(struct ne2k_phy *phy) {

	unsigned char macbuf[16];
	int i;
	int err;

	if ((err = ne2k_probe(phy)))
		goto err_out;

	kprintf("ne2k: probe successful\n");

	/* 1) stop mode 0x21, abort DMA and stop card */
	ne2k_reg_write(phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STP);

	/* 2) init DCR,  FIFO rx threshold 8 bytes, normal loopback (off),
	 * and 16-bit wide DMA transfers */
	ne2k_reg_write(phy, NE2K_REG_DCR, NE2K_DCR_FT1 | NE2K_DCR_LS | NE2K_DCR_WTS);

	/* get mac */
	ne2k_read_mem(phy, NE2K_NOVELL_DATA, (void *) macbuf, 16);
	for (i = 0; i < ETH_ALEN; i++)
		phy->macaddr[i] = macbuf[i * 2];
	ne2k_reg_write(phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STP);

	/* 3) clear RBCR0 and RBCR1 */
	ne2k_reg_write(phy, NE2K_REG_RBCR0, 0x00);
	ne2k_reg_write(phy, NE2K_REG_RBCR1, 0x00);

	/* 4) init rx configuration register, accept bcast */
	ne2k_reg_write(phy, NE2K_REG_RCR, NE2K_RCR_AB | NE2K_RCR_AR);

	/* 5) place NIC in internal loopback mode 1 */
	ne2k_reg_write(phy, NE2K_REG_TCR, NE2K_TCR_LB0);

	/* set tx buffer page start addr */
	ne2k_reg_write(phy, NE2K_REG_TPSR, NE2K_TXPSTART);

	/* 6) init recv buffer ring, BNRY, PSTART, and PSTOP */
	ne2k_reg_write(phy, NE2K_REG_PSTART, phy->rx_pstart);
	ne2k_reg_write(phy, NE2K_REG_BNRY, phy->rx_pstart);
	ne2k_reg_write(phy, NE2K_REG_PSTOP, phy->rx_pstop);

	/* 7) clear ISR */
	ne2k_reg_write(phy, NE2K_REG_ISR, 0xFF);

	/* 8) init IMR */
	ne2k_reg_write(phy, NE2K_REG_IMR, NE2K_IMR_RDCE | NE2K_IMR_TXEE | NE2K_IMR_PTXE | NE2K_IMR_PRXE);

	/* 9) switch to page 1 and init PAR0-5, MAR0-7, and CURR */
	ne2k_reg_sw_page(phy, 1);
	/* write mac */
	for (i = 0; i < ETH_ALEN; i++)
		ne2k_reg_write(phy, NE2K_REG_PAR0 + i, phy->macaddr[i]);
	ne2k_reg_write(phy, NE2K_REG_CURR, phy->next_pkt);

	/* 10) put NIC in START mode, back in page 0 */
	ne2k_reg_write(phy, NE2K_REG_CR, NE2K_CR_RD2 | NE2K_CR_STA);

	/* 11) init TCR in normal mode */
	ne2k_reg_write(phy, NE2K_REG_TCR, 0x00);

	/* install interrupt handler */
	//init_idt_entry(NE2K_IRQ + 0x60, ne2k_isr);

	return 0;
err_out:
	return err;
}

/* turn on ne2k and fill in phy */
int ne2k_init(struct ne2k_phy *phy) {

	int err;
	phy->nicaddr = NE2K_BASE_ADDR;
	phy->asicaddr = phy->nicaddr + NE2K_ASIC_OFFSET;

	/* rx ring */
	phy->rx_pstart = NE2K_PSTART;
	phy->rx_pstop = NE2K_PSTOP;
	phy->rx_start = phy->rx_pstart * NE2K_PAGE_SIZE;
	phy->rx_stop = phy->rx_pstop * NE2K_PAGE_SIZE;

	phy->next_pkt = phy->rx_pstart + 1;

	/* turn on the card */
	if(err = ne2k_start(phy)) {
		kprintf("ne2k: failed to bring up interface: %d", err);
		goto err_out;
	}


	return 0;
err_out:
	return err;
}


void ne2k_print_mac(WINDOW* wnd) {

	struct ne2k_phy *phy = &ne2k_phy;
	int i;
	for (i = 0; i < ETH_ALEN; i++) {
		wprintf(wnd, "%02X", phy->macaddr[i]);
		if (i != ETH_ALEN - 1)
			wprintf(wnd, ":");
	}

	int page = ne2k_reg_get_page(phy);
	ne2k_reg_sw_page(phy, 2);
	ne2k_reg_hexdump(phy);
	ne2k_reg_sw_page(phy, page);
	//ne2k_reg_hexdump(phy);
}

void ne2k_process(PROCESS self, PARAM param) {

	while (1) {
		/* driver stuff */
	}
}

void init_ne2k() {

	int err;
	if (err = ne2k_init(&ne2k_phy)) {
		kprintf("ne2k: couldn't bring up card! error %d\n", err);
		return;
	}

	/* give NE2000 priority 1 for now since the scheduler isn't preemptive
	 * and we busy wait */
	create_process(ne2k_process, 1, 0, "NE2000");
	resign();
}
