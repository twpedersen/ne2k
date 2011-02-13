/* NE2K driver for the TOS kernel
 * divya and thomas, csc 720 spring 2011
 */
#include "kernel.h"

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
#define NE2K_CMD_STP	1 << 0
#define NE2K_CMD_STA	1 << 1
#define NE2K_CMD_TXP	1 << 2
#define NE2K_CMD_RD0	1 << 3
#define NE2K_CMD_RD1	1 << 4
#define NE2K_CMD_RD2	1 << 5
#define NE2K_CMD_PS0	1 << 6
#define NE2K_CMD_PS1	1 << 7

#define NE2K_CMD_P0		0x00
#define NE2K_CMD_P1		NE2K_CMD_PS0

/* write cmd to given register, specify page in cmd */
int ne2k_post_cmd(struct ne2k_phy *phy, unsigned char cmd,
				                        unsigned char reg) {

	outportb(phy->portaddr + reg, cmd);
	/* check for success? */
	return 0;
}

/* read register contents */
unsigned char ne2k_reg_read(struct ne2k_phy *phy,
							unsigned char reg) {

	return inportb(phy->portaddr + reg);
}

int ne2k_reg_sw_page(struct ne2k_phy *phy, int pagenum) {

	unsigned char page = NE2K_CMD_STP | NE2K_CMD_RD2;
	int err;

	switch (pagenum) {
		case 0:
			page |= NE2K_CMD_P0;
			break;
		case 1:
			page |= NE2K_CMD_P1;
			break;
		default:
			kprintf("ne2k: page not implemented\n");
			goto err_out;
	}

	if (err = ne2k_post_cmd(phy, page, NE2K_REG_CR))
		goto err_out;

	return 0;
err_out:
	return err;
}

/* the init procedure is described on p. 29 of the datasheet */
int ne2k_start(struct ne2k_phy *phy) {

	unsigned char cmd = ne2k_reg_read(phy, NE2K_REG_CR);

	/* 1) switch to page 0 */
	ne2k_reg_sw_page(phy, 0);
	/* 2) init DCR */
	/* 3) clear RBCR0 and RBCR1 */
	/* 4) init RCR */
	/* 5) place NIC in loopback mode */
	/* 6) init recv buffer ring, BNDRY, PSTART, and PSTOP */
	/* 7) clear ISR */
	cmd = 0xFF;
	ne2k_post_cmd(phy, cmd, NE2K_REG_ISR);
	/* 8) init IMR */
	/* 9) switch to page 1 and init PAR0-5, MAR0-7, and CURR */
	/* 10) put NIC in START mode */
	cmd = NE2K_CMD_STA | NE2K_CMD_RD2;
	ne2k_post_cmd(phy, cmd, NE2K_REG_CR);
	/* 11) init TCR */
	return 0;
}

int ne2k_get_attr(struct ne2k_phy *phy) {

	/* read mac address */
	int i;
	unsigned char mac_reg = NE2K_REG_PAR0;
	unsigned char page = NE2K_CMD_P1;

	/* switch to page 1 */
	ne2k_reg_sw_page(phy, 1);

	for (i = 0; i < ETH_ALEN; i++) {
		phy->macaddr.byte[i] = ne2k_reg_read(phy, mac_reg);
		mac_reg *= 2;
	}
	return 0;
}

/* turn on ne2k and fill in phy */
int ne2k_init(struct ne2k_phy *phy) {

	int err;
	phy->portaddr = NE2K_BASE_ADDR;

	/* turn on the card */
	if(err = ne2k_start(phy)) {
		kprintf("ne2k: failed to bring up interface: %d", err);
		goto err_out;
	}

	/* read attributes */
	if (err = ne2k_get_attr(phy)) {
		kprintf("ne2k: failed to get attributes: %d", err);
		goto err_out;
	};

	wprintf(kernel_window, "ne2k loaded");

	return 0;
err_out:
	return err;
}

void ne2k_print_mac(WINDOW* wnd, struct ne2k_phy *phy) {

	int i;
	/* macaddr.n is little-endian, so print it backwards until we have
	 * a htonl() */
	for (i = ETH_ALEN; i > 0; i--) {
		wprintf(wnd, "%X", phy->macaddr.byte[i - 1]);
	}
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
