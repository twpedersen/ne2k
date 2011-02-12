/* NE2K driver for the TOS kernel 
 * divya, babak, and thomas, csc 720 spring 2011
 */
#include "kernel.h"


/* turn on ne2k and read data into phy */
int ne2k_init(struct ne2k_phy *phy) {

	/* turn on the card */

	/* read attributes */
	ne2k_phy.macaddr.n = 0xDEADBEEFBABE;

	wprintf(kernel_window, "ne2k loaded");
	return 0;
}

void ne2k_print_mac(WINDOW* wnd, struct ne2k_phy *phy) {

	int i;
	/* macaddr.n is little-endian, so print it backwards until we have
	 * a htonl() */
	for (i = ETH_ALEN; i > 0; i--) {
		wprintf(wnd, "%X", ne2k_phy.macaddr.byte[i - 1]);
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
