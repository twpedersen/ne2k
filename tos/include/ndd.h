#ifndef _NDD_
#define _NDD_
/* ndd.h */
#include "nll.h"

/************************** NLL.H ****************/
#define NLL_MAX_PKT_LEN     1500    /* max mtu for now until we figure out how to handle fragmentation */
#define ETH_ALEN	6

/* txinfo is filled out on rx/tx */
struct txinfo {

    /* network device driver - data link layer info */
    struct {
        MACAddress dst;
        MACAddress src;
        /* give this a value above 1500 to set the type, else just nlb->len */
        unsigned short len;
    } mac_hdr;

    /* network logical layer - network layer info */
    struct {
        IPAddress dst;
        IPAddress src;
    } ip_hdr;

 /* add additional protocol headers here */
};

/* network layer buffer */
struct nl_b {
    struct txinfo *info;
    unsigned char payload[NLL_MAX_PKT_LEN];
    int head; /* grows down */
    int tail; /* grows up */
    int len;
};

/* init nlb as follows:
 * b->head = NLL_MAX_PKT_LEN / 2;
 * b->tail = b->head;
 * b->len = 0;
 */
void nl_b_init(struct nl_b *b);

/* nlb manipulation functions */
/* update head, tail, and len.
 * returns an error on doing something illegal,
 * 0 <= head <= tail < len
 * do we have some
 * errors #defined someplace in tos?
 */
int nl_b_add_head(struct nl_b *nlb, int len);
int nl_b_del_head(struct nl_b *nlb, int len);
int nl_b_add_tail(struct nl_b *nlb, int len);
int nl_b_del_tail(struct nl_b *nlb, int len);


/************ NDD.H *****************/
struct recv_ring_desc {
  unsigned char rsr;                   // Receiver status
  unsigned char next_pkt;              // Pointer to next packet
  unsigned short count;                // Bytes in packet (length + 4)
};

struct ne2k_phy {
	unsigned short nicaddr;
	unsigned short asicaddr;
	unsigned short irq;

	/* [0] is MSB */
	unsigned char macaddr[ETH_ALEN];

	/* rx / tx ring buffers */
	unsigned char rx_pstart;
	unsigned char rx_pstop;
	/* ring start / stop address */
	unsigned short rx_start;
	unsigned short rx_stop;
	/* next packet in rx ring ptr */
	unsigned char next_pkt;
};


struct ne2k_phy ne2k_phy;
struct nl_b nlb;

#endif