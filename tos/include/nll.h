#ifndef ARP_H_
#define ARP_H_
#include "kernel.h"


//define a struct to hold the arp fields


/********************** Network Logical Layer ***********************/

/******************ARP PACKET *******************/

/*
Internet Protocol (IPv4) over Ethernet ARP packet
bit offset 	0 – 7 	8 – 15
0 	Hardware type (HTYPE)
16 	Protocol type (PTYPE)
32 	Hardware address length (HLEN) 	Protocol address length (PLEN)
48 	Operation (OPER)
64 	Sender hardware address (SHA) (first 16 bits)
80 	(next 16 bits)
96 	(last 16 bits)
112 	Sender protocol address (SPA) (first 16 bits)
128 	(last 16 bits)
144 	Target hardware address (THA) (first 16 bits)
160 	(next 16 bits)
176 	(last 16 bits)
192 	Target protocol address (TPA) (first 16 bits)
208 	(last 16 bits)
*/

// get some defines in for the default types - Assuming Ethernet, IPV4

#define ARP_DEF_HTYPE  0x0001 // Hardware Type default = Ethernet.
#define ARP_DEF_PTYPE  0x0800 // Protocol Type default = IPV4.
#define ARP_DEF_HLEN  0x0006 // Hardware Address Length default = 6 bytes.
#define ARP_DEF_PLEN  0x0004 // Protocol Address Length default = 4 bytes.
#define ARP_OP_REQUEST  0x0001 // request operation.
#define ARP_OP_REPLY  0x0002 // reply operation.


#define MAGIC_ARP 0xDEADBEEF


typedef BYTE MACAddress[6];
typedef BYTE IPAddress[4];

#define LITTLE_ENDIAN 0
#define BIG_ENDIAN 1
extern int endianess;
int get_endianess();

// this is like __be32 in linux/types.h
typedef LONG BE_LONG;
// following in_addr in linux/in.h
typedef BE_LONG IP_ADDR;
// just like __be16 in linux/types.h
typedef WORD BE_WORD;
typedef BE_WORD UDP_PORT;

// "public" methods
BE_WORD nll_htons(WORD val);
BE_LONG nll_htonl(LONG val);
MACAddress* nll_get_my_mac();
IPAddress* nll_get_my_ip();


// move to an include file later.
typedef struct
{
	WORD HTYPE;
	WORD PTYPE;
	BYTE HLEN;
	BYTE PLEN;
	WORD OPER;
	MACAddress SHA;
	IPAddress SPA;
	MACAddress THA;
	MACAddress TPA;
} ARPPDU;

MACAddress DEF_MAC_ADDRESS;
MACAddress LOCAL_MAC_ADDRESS;

IPAddress DEF_IP_ADDRESS;
IPAddress LOCAL_IP_ADDRESS;



/********************** nll_arppacket.c *******************/
void PrintArpPacket(ARPPDU* pdu);
ARPPDU* encodeARP(WORD operation, MACAddress *targetmac, IPAddress *targetip);
ARPPDU* decodeARP(BYTE* bufferIn);

/************ ARP TABLE *****************/
#define MAX_ARPRECS  20
#define ARP_TTL_DEF 0x0000FFFF

struct _ARB;
typedef struct _ARB* ARPBLOCK;

typedef struct _ARB
{
	unsigned magic;
	unsigned used;
	// The hardware level address - 6 byte MAC address
	MACAddress mac;
	// Targets Resolved Address e.g. 192.168.0.2 etc
	IPAddress ip;
	ARPBLOCK prev;
	ARPBLOCK next;
	unsigned int TTL;
} ARB;


// we have an arp cache that stores the latest target addresses,
// along with their resolved hardware addresses.

extern ARB arptable[];

ARPBLOCK nll_arp_table_findfreeblock();
ARPBLOCK nll_create_arpblock (MACAddress* mac, IPAddress* ip);
void nll_arptable_print_table_heading();
void nll_arptable_print_block_details(ARPBLOCK a);
void nll_arp_table_print();
void nll_arp_inittable();
int nll_arp_table_find(ARPBLOCK arp);
int nll_arp_table_remove(ARPBLOCK arp);
void nll_arp_table_refresh();


/********************** nll utils.c *******************/
int nll_utils_copybuf(char* src, char* dest, int len);
int nll_utils_copybytebuf(BYTE* src, BYTE* dest, int len);


#endif /*ARP_H_*/
