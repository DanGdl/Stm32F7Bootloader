
#ifndef TFTP_H_
#define TFTP_H_

#define SIZE_TFTP_PACKET	516
#define SIZE_TFTP_HEADER	4
#define SIZE_TFTP_DATA		(SIZE_TFTP_PACKET - SIZE_TFTP_HEADER)

// TFTP op-codes
#define OP_RRQ		1
#define OP_DATA		3
#define OP_ACK		4
#define	OP_ERROR	5

#endif /* TFTP_H_ */
