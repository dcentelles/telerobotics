/*
 * Constants.h
 *
 *  Created on: 7 nov. 2016
 *      Author: diego
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace dcauv {

#define IMG_TRUNK_INFO_SIZE 2
#define IMG_FIRST_TRUNK_FLAG 0x8000
#define IMG_LAST_TRUNK_FLAG 0x4000
#define MAX_IMG_SIZE 16383 //(2^14-1)
#define MAX_IMG_TRUNK_LENGTH 250;
#define MAX_IMG_STATE_LENGTH 40
#define MAX_PACKET_LENGTH 2048
#define IMG_CHKSUM_SIZE 4

enum LinkType { fullDuplex, halfDuplex };
}

#endif /* CONSTANTS_H_ */
