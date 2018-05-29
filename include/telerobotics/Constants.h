/*
 * Constants.h
 *
 *  Created on: 7 nov. 2016
 *      Author: diego
 */

#ifndef TELEROBOTICS_CONSTANTS_H_
#define TELEROBOTICS_CONSTANTS_H_

namespace telerobotics {

static const int IMG_TRUNK_INFO_SIZE = 2;
static const int IMG_FIRST_TRUNK_FLAG = 0x8000;
static const int IMG_LAST_TRUNK_FLAG = 0x4000;
static const int MAX_IMG_SIZE = 16383; //(2^14-1)
static const int MAX_IMG_TRUNK_LENGTH = 150;
static const int MAX_NODE_STATE_LENGTH = 40;
static const int IMG_CHKSUM_SIZE = 2;
static const int MAX_PACKET_LENGTH = 2048;

enum LinkType { fullDuplex, halfDuplex };
}

#endif /* CONSTANTS_H_ */
