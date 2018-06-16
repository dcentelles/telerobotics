/*
 * Constants.h
 *
 *  Created on: 7 nov. 2016
 *      Author: diego
 */

#ifndef TELEROBOTICS_CONSTANTS_H_
#define TELEROBOTICS_CONSTANTS_H_

namespace telerobotics {

static const int MSG_INFO_SIZE = 1;
static const int IMG_FIRST_TRUNK_FLAG = 0x80;
static const int IMG_LAST_TRUNK_FLAG = 0x40;
static const int MAX_IMG_SIZE = 16383; //(2^14-1)
static const int MAX_IMG_TRUNK_LENGTH = 150;
static const int MAX_NODE_STATE_LENGTH = 40;
static const int IMG_CHKSUM_SIZE = 2;
static const int MAX_PACKET_LENGTH = 2048;
static const int MSG_STATE_SIZE_MASK = 0x3f;
static const int DEFAULT_IMG_TRUNK_LENGTH = 40;

static const int MAX_HROVSTATE_LENGHT = 40;
static const int S100_MAX_BITRATE = 1950;

enum ARDUSUB_NAV_MODE {
  NAV_MANUAL = 0,
  NAV_STABILIZE,
  NAV_DEPTH_HOLD,
  NAV_GUIDED,
  NAV_POS_HOLD,
  NAV_UNKNOWN
};

enum LinkType { fullDuplex, halfDuplex };
}

#endif /* CONSTANTS_H_ */
