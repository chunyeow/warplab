////////////////////////////////////////////////////////////////////////////////
// File   :	wl_user.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "wl_common.h"

#ifndef WL_USER_H_
#define WL_USER_H_

#define USER_EEPROM_WRITE_STRING 1
#define USER_EEPROM_READ_STRING 2

int user_processCmd(const wl_cmdHdr*, const void*, wl_respHdr*, void*, void*, unsigned int);
int user_init();

#endif /* WL_USER_H_ */
