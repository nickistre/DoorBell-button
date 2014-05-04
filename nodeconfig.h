/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef __NODECONFIG_H__
#define __NODECONFIG_H__

boolean nodeconfig_exists();
uint8_t nodeconfig_read(void);
void nodeconfig_listen(void);
void nodeconfig_update(uint16_t new_address);

#endif // __NODECONFIG_H__
