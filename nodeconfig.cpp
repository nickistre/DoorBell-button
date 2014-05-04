/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "RF24Network_config.h"
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "nodeconfig.h"

// Where in EEPROM is the address stored?
uint8_t* address_at_eeprom_location = (uint8_t*)10;

// What flag value is stored there so we know the value is valid?
const uint8_t valid_eeprom_flag = 0xdf;


boolean nodeconfig_exists()
{
  return eeprom_read_byte(address_at_eeprom_location) == valid_eeprom_flag;
}

uint8_t nodeconfig_read(void)
{
  uint8_t result = 0;

  // Look for the token in EEPROM to indicate the following value is
  // a validly set node address 
  if ( nodeconfig_exists() )
  {
    // Read the address from EEPROM
    result = eeprom_read_word((uint16_t *) address_at_eeprom_location+1);
    printf_P(PSTR("ADDRESS: %u\n\r"),result);
  }
  else
  {
    printf_P(PSTR("*** No valid address found.  Send via serial to set node address(from 0 to 65535)\r\n: "));
  }
  
  return result;
}

void nodeconfig_update(uint16_t new_address) {
  eeprom_write_byte(address_at_eeprom_location,valid_eeprom_flag);
  eeprom_write_word((uint16_t*)address_at_eeprom_location+1,new_address);

  // And we are done right now (no easy way to soft reset)
  printf_P(PSTR("\n\rManually reset address 0%o\n\rPress RESET to continue!"),new_address);
}
// vim:ai:cin:sts=2 sw=2 ft=cpp
