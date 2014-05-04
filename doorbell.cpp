#include "doorbell.h"
#include <Arduino.h>
#include "nodeconfig.h"

const unsigned int serial_buffer_length = 255;
char buffer[serial_buffer_length];  


void needReset() {
  printf_P(PSTR("\r\nPress RESET to continue!\r\n"));
  while(1) {
    checkSerial();
  }
}

void loadBufferLine()
{
  int bytesRead = Serial.readBytesUntil('\n', buffer, serial_buffer_length-1);
  buffer[bytesRead] = char(0); // Make sure null terminator is setup.  
}

void checkSerial() {
  // Check serial connection for any new commands
  if (Serial.available() > 0) {
    loadBufferLine();
    String command = String(buffer);
    command.trim();
    
    // Check for which command was run and handle this command
    if (command.equals("SET NODE ADDRESS")) {
      handleUpdateNodeAddress();
    }
    else if (command.equals("HELP")) {
      showHelpInfo();
    }
  }
}

void showHelpInfo() {
  printf_P(PSTR("The following commands are avaiable: \r\n"));
  
  printf_P(PSTR(" HELP\r\n"));
  printf_P(PSTR(" SET NODE ADDRESS\r\n"));
  
  printf_P(PSTR("\r\n"));
}

void handleUpdateNodeAddress() {
  // Wait until serial data is available.
  printf_P(PSTR("CHANGE NODE ADDRESS\r\n \r\n"));
  
  printf_P(PSTR("Each node must be assigned an 15-bit address by the administrator.\r\n"));
  printf_P(PSTR("This address exactly describes the position of the node within the\r\n"));
  printf_P(PSTR("tree. The address is an octal number. Each digit in the address represents a\r\n"));
  printf_P(PSTR("position in the tree further from the base.\r\n"));
  printf_P(PSTR("\r\n"));
  printf_P(PSTR("- Node 00 is the base node.\r\n"));
  printf_P(PSTR("- Nodes 01-05 are nodes whose parent is the base.\r\n"));
  printf_P(PSTR("- Node 021 is the second child of node 01.\r\n"));
  printf_P(PSTR("- Node 0321 is the third child of node 021, an so on.\r\n"));
  printf_P(PSTR("- The largest node address is 05555, so 3,125 nodes are allowed on a single channel.\r\n"));
  printf_P(PSTR("\r\n"));  
  printf_P(PSTR("Please type in the node address (from 0 to 65535)\r\n: "));
  while (Serial.available() == 0) {
    // Do nothing!
  }
  
  loadBufferLine();
  uint16_t new_node = strtoul(buffer, NULL, 8);
  printf_P(PSTR("\r\nAddress entered: %s\r\n"), buffer);
  printf_P(PSTR("Decimal address detected: %i\r\n"), new_node);
  printf_P(PSTR("Address as octal: 0%o\r\n"), new_node);
  printf_P(PSTR("Setting node address in eeprom\r\n"));
  
  // Do update of address
  nodeconfig_update(new_node);
  
  needReset();
}
