// =================================================================================================
// eModbus: Copyright 2024 by Michael Harwerth, Bert Melis and the contributors to eModbus
//               MIT license - see license.md for details
// =================================================================================================
#ifndef _MODBUS_BRIDGE_ETHERNET_H
#define _MODBUS_BRIDGE_ETHERNET_H
#include "options.h"
#if HAS_ETHERNET == 1
#include <ETH.h>
#include <SPI.h>

#undef SERVER_END
#define SERVER_END // NIL for Ethernet

#include "ModbusServerTCPtemp.h"
#include "ModbusBridgeTemp.h"

using ModbusBridgeEthernet = ModbusBridge<ModbusServerTCP<WiFiServer, WiFiClient>>;
#endif

#endif
