"""
init.py - Python entry point for the custom Modbus TCP component

This file defines:
  - The YAML schema (what options the user can put in `modbus_tcp:`).
  - The code-generation logic to create the corresponding C++ objects.
"""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, CONF_PORT
from esphome.cpp_types import Component, App

# Declare a new namespace for our Modbus TCP code
modbus_tcp_ns = cg.esphome_ns.namespace('modbus_tcp')
ModbusTCPComponent = modbus_tcp_ns.class_('ModbusTCPComponent', Component)

# Configuration keys (YAML)
CONF_MODBUS_TCP = 'modbus_tcp'
CONF_UNIT_ID = 'unit_id'

# Define what options are supported in the YAML
# Example usage in YAML:
#  modbus_tcp:
#    port: 502
#    unit_id: 1
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusTCPComponent),
    cv.Optional(CONF_PORT, default=502): cv.port,
    cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=247),  # typical unit ID range
})

# This is the function that processes user config -> actual C++ code
def to_code(config):
    # Create a new C++ object
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(App.register_component(var))
