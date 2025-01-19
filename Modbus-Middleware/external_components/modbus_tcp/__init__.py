import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import modbus
from esphome.const import CONF_ID

# Dependencies and auto-load
DEPENDENCIES = ["modbus"]
AUTO_LOAD = ["modbus_controller"]

modbus_tcp_ns = cg.esphome_ns.namespace("modbus_tcp")
ModbusTCP = modbus_tcp_ns.class_("ModbusTCP", modbus.Modbus, cg.Component)

# YAML keys
CONF_HOST = "host"
CONF_PORT = "port"
CONF_UNIT_ID = "unit_id"
CONF_MODBUS_ID = "modbus_id"

# Updated CONFIG_SCHEMA
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusTCP),
    cv.Required(CONF_HOST): cv.string,
    cv.Optional(CONF_PORT, default=502): cv.port,
    cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=247),
    cv.Required(CONF_MODBUS_ID): cv.use_id(modbus.Modbus),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))

    # Link to parent modbus_controller
    parent = await cg.get_variable(config[CONF_MODBUS_ID])
    cg.add(parent.add_device(var))

    await cg.register_component(var, config)
    await modbus.register_modbus_device(var, config)
