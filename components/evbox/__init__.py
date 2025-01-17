import enum
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.cpp_helpers import gpio_pin_expression
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_FLOW_CONTROL_PIN,
)
from esphome import pins

AUTO_LOAD = ["sensor"]

CONF_CUSTOM_CC = "custom_charge_current"
CONF_FALLBACK_CC = "fallback_charge_current"
CONF_MAX_CC = "max_charge_current"
CONF_MIN_CC = "min_charge_current"
CONF_VOLTAGE = "voltage"
CONF_ACTIVE_PHASES = "nb_active_phases"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_SAMPLETIME = "sampletime"
CONF_TARGET_SOLAR_POWER_CONSUMPTION = "target_solar_power_consumption"
CONF_MODE = "mode"

CODEOWNERS = ["@ajvdw", "@berndschrooten"]
DEPENDENCIES = ["uart"]

CONF_EVBOX_ID = "evbox_id"

evbox_ns = cg.esphome_ns.namespace("esphome::evbox")
EVBoxDevice = evbox_ns.class_("EVBoxDevice", uart.UARTDevice, cg.Component)

OperatingModes = evbox_ns.enum("OperatingModes")
MODE_OPTIONS = {
    "OFF": OperatingModes.MODE_OFF,
    "MIN": OperatingModes.MODE_MIN,
    "SOLAR": OperatingModes.MODE_SOLAR,
    "MAX": OperatingModes.MODE_MAX,
    "CUSTOM": OperatingModes.MODE_CUSTOM,
}

SetTargetSolarPowerConsumptionAction = evbox_ns.class_(
    "SetTargetSolarPowerConsumptionAction", automation.Action
)

SetOperatingModeAction = evbox_ns.class_("SetOperatingModeAction", automation.Action)

SetCustomChargeCurrentAction = evbox_ns.class_(
    "SetCustomChargeCurrentAction", automation.Action
)

EVBOX_COMPONENT_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.Required(CONF_EVBOX_ID): cv.use_id(EVBoxDevice),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EVBoxDevice),
            cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_VOLTAGE): cv.int_,
            cv.Required(CONF_ACTIVE_PHASES): cv.int_,
            cv.Optional(CONF_FALLBACK_CC, default=6.0): cv.float_range(
                min=1.0, max=11.0
            ),
            cv.Optional(CONF_MIN_CC, default=6.0): cv.float_range(min=1.0, max=32.0),
            cv.Optional(CONF_MAX_CC, default=16.0): cv.float_range(min=1.0, max=32.0),
            cv.Optional(CONF_KP, default=0.7): cv.float_,
            cv.Optional(CONF_KI, default=0.1): cv.float_,
            cv.Optional(CONF_KD, default=0.05): cv.float_,
            cv.Optional(CONF_SAMPLETIME, default=1.0): cv.float_,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    cg.add_global(evbox_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
    if CONF_FALLBACK_CC in config:
        cg.add(var.set_fallback_current(config[CONF_FALLBACK_CC]))
    if CONF_MIN_CC in config:
        cg.add(var.set_min_cc(config[CONF_MIN_CC]))
    if CONF_MAX_CC in config:
        cg.add(var.set_max_cc(config[CONF_MAX_CC]))
    if CONF_VOLTAGE in config:
        cg.add(var.set_voltage(config[CONF_VOLTAGE]))
    if CONF_ACTIVE_PHASES in config:
        cg.add(var.set_nb_active_phases(config[CONF_ACTIVE_PHASES]))
    if CONF_KP in config:
        cg.add(var.set_kp(config[CONF_KP]))
    if CONF_KI in config:
        cg.add(var.set_ki(config[CONF_KI]))
    if CONF_KD in config:
        cg.add(var.set_kd(config[CONF_KD]))
    if CONF_SAMPLETIME in config:
        cg.add(var.set_sampletime(config[CONF_SAMPLETIME]))

    # cg.add_library("fmtlib/fmt", None)


@automation.register_action(
    "evbox.set_target_solar_power_consumption",
    SetTargetSolarPowerConsumptionAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(EVBoxDevice),
            cv.Required(CONF_TARGET_SOLAR_POWER_CONSUMPTION): cv.templatable(cv.float_),
        }
    ),
)
async def evbox_set_target_solar_power_consumption_to_code(
    config, action_id, template_arg, args
):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(
        config[CONF_TARGET_SOLAR_POWER_CONSUMPTION], args, float
    )
    cg.add(var.set_target_solar_power_consumption(template_))
    return var


@automation.register_action(
    "evbox.set_mode",
    SetOperatingModeAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(EVBoxDevice),
            cv.Required(CONF_MODE): cv.templatable(cv.enum(MODE_OPTIONS, upper=True)),
        }
    ),
)
async def evbox_set_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_MODE], args, int)
    cg.add(var.set_mode(template_))
    return var


@automation.register_action(
    "evbox.set_custom_charge_current",
    SetCustomChargeCurrentAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(EVBoxDevice),
            cv.Required(CONF_CUSTOM_CC): cv.templatable(cv.float_),
        }
    ),
)
async def evbox_set_custom_charge_current_to_code(
    config, action_id, template_arg, args
):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_CUSTOM_CC], args, float)
    cg.add(var.set_custom_charge_current(template_))
    return var
