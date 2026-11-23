"""The ESP32 as the Tigo TAP's RS485 bus master, or a standby beside one.

The only component in this project that transmits on the bus. It owns the
RS485 uart, and tigo_monitor reads the bus through it:

    uart:
      id: rs485
      tx_pin: GPIO17
      rx_pin: GPIO18
      flow_control_pin: GPIO21   # DE/RE, for a transceiver without auto-direction
      baud_rate: 38400

    tigo_master:
      id: bus
      uart_id: rs485
      monitor_id: tigo_hub
      role: standby

    tigo_monitor:
      id: tigo_hub
      uart_id: bus

See docs/superpowers/specs/2026-10-29-esp32-bus-master-design.md.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import binary_sensor, button, sensor, switch, text, text_sensor, uart
from esphome.components.uart import CONF_DATA_BITS, CONF_PARITY, CONF_STOP_BITS
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_ID,
    CONF_STATUS,
    CONF_UART_ID,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_MILLISECOND,
    UNIT_SECOND,
)

CODEOWNERS = ["@RAR"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "switch", "text", "button", "tigo_monitor"]

tigo_master_ns = cg.esphome_ns.namespace("tigo_master")
TigoMasterComponent = tigo_master_ns.class_("TigoMasterComponent", cg.Component, uart.UARTComponent)
PvOutputSwitch = tigo_master_ns.class_("PvOutputSwitch", switch.Switch)
OptimizerInputText = tigo_master_ns.class_("OptimizerInputText", text.Text)
AddOptimizerButton = tigo_master_ns.class_("AddOptimizerButton", button.Button)
RemoveOptimizerButton = tigo_master_ns.class_("RemoveOptimizerButton", button.Button)
tigo_monitor_ns = cg.esphome_ns.namespace("tigo_monitor")
TigoMonitorComponent = tigo_monitor_ns.class_("TigoMonitorComponent")

CONF_MONITOR_ID = "monitor_id"
CONF_ROLE = "role"
CONF_TAKEOVER_AFTER = "takeover_after"
CONF_GATEWAY_ID = "gateway_id"
CONF_TAPS = "taps"
CONF_STARTUP = "startup"
CONF_HOUSEKEEPING = "housekeeping"
CONF_QUERIES = "queries"
CONF_OPTIMIZER_WRITES = "optimizer_writes"
CONF_TABLE_WRITES = "table_writes"
CONF_OPTIMIZER_INPUT = "optimizer_input"
CONF_ADD_OPTIMIZER = "add_optimizer"
CONF_REMOVE_OPTIMIZER = "remove_optimizer"
CONF_INITIALIZE_TAP = "initialize_tap"
CONF_CHANNEL = "channel"
CONF_PAN = "pan"
CONF_KEY = "key"
CONF_PROFILE = "profile"
CONF_PROBLEM = "problem"
CONF_DISCOVERY_REMAINING = "discovery_remaining"
CONF_TAKEOVERS = "takeovers"
CONF_LONGEST_PAUSE = "longest_pause"
CONF_PV_OUTPUT = "pv_output"
CONF_RESTORE_MODE = "restore_mode"


def _pv_output_view_only(config):
    # The switch shows the master's PV state, kept by the component itself; a
    # restored switch state would be a fresh command at every boot.
    if config.get(CONF_RESTORE_MODE, "DISABLED") != "DISABLED":
        raise cv.Invalid("pv_output keeps its state itself: leave restore_mode out")
    return config


def _gateway_id(value):
    # As gateway_id_ok() on the Pi: the bring-up hops the TAP to the id one up,
    # which must be neither the factory address nor past the top.
    value = cv.hex_int(value)
    if not 1 <= value <= 0x7FFE or value in (0x1234, 0x1235):
        raise cv.Invalid("a TAP id must be 0x0001-0x7FFE, and not 0x1234 or 0x1235")
    return value


def _unique_taps(value):
    if len(set(value)) != len(value):
        raise cv.Invalid("taps: each TAP once")
    return value


def _pan_id(value):
    value = cv.hex_int(value)
    if not 1 <= value <= 0xFFFE:
        raise cv.Invalid("a PAN id must be 0x0001-0xFFFE")
    return value


def _network_key(value):
    s = cv.string_strict(value).strip().replace(" ", "")
    if len(s) != 32:
        raise cv.Invalid("a network key is 16 bytes (32 hex digits)")
    try:
        raw = bytes.fromhex(s)
    except ValueError as err:
        raise cv.Invalid("a network key must be 32 hex digits") from err
    if any(b in (0x00, 0xFF) for b in raw):
        raise cv.Invalid("a network key cannot contain 0x00 or 0xFF bytes")
    return list(raw)


INITIALIZE_TAP_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_CHANNEL): cv.int_range(min=11, max=26),
        cv.Optional(CONF_PAN): _pan_id,
        cv.Optional(CONF_KEY): _network_key,
        cv.Optional(CONF_PROFILE): cv.one_of(40, 75, 120, 300, int=True),
    }
)


def _initialize_tap(value):
    if isinstance(value, bool):
        if not value:
            return None
        return INITIALIZE_TAP_SCHEMA({})
    if isinstance(value, dict):
        return INITIALIZE_TAP_SCHEMA(value)
    raise cv.Invalid("initialize_tap must be true, false, or a mapping of radio settings (channel, pan, key, profile)")


def _initialize_tap_single_tap(config):
    if config.get(CONF_INITIALIZE_TAP) is not None and len(config.get(CONF_TAPS, [])) > 1:
        raise cv.Invalid("initialize_tap does not support multiple TAPs")
    return config


def _writes_need_queries(config):
    if config[CONF_OPTIMIZER_WRITES] and not config[CONF_QUERIES]:
        raise cv.Invalid(f"{CONF_OPTIMIZER_WRITES} needs {CONF_QUERIES}: the writes follow the queries' answers")
    return config


def _buttons_need_input(config):
    if (CONF_ADD_OPTIMIZER in config or CONF_REMOVE_OPTIMIZER in config) and CONF_OPTIMIZER_INPUT not in config:
        raise cv.Invalid(f"{CONF_ADD_OPTIMIZER} and {CONF_REMOVE_OPTIMIZER} require {CONF_OPTIMIZER_INPUT}")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TigoMasterComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_MONITOR_ID): cv.use_id(TigoMonitorComponent),
            # No "listen": leave this component out, and nothing can transmit.
            cv.Optional(CONF_ROLE, default="standby"): cv.one_of("master", "standby", lower=True),
            cv.Optional(CONF_TAKEOVER_AFTER, default="8s"): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(min=cv.TimePeriod(seconds=2), max=cv.TimePeriod(seconds=60)),
            ),
            cv.Optional(CONF_GATEWAY_ID, default=0x1201): _gateway_id,
            # The TAPs to run, in polling order. Left out, a standby learns
            # them from the master it stands by for and keeps them in flash.
            cv.Optional(CONF_TAPS): cv.All(cv.ensure_list(_gateway_id), cv.Length(min=1, max=8), _unique_taps),
            cv.Optional(CONF_STARTUP, default=False): cv.boolean,
            cv.Optional(CONF_HOUSEKEEPING, default=False): cv.boolean,
            cv.Optional(CONF_QUERIES, default=False): cv.boolean,
            cv.Optional(CONF_OPTIMIZER_WRITES, default=False): cv.boolean,
            cv.Optional(CONF_TABLE_WRITES, default=False): cv.boolean,
            cv.Optional(CONF_INITIALIZE_TAP): _initialize_tap,
            cv.Optional(CONF_OPTIMIZER_INPUT): text.text_schema(
                OptimizerInputText,
                icon="mdi:barcode-scan",
                mode="TEXT",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_ADD_OPTIMIZER): button.button_schema(
                AddOptimizerButton,
                icon="mdi:plus-circle-outline",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_REMOVE_OPTIMIZER): button.button_schema(
                RemoveOptimizerButton,
                icon="mdi:minus-circle-outline",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            # What tigo_monitor's check reads off the uart it is given: the bus's.
            cv.Optional(CONF_BAUD_RATE, default=38400): cv.one_of(38400, int=True),
            cv.Optional(CONF_DATA_BITS, default=8): cv.one_of(8, int=True),
            cv.Optional(CONF_PARITY, default="NONE"): cv.one_of("NONE", upper=True),
            cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, int=True),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                icon="mdi:transit-connection-variant"
            ),
            cv.Optional(CONF_PROBLEM): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM),
            cv.Optional(CONF_DISCOVERY_REMAINING): sensor.sensor_schema(
                unit_of_measurement=UNIT_SECOND,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_TAKEOVERS): sensor.sensor_schema(
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                accuracy_decimals=0,
            ),
            # Off: the optimizers' output off, through the TAP (0x0B00 00).
            # Not rapid shutdown: the RSD cuts the TAP's power.
            cv.Optional(CONF_PV_OUTPUT): cv.All(
                switch.switch_schema(PvOutputSwitch, icon="mdi:solar-power", default_restore_mode="DISABLED"),
                _pv_output_view_only,
            ),
            cv.Optional(CONF_LONGEST_PAUSE): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLISECOND,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                accuracy_decimals=0,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    cv.only_with_framework("esp-idf"),
    _writes_need_queries,
    _initialize_tap_single_tap,
    _buttons_need_input,
)


def _no_flash_writes(config):
    """A flash write stops both cores' cache, the master's task with them, for
    up to a second (the history commits measured 0.63 s median, 1.02 s worst).
    So the device that runs the bus keeps no history and serves no web UI."""
    full = fv.full_config.get()
    if "tigo_server" in full:
        raise cv.Invalid(
            "tigo_master runs on a thin config: leave tigo_server out. Its history writes to flash, and "
            "a flash write freezes the bus master for up to a second."
        )
    components = full.get("esp32", {}).get("framework", {}).get("components", [])
    if any("esp_tsdb" in str(c) for c in components):
        raise cv.Invalid(
            "tigo_master runs on a thin config: leave esp_tsdb out of the framework components. "
            "tigo_monitor keeps its on-flash history when esp_tsdb is there, and a flash write freezes "
            "the bus master for up to a second."
        )
    return config


FINAL_VALIDATE_SCHEMA = cv.All(
    uart.final_validate_device_schema(
        "tigo_master",
        baud_rate=38400,
        require_tx=True,
        require_rx=True,
        data_bits=8,
        parity="NONE",
        stop_bits=1,
    ),
    _no_flash_writes,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    bus = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_bus(bus))
    cg.add(var.set_baud_rate(38400))
    cg.add(var.set_data_bits(8))
    cg.add(var.set_stop_bits(1))
    if CONF_MONITOR_ID in config:
        monitor = await cg.get_variable(config[CONF_MONITOR_ID])
        cg.add(var.set_monitor(monitor))
    if config[CONF_ROLE] == "standby":
        cg.add(var.set_standby(config[CONF_TAKEOVER_AFTER].total_seconds))
    cg.add(var.set_gateway_id(config[CONF_GATEWAY_ID]))
    if CONF_TAPS in config:
        cg.add(var.set_taps(config[CONF_TAPS]))
    cg.add(var.set_startup(config[CONF_STARTUP]))
    cg.add(var.set_housekeeping(config[CONF_HOUSEKEEPING]))
    cg.add(var.set_queries(config[CONF_QUERIES], config[CONF_OPTIMIZER_WRITES]))
    cg.add(var.set_table_writes(config[CONF_TABLE_WRITES]))
    if config.get(CONF_INITIALIZE_TAP) is not None:
        init_cfg = config[CONF_INITIALIZE_TAP]
        cg.add(
            var.set_initialize_tap(
                True,
                init_cfg.get(CONF_CHANNEL, 0),
                init_cfg.get(CONF_PAN, 0),
                init_cfg.get(CONF_PROFILE, 0),
                init_cfg.get(CONF_KEY, []),
            )
        )
    if CONF_STATUS in config:
        cg.add(var.set_status_sensor(await text_sensor.new_text_sensor(config[CONF_STATUS])))
    if CONF_PROBLEM in config:
        cg.add(var.set_problem_sensor(await binary_sensor.new_binary_sensor(config[CONF_PROBLEM])))
    if CONF_DISCOVERY_REMAINING in config:
        cg.add(var.set_discovery_sensor(await sensor.new_sensor(config[CONF_DISCOVERY_REMAINING])))
    if CONF_TAKEOVERS in config:
        cg.add(var.set_takeovers_sensor(await sensor.new_sensor(config[CONF_TAKEOVERS])))
    if CONF_LONGEST_PAUSE in config:
        cg.add(var.set_pause_sensor(await sensor.new_sensor(config[CONF_LONGEST_PAUSE])))
    if CONF_PV_OUTPUT in config:
        sw = await switch.new_switch(config[CONF_PV_OUTPUT])
        cg.add(sw.set_parent(var))
        cg.add(var.set_pv_switch(sw))
    if CONF_OPTIMIZER_INPUT in config:
        inp = await text.new_text(config[CONF_OPTIMIZER_INPUT])
        cg.add(var.set_optimizer_input(inp))
    if CONF_ADD_OPTIMIZER in config:
        btn = await button.new_button(config[CONF_ADD_OPTIMIZER])
        cg.add(btn.set_parent(var))
    if CONF_REMOVE_OPTIMIZER in config:
        btn = await button.new_button(config[CONF_REMOVE_OPTIMIZER])
        cg.add(btn.set_parent(var))
