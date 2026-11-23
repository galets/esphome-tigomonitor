# ESP32 TAP Network Initialization & Runtime Optimizer Table Tools

## Problem Statement

An ESP32 running `tigo_master` as a standalone bus master (in place of a Tigo CCA) could previously only poll an already-provisioned TAP and manage its discovery window. Two commissioning and maintenance workflows still required either a CCA or the Linux `tigomonitord` daemon:

1. **Initial TAP Radio Provisioning**: A factory-fresh or reset TAP boots with radio channel `0` and an all-zero 16-byte network key. Until it is given a valid IEEE 802.15.4 channel (`11..26`), a PAN ID, a mesh slot map, and a 16-byte network key, no optimizers can join it.
2. **Runtime Optimizer Management**: During normal operation, an installer or homeowner using an ESPHome-only setup had no way to add a new optimizer (or replace/remove an existing one) in the TAP's flash node table without disconnecting the ESP32 and using another tool.

## Solution

Extend the ESPHome `tigo_master` component with two ESPHome-focused capabilities built on top of the existing `MasterCore` state machines (`ProvisionJob`, `TableAdd`, `TableRemove`, and `JoinWatch`):

1. **Automatic Startup Radio Initialization (`initialize_tap`)**: When enabled in YAML, `MasterCore` automatically reads the TAP's active `GatewayCfg` (`0x0D 00 00`) on startup or standby takeover. If the TAP is on factory radio settings (channel outside `11..26` or an all-zero key), it provisions the TAP with either auto-derived or YAML-configured radio parameters and commits them to flash without resetting the TAP. If the TAP is already initialized, provisioning is bypassed.
2. **Runtime Optimizer Table Add & Remove**: Gate table writes behind a `table_writes` YAML option (default `false`), expose `add_optimizer` and `remove_optimizer` C++ methods on `TigoMasterComponent` (callable from ESPHome `api: actions:` or lambdas), and provide optional Home Assistant `text` (`optimizer_input`) and `button` (`add_optimizer`, `remove_optimizer`) entities. Both interfaces accept either a printed Tigo barcode label (e.g. `4-9A57A2L`) or a 16-hex-digit MAC (`04C05B...`).

## User Stories

1. As an installer commissioning a brand-new TAP with an ESP32 bus master, I want `initialize_tap: true` to automatically detect factory radio settings on boot and configure a valid channel, PAN ID, mesh profile, and random network key, so that optimizers can join without manual CLI or CCA intervention.
2. As an installer with specific RF planning requirements, I want to pass optional overrides (`channel`, `pan`, `profile`, `key`) under `initialize_tap:`, so that I can pin exact radio parameters while letting any omitted fields be auto-derived.
3. As a homeowner rebooting my ESP32 master, I want `initialize_tap` to bypass provisioning when the TAP already has a valid channel (`11..26`) and a non-zero key, so that existing joined optimizers are never disrupted or re-keyed on reboot.
4. As an operator with an H1- or G8-firmware TAP, I want `initialize_tap` to automatically select the firmware-appropriate flash persistence mechanism (`0x0D 00 01` on H1, `CMD_STORE 0x0012` on G8) without resetting the TAP, so that settings survive power cycles.
5. As an operator running `initialize_tap` without `startup: true`, I want the master to fetch the TAP's 8-byte MAC (`GET_MAC_NOW 0x003A`) and firmware version (`VERSION 0x000A`) before provisioning, so that MAC verification, PAN derivation, and H1/G8 store detection still succeed.
6. As an operator on a multi-TAP bus, I want `initialize_tap` to refuse configuration at compile time when multiple `taps:` are listed and emit an error without writing anything if multiple TAPs are detected at runtime, so that a multi-TAP site is never misconfigured with mismatched keys or channels.
7. As a homeowner adding a new solar panel/optimizer to an active array, I want to enter its printed barcode label (e.g. `4-9A57A2L`) or 16-hex MAC in Home Assistant (or call an `api` action) and trigger an add, so that the TAP lists the optimizer and keeps the discovery window open until it joins.
8. As a homeowner removing a failed or decommissioned optimizer, I want to trigger a removal by label or MAC, so that the TAP's node table is updated without requiring a CCA or Raspberry Pi.
9. As an ESPHome user concerned about accidental bus writes, I want real table writes (`0x29` add and `0x2B` erase+rewrite) to be disabled by default (`table_writes: false`) while still allowing `dry_run` checks, so that nothing modifies the TAP's optimizer table unless explicitly permitted.
10. As an ESPHome bus master developer, I want `remove_optimizer` to back up surviving table records in RAM rather than writing to NVS/flash during bus-master operation, so that flash cache-disable stalls never interrupt active RS485 polling.

## Implementation Decisions

### 1. `initialize_tap` Schema & Overrides (`tigo_master`)
- `initialize_tap` accepts either a boolean (`true` / `false`) or a mapping of optional radio overrides:
  - `channel`: integer `11..26` (omitted → auto-selected via `channel_for`, defaulting to `15` to clear Wi-Fi channels 1, 6, and 11).
  - `pan`: hex integer `0x0001..0xFFFE` (omitted → auto-derived via `pan_for` from the TAP's MAC low 16 bits).
  - `profile`: `40`, `75`, `120`, or `300` (omitted → auto-sized / preserves a valid existing TAP profile).
  - `key`: 32-character hex string with no `0x00` or `0xFF` bytes (omitted → cryptographically random 16-byte key via `key_from`).
- Because ESPHome's schema validator validates `default=` values against field validators, optional keys in the override schema omit `default=` and resolve absent keys to auto-derive sentinels during codegen.
- Compile-time validation rejects `initialize_tap` when `taps:` contains more than one TAP.

### 2. `MasterCore` Startup Initialization State Machine
- Enabling `initialize_tap` automatically enables the initial `GatewayCfg` (`0x0D 00 00`) read on startup, takeover, and recovery from a lost TAP.
- When `startup` is `false` and the TAP's MAC is not yet known, `MasterCore` sends a single `GET_MAC_NOW` (`0x003A`) to the TAP's configured ID before proceeding.
- Factory-state detection (`is_factory_radio_`) evaluates to `true` if `channel < 11 || channel > 26` or all 16 `key` bytes are `0x00`.
- Multi-TAP runtime guard: if more than one TAP is configured/learned or an extra TAP is detected at `0x1235`, `MasterCore` logs an error once and skips initialization.
- When factory state is confirmed on a single TAP, `MasterCore` starts a `ProvisionJob` (`reset = false`, `dry_run = false`) with `saved_by_store` determined from `VERSION`. The send gate (`may_send`) permits `ProvisionJob` frames whenever `initialize_tap` is enabled, without requiring `table_writes: true`.
- If `ProvisionJob` fails (e.g., transient timeout), `MasterCore` logs the failure and does not retry in a tight loop; initialization is re-evaluated on the next restart, takeover, or recovery from lost state.

### 3. Runtime Optimizer Table Add & Remove (`TigoMasterComponent`)
- **YAML Gating & Entities**:
  - `table_writes` (boolean, default `false`) configures `MasterCore::set_table_writes()`.
  - Optional `optimizer_input` (`text` entity, `mode: TEXT`), `add_optimizer` (`button` entity), and `remove_optimizer` (`button` entity) are registered via ESPHome's `text` and `button` components (`AUTO_LOAD`), with schema validation requiring `optimizer_input` if either button is declared.
- **C++ Methods**:
  - `add_optimizer(mac_or_label, id = 0, dry_run = false, tap = 0)`
  - `remove_optimizer(mac_or_label, dry_run = false, tap = 0)`
  - A shared parser strips whitespace, decodes printed labels containing `-` via `mac_of_label()`, or parses 16-hex MACs (ignoring `:`) and validates the `04C05B` Tigo vendor prefix via `label_of()`.
  - On multi-TAP configurations, `add_optimizer` gathers active device IDs from `TigoMonitorComponent` into the `avoid` set passed to `MasterCore::request_table_add()`.
- **RAM-Only Survivor Backup**:
  - `TigoMasterComponent::setup()` registers a `TableSaved` callback with `MasterCore` that stores surviving `TableRecord`s in a mutex-guarded RAM vector rather than writing to NVS/flash.
- **Diagnostics**:
  - Validation errors, refusal reasons, job starts, and join outcomes are reported via `ESP_LOG*` without adding a separate status sensor.

## Testing Decisions

- **Highest Seam**: `MasterCore` against the in-memory `MultiTap` / `TapPort` bus simulator in the host test suite (`tests/host/master_core_test.cpp`), testing external bus behavior (frames transmitted, TAP RAM/stored radio blocks updated, log messages emitted) rather than internal helper state.
- **Covered Host Scenarios**:
  1. Factory TAP (`channel == 0`, zero key) with `startup: false` is probed for MAC/version, provisioned with auto-derived settings, and committed via `CMD_STORE` on G8.
  2. Factory TAP with explicit `channel`, `pan`, `profile`, and `key` overrides applies the exact configured values.
  3. Already-initialized TAP (`channel` in `11..26`, non-zero key) bypasses provisioning with zero radio writes.
  4. Multi-TAP bus (either multiple configured TAPs or an extra TAP answering at `0x1235`) logs an error and performs no radio writes.
- **Prior Art & Regression**: Existing `tap_config_test`, `table_edit_test`, and `master_core_test` host suites, plus Python schema compilation (`py_compile`) and ESPHome config validation against the board example configs.

## Out of Scope

- Exposing `initialize_tap` or ESPHome-specific entity bindings in the Linux daemon (`tigomonitord`).
- Multi-TAP automatic radio initialization (channel/PAN collision avoidance or key sharing across multiple TAPs via `initialize_tap`).
- NVS/flash persistence of surviving optimizer records across a power loss mid-`TableRemove` on ESP32 (intentionally kept in RAM to avoid flash-write stalls).
- Dedicated ESPHome `text_sensor` for table edit step/status reporting.
- Automatic `taps` re-addressing (`0x1235` → `gateway_id`) when `startup: false`.

## Further Notes

- Reference board configuration examples are provided in `boards/example-waveshare-relay-1ch-standby.yaml` (commented `initialize_tap` reference) and `boards/example-nodemcu-32s.yaml` (`initialize_tap`, `table_writes`, `optimizer_input`, `add_optimizer`, `remove_optimizer`, and commented `api: actions:`).
