# Standalone TAP Network & Optimizer Configuration Guide (`CONFIGURE.md`)

This document specifies the exact RS485 serial wire protocol, framing, data structures, and state machines extracted from the TigoMonitor codebase (`components/tigo_monitor/tigo_protocol.h`, `components/tigo_monitor/tigo_master.h`, `components/tigo_monitor/tap_config.h`, `components/tigo_monitor/table_edit.h`, and `components/tigo_master/master_core.cpp`).

It provides everything required to build a standalone CLI tool that connects directly to a Tigo Access Point (TAP) over a TTY / RS485 serial adapter and performs:
1. **TAP Discovery & RS485 Addressing** (moving a TAP from factory `0x1235` to `0x1201` and committing to flash)
2. **Building a Network ("Add a TAP's radio step")** (configuring the 802.15.4 channel, PAN ID, mesh slot map, and 16-byte network key, and persisting across power cycles on both H1 and G8 firmware)
3. **Adding an Optimizer ("Add an optimizer")** (converting printed barcode labels to 8-byte MACs, walking the node table, appending a provisional record, and holding the discovery window open until the optimizer joins)

---

## 1. Physical Layer & Serial Port Setup

The TAP communicates over half-duplex RS485 at **38400 baud, 8N1, raw mode, no flow control**.

### POSIX `termios` Configuration
From `linux/src/serial_port.cpp`:
- **Baud Rate**: `38400` (`B38400`)
- **Flags**:
  - Open with `O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK` initially so `open()` does not block waiting for DCD carrier, then clear `O_NONBLOCK` via `fcntl` after configuring `CLOCAL`.
  - Apply `cfmakeraw(&tio)`.
  - `tio.c_cflag |= (CLOCAL | CREAD);`
  - `tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);`
  - `tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;`
  - Flush input buffer with `tcflush(fd, TCIFLUSH)`.
- **Write & Drain**: After writing a frame to the file descriptor, call `tcdrain(fd)` to wait until all bytes leave the UART TX buffer before waiting for the reply.
- **Turnaround & Timeouts**: Normal TAP turnaround is 10–13 ms (spikes to ~140 ms). Standard poll/command timeout is `50 ms`, table page read timeout is `500 ms`, radio/table write and flash operations use `2000–3000 ms`.

---

## 2. Wire Framing, Escaping & CRC-16

All communication uses framed packets delimited by `7E 07` (start) and `7E 08` (end). Master requests are preceded by a 3-byte preamble `00 FF FF`.

### 2.1 Unescaped Body Layout
Before escaping, a frame body consists of:
```text
+---------------+---------------+-----------------------+---------------+
|  DEST (2B BE) |  CMD (2B BE)  |   PAYLOAD (0..N B)    |  CRC (2B)     |
+---------------+---------------+-----------------------+---------------+
```
- **Master → TAP Request**: `DEST` has bit 15 clear (e.g., `0x1201` for configured TAP 1, `0x1235` for factory address, `0x0000` for broadcast).
- **TAP → Master Reply**:
  - `DEST` has bit 15 set (`dest | 0x8000`, e.g., `0x9201`, `0x9235`, or `0x8000` for broadcast reply).
  - `CMD` is `request_cmd + 1` (e.g., request `0x0B0F` → reply `0x0B10`; request `0x003A` → reply `0x003B`).
  - **Exception (NAK)**: If the TAP refuses a command, it replies with `CMD = 0x0006` and an empty payload.

### 2.2 CRC-16 Calculation
From `components/tigo_monitor/tigo_protocol.h`:
- **Algorithm**: Reflected CRC-16/CCITT
- **Polynomial**: `0x8408`
- **Initial Value**: `0x8408`
- **Wire Byte Order**: **Little-endian** (low byte first, high byte second on the wire). *(Note: `protocol::crc16()` in C++ byte-swaps its return value and pushes high-then-low, which places the raw CRC low byte first on the wire).*

```python
def crc16_wire_bytes(data: bytes) -> bytes:
    crc = 0x8408
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    # Low byte first on the wire
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])
```

### 2.3 Byte Stuffing (Escaping)
Every byte in `DEST + CMD + PAYLOAD + CRC` must be escaped if it matches one of the 7 reserved values:

| Raw Byte | Escaped Sequence |
| :--- | :--- |
| `0x7E` | `7E 00` |
| `0x24` | `7E 01` |
| `0x23` | `7E 02` |
| `0x25` | `7E 03` |
| `0xA4` | `7E 04` |
| `0xA3` | `7E 05` |
| `0xA5` | `7E 06` |

### 2.4 Complete Frame Assembly & Decoding

```python
ESCAPED = [0x7E, 0x24, 0x23, 0x25, 0xA4, 0xA3, 0xA5]

def encode_frame(dest: int, cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([dest >> 8, dest & 0xFF, cmd >> 8, cmd & 0xFF]) + payload
    body += crc16_wire_bytes(body)
    out = bytearray(b"\x00\xFF\xFF\x7E\x07")
    for b in body:
        if b in ESCAPED:
            out.extend([0x7E, ESCAPED.index(b)])
        else:
            out.append(b)
    out.extend(b"\x7E\x08")
    return bytes(out)

def decode_frame(escaped_between_7e07_and_7e08: bytes):
    out, i = bytearray(), 0
    b = escaped_between_7e07_and_7e08
    while i < len(b):
        if b[i] == 0x7E and i + 1 < len(b) and b[i + 1] < 7:
            out.append(ESCAPED[b[i + 1]])
            i += 2
        else:
            out.append(b[i])
            i += 1
    if len(out) < 6:
        return None
    if crc16_wire_bytes(out[:-2]) != bytes(out[-2:]):
        return None
    dest = (out[0] << 8) | out[1]
    cmd = (out[2] << 8) | out[3]
    payload = bytes(out[4:-2])
    return dest, cmd, payload
```

### 2.5 Tunnel Framing (`0x0B0F` / `0x0B10`)
Most radio and optimizer table operations use the tunnel command (`CMD = 0x0B0F`, reply `CMD = 0x0B10`):
- **Tunnel Request Payload**:
  ```text
  00 00 00 <SUB: 1B> <SEQ: 1B> <ARGS: 0..N B>
  ```
  - `SEQ` is a 1-byte counter (`0x01`..`0xFE`, wrapping from `0xFE` to `0x01`; never `0x00` or `0xFF`).
- **Tunnel Reply Payload**:
  ```text
  00 <0D or 0E> 00 <SUB + 1: 1B> <SEQ: 1B> <DATA: 0..N B>
  ```
  - The TAP echoes `SEQ` in byte 4, and sets byte 3 to `request_sub + 1`.

---

## 3. Preliminary Step: Discovering & Addressing the TAP

A factory-fresh TAP or an unstored TAP boots at the universal factory RS485 address **`0x1235`**, whereas normal operation uses **`0x1201`** (or `0x1202`+ for multi-TAP buses). All management commands (`0x0010`, `0x0012`, `0x0014`, `0x003C`) require the fixed 4-byte setup magic key:
```text
SETUP_KEY = 37 24 92 66
```

### Step-by-Step Addressing & Identification Sequence
1. **Probe Configured Address (`0x1201`)**:
   - Send `GET_MAC_NOW` (`DEST = 0x1201`, `CMD = 0x003A`, `PAYLOAD = empty`, timeout `50–500 ms`).
   - If the TAP answers (`DEST = 0x9201`, `CMD = 0x003B`, `PAYLOAD = <8-byte MAC><2-byte RS485 ID BE>`), record its 8-byte `TAP_MAC` and `TAP_ID = 0x1201`.
2. **Probe Factory Address (`0x1235`)** (if no reply at `0x1201` after 3 tries):
   - Send `GET_MAC_NOW` (`DEST = 0x1235`, `CMD = 0x003A`, `PAYLOAD = empty`).
   - If still no reply and only one TAP is on the bench bus, you can pull it to `0x1235` via a broadcast `SET_485_ID`:
     - Request: `DEST = 0x0000`, `CMD = 0x0014`, `PAYLOAD = 37 24 92 66 12 35`
     - Reply: `DEST = 0x8000`, `CMD = 0x0015`
     - Follow with `GET_MAC` (`DEST = 0x1235`, `CMD = 0x0038`, empty payload, timeout `800 ms`) or `GET_MAC_NOW` (`0x003A`).
3. **Assign RS485 ID (`0x1201`) & Commit to Flash**:
   - **Set ID by MAC**:
     - Request: `DEST = 0x1235`, `CMD = 0x003C`, `PAYLOAD = 37 24 92 66 + <8-byte TAP_MAC> + 12 01`
     - Reply: `DEST = 0x9235`, `CMD = 0x003D`
   - **Verify at `0x1201`**:
     - Request: `DEST = 0x1201`, `CMD = 0x003A`, `PAYLOAD = empty`
     - Reply: `DEST = 0x9201`, `CMD = 0x003B`, payload `<8-byte TAP_MAC> 12 01`
   - **Store ID to Non-Volatile Flash** (so it survives power cycles and resets):
     - Request: `DEST = 0x1201`, `CMD = 0x0012` (`CMD_STORE`), `PAYLOAD = 37 24 92 66`
     - Reply: `DEST = 0x9201`, `CMD = 0x0013`
4. **Read Firmware Version (`0x000A`)**:
   - **Crucial for Step 4**: You must know whether the TAP runs **H1** or **G8** firmware because they save radio settings differently!
   - Request: `DEST = 0x1201`, `CMD = 0x000A`, `PAYLOAD = empty`
   - Reply: `DEST = 0x9201`, `CMD = 0x000B`, `PAYLOAD = ASCII string` (e.g., `"Mgate Version G8.65 ..."` or `"Mgate Version H1.0007 ..."`).
   - Check if the version string starts with `"G8."` or contains `" G8."` (`saves_with_store(version)`).
5. **Turn PV Output On (`0x0B00`)** (Bring-up session frame):
   - Request: `DEST = 0x1201`, `CMD = 0x0B00`, `PAYLOAD = 01`
   - Reply: `DEST = 0x9201`, `CMD = 0x0B01`, `PAYLOAD = 01`

---

## 4. Operation 1: Building a Network ("Add a TAP's Radio Step")

Extracted from `components/tigo_monitor/tap_config.h` (`RadioBlock` and `ProvisionJob`).

A factory-reset or freshly updated TAP has channel `0` and an all-zero key. Until it is given a valid channel (`11..26`), a PAN ID, a mesh slot map, and a 16-byte network key, optimizers cannot join it.

### 4.1 The GatewayCfg Radio Block (`36-byte` and `46-byte` Forms)
Radio settings are read and written via tunnel sub-command **`0x0D`** (`SUB_GATEWAY_CFG`), answered by sub-command **`0x0E`** (`SUB_GATEWAY_CFG_REPLY`).

There are two block sizes in the field:
- **Standard 36-byte form** (e.g., `H1.0007`, `G8.65`)
- **Wide 46-byte form** (e.g., `G8.59`): inserts 10 extra bytes at offset `31..40` between the network key and optimization. Even on a 46-byte TAP, the *stored* read (`00 01`) replies with 36 bytes. **Rule**: Always read the active block (`00 00`) first, and write back the exact form (36 or 46 bytes) the TAP returned, preserving its 10 `extra` bytes if it returned 46 bytes.

#### Byte Layout of `RadioBlock`
| Byte Offset (36B) | Byte Offset (46B) | Size | Field | Value / Description |
| :--- | :--- | :--- | :--- | :--- |
| `0..1` | `0..1` | 2B BE | `channel` | IEEE 802.15.4 channel: **`11` to `26`** (`0x000B`..`0x001A`). Byte 0 is always `0x00`. The TAP refuses writes outside 11..26. |
| `2..3` | `2..3` | 2B BE | `pan` | 16-bit PAN ID (`0x0001`..`0xFFFE`, never `0x0000` or `0xFFFF`). |
| `4..13` | `4..13` | 10B | `map` | Sizing profile slot map (see table below). Bytes `5..7` of `map` (block bytes `9..11`) must be `00 00 00`. |
| `14` | `14` | 1B | `slt` | Signal Level Threshold for join admission: **`0x5A`** (90). |
| `15..30` | `15..30` | 16B | `key` | 16-byte network encryption key. Each byte is randomly chosen from `0x01..0xFE` (**no `0x00` and no `0xFF` bytes**). |
| — | `31..40` | 10B | `extra` | Present **only** in the 46-byte form. Copy verbatim from the TAP's `0D 00 00` read reply. |
| `31..34` | `41..44` | 4B BE | `optimization` | Fixed big-endian constant: **`00 00 02 00`** (`0x00000200`). |
| `35` | `45` | 1B | `slm` | Signal Level margin: **`0x3C`** (60). |

### 4.2 How Parameter Values Are Chosen (`tap_config.h`)
1. **Channel (`channel_for`)**:
   - Valid range: `11..26` (where frequency is $2405 + 5 \times (c - 11)\text{ MHz}$).
   - Preferred channels that sit cleanly between 2.4 GHz Wi-Fi channels 1, 6, and 11: **`15`, `20`, `25`, `26`** (default first pick for a single TAP is **`15`**, i.e. `0x000F`).
   - Multi-TAP rule: pick a channel at least 2 channels away from any other TAP on the bus (`|c - used| >= 2`).
2. **PAN ID (`pan_for`)**:
   - Derived from the low 16 bits of a 2-byte host seed (or the TAP's MAC low 16 bits `(mac[6] << 8) | mac[7]`), stepped by `+ 0x1000` if `0x0000`, `0xFFFF`, or already in use by another TAP.
3. **Slot Map (`map_for_profile`)**:
   - Chosen by capacity (default for up to 40 optimizers is profile `12`):
     | Profile Capacity | Map Name | 10-Byte Hex Value (`map[0..9]`) |
     | :--- | :--- | :--- |
     | Up to **40** optimizers | `12-2-2-1/28/1` | `0C 02 02 01 00 00 00 00 1C 01` |
     | Up to **75** optimizers | `14-2-2-1/28/1` | `0E 02 02 01 00 00 00 00 1C 01` |
     | Up to **120** optimizers | `20-2-2-1/28/1` | `14 02 02 01 00 00 00 00 1C 01` |
     | Up to **300** optimizers | `24-4-2-1/28/1` | `18 04 02 01 00 00 00 00 1C 01` |
   - If the TAP already has a valid known profile (`map[8] != 0` and `map[0]` in `{12, 14, 20, 24}`) that covers its current optimizer count, preserve `map[0..4]` and `map[8..9]`, and zero out `map[5..7]`.
4. **Network Key (`key_from`)**:
   - Draw 16 cryptographically random bytes, rejecting `0x00` and `0xFF` (`while b in (0x00, 0xFF): redraw()`).
   - **Multi-TAP Site Note**: On a multi-TAP site, **all TAPs must share the exact same 16-byte network key** while having distinct channels and PAN IDs.

### 4.3 Step-by-Step Provisioning State Machine (`ProvisionJob`)

Execute these steps sequentially against `DEST = 0x1201`:

1. **Step 1 — Verify TAP Identity (`identify`)**:
   - Send `CMD_GET_MAC_NOW` (`CMD = 0x003A`, empty payload, timeout `2000 ms`).
   - Verify `reply.cmd == 0x003B`, `len(reply.payload) >= 10`, and `reply.payload[0:8] == expected_tap_mac`. Abort if it does not match.
2. **Step 2 — Read Current Active Radio Settings (`read_before`)**:
   - Send Tunnel `SUB_GATEWAY_CFG` (`0x0D`) with args `00 00` (timeout `2000 ms`):
     - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 0D <seq> 00 00`
   - Parse Tunnel Reply (`CMD = 0x0B10`, `sub == 0x0E`):
     - `data = reply.payload[5:]`
     - `block_bytes = data[:-1]` (must be **36** or **46** bytes), `status = data[-1]`.
     - If `len(block_bytes) == 46`, set `wide = True` and copy `extra = block_bytes[31:41]` into the new block to be written.
3. **Step 3 — One Poll Turn Between Read and Write**:
   - Send one normal telemetry poll (`CMD = 0x0148`, `PAYLOAD = 00 01 00 00 04`) or pause ~25 ms before the write, matching CCA timing.
4. **Step 4 — Write New Radio Settings to RAM (`write`)**:
   - Encode the 36-byte (or 46-byte if `wide`) `RadioBlock`.
   - Prepend write selector `01 00` (`args = 01 00 + encoded_block`, 38 or 48 bytes total).
   - Send Tunnel `SUB_GATEWAY_CFG` (`0x0D`, timeout `3000 ms`):
     - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 0D <seq> 01 00 <36_or_46_byte_block>`
   - Verify Tunnel Reply (`CMD = 0x0B10`, `sub == 0x0E`):
     - `echo_block = data[:-1]`, `status = data[-1]`.
     - `status` **must be `0x00` or `0x01`** (`0x01` simply indicates the TAP's join window is currently open; any other value is a refusal).
     - `echo_block` **must match** `encoded_block` byte-for-byte.
   - *Effect*: The TAP updates RAM and retunes its radio immediately, but **nothing is saved to flash yet**.
5. **Step 5 — Read Stored Copy / Trigger H1 Save (`save`)**:
   - Send Tunnel `SUB_GATEWAY_CFG` (`0x0D`) with args `00 01` (timeout `2000 ms`):
     - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 0D <seq> 00 01`
   - Expect Tunnel Reply (`CMD = 0x0B10`, `sub == 0x0E`, 36-byte block + 1 status byte).
   - *Firmware difference*:
     - On **H1** firmware (`H1.xxxx`), this `0D 00 01` read is what marks every field to be saved to flash in the background, and its reply echoes the new RAM settings (`same_settings` must match).
     - On **G8** firmware (`G8.xx`), `0D 00 01` returns the old stored flash copy and saves nothing yet.
6. **Step 6 — Check Active RAM Copy (`check`)**:
   - Send Tunnel `SUB_GATEWAY_CFG` (`0x0D`) with args `00 00` (timeout `2000 ms`):
     - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 0D <seq> 00 00`
   - Verify the returned active block matches `encoded_block` byte-for-byte.
7. **Step 7 — Commit to Flash on G8 TAPs (`store` + `store_read`)**:
   - If the TAP is a **G8** (version string starts with `"G8."` or contains `" G8."`), or if explicitly requested on the bench:
     1. Send top-level `CMD_STORE` (`DEST = 0x1201`, `CMD = 0x0012`, `PAYLOAD = 37 24 92 66`, timeout `3000 ms`).
     2. Verify reply `CMD = 0x0013` (on G8 it typically carries 19 payload bytes).
     3. Send Tunnel `SUB_GATEWAY_CFG` (`0x0D`) with args `00 01` (`store_read`, timeout `2000 ms`) and verify that the stored copy now matches the written `channel`, `pan`, `map`, `slt`, `key`, `optimization`, and `slm` (`same_settings`).

---

## 5. Operation 2: Adding an Optimizer ("Add an Optimizer")

Extracted from `components/tigo_monitor/table_edit.h` (`mac_of_label`, `TableWalk`, `TableAdd`, `JoinWatch`) and `components/tigo_monitor/tigo_master.h` (`WindowKeeper`, `Queries`, `Housekeeping`).

### 5.1 Converting an Optimizer Barcode Label to an 8-Byte MAC
An optimizer can be specified either by its full 16-hex-character MAC (`04C05B...`) or by the short label printed on its sticker (e.g., **`4-9A57A2L`**).

#### Barcode Decoding Rules (`mac_of_label` in `table_edit.h`)
1. Strip spaces/tabs and convert to uppercase.
2. Verify there is a dash `'-'` at index `1` (`s[1] == '-'`) and total length is at least 3.
3. The last character `s[-1]` is a **CRC-4 check letter** from the alphabet:
   ```text
   LABEL_CHECK_DIGITS = "GHJKLMNPRSTVWXYZ"  (values 0..15)
   ```
4. The hex nibbles after the fixed Tigo vendor prefix `04 C0 5B` are formed by:
   - Taking the single hex character before the dash (`s[0]`),
   - Taking the hex characters after the dash up to the check letter (`s[2:-1]`),
   - Inserting `'0'` characters immediately after `s[0]` (where the dash was) so the nibble string is padded to **10 hex characters** (5 bytes).
5. Prepend `04 C0 5B` to those 5 bytes to form the 8-byte MAC.
   - *Example*: `"4-9A57A2L"` → first nibble `"4"`, tail `"9A57A2"` (6 chars) → insert three `'0'`s → `"40009A57A2"` → `04 C0 5B 40 00 9A 57 A2`.
6. Compute CRC-4 (polynomial `0x3`, initial value `0x2`) over all 8 bytes of the MAC and verify `LABEL_CHECK_DIGITS[crc] == s[-1]`:

```python
LABEL_CHECK_DIGITS = "GHJKLMNPRSTVWXYZ"
TIGO_PREFIX = bytes([0x04, 0xC0, 0x5B])

def label_check(mac: bytes) -> int:
    crc = 0x2
    for b in mac:
        r = (b ^ (crc << 4)) & 0xFF
        for _ in range(8):
            r = ((r << 1) ^ 0x30) & 0xFF if (r & 0x80) else (r << 1) & 0xFF
        crc = (r >> 4) & 0x0F
    return crc

def mac_of_label(typed: str) -> bytes:
    s = "".join(c.upper() for c in typed if c not in " \t")
    if len(s) < 3 or s[1] != "-":
        raise ValueError("Invalid label format (expected N-XXXXXC)")
    nibbles = s[0] + s[2:-1]
    if len(nibbles) > 10 or any(c not in "0123456789ABCDEF" for c in nibbles):
        raise ValueError("Invalid hex digits in label")
    nibbles = nibbles[0] + ("0" * (10 - len(nibbles))) + nibbles[1:]
    mac = TIGO_PREFIX + bytes.fromhex(nibbles)
    expected_check = LABEL_CHECK_DIGITS[label_check(mac)]
    if s[-1] != expected_check:
        raise ValueError(f"Label check letter mismatch: got {s[-1]}, expected {expected_check}")
    return mac
```

### 5.2 Walking the TAP's Optimizer Table (`TableWalk`, Tunnel Sub `0x26` / `0x27`)
The TAP stores up to 1024 records in flash. Each record maps an 8-byte optimizer MAC to a 16-bit `TUID`:
- **Bits `0..14` (`tuid & 0x7FFF`)**: Short node address (`id`), valid range **`0x0002`..`0x03FF`** (`2..1023`). (`0x0001` is reserved for the TAP itself).
- **Bit `15` (`tuid & 0x8000`)**: Provisional / "not joined yet" flag.
  - `1` (`0x8000 | id`): Listed in the table, waiting to join.
  - `0` (`id`): Optimizer has joined the TAP.

#### Pagination Algorithm
1. Start at `index = 0x0000`.
2. Send Tunnel `GetTable` (`sub = 0x26`, timeout `500 ms`):
   - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 26 <seq> <index_hi> <index_lo>`
3. Parse Tunnel Reply (`CMD = 0x0B10`, `sub == 0x27`):
   - `data = reply.payload[5:]` (must be at least 4 bytes):
     ```text
     +------------------+------------------+------------------------------+
     | start_id (2B BE) |  count (2B BE)   | count * [MAC (8B) + TUID(2B)]|
     +------------------+------------------+------------------------------+
     ```
   - If `count == 0`: you have reached the end of the table.
   - Otherwise, parse `count` records (10 bytes each: `mac = rec[0:8]`, `tuid = (rec[8] << 8) | rec[9]`, `id = tuid & 0x7FFF`).
   - Advance `index = (last_record.tuid & 0x7FFF) + 1` and repeat until `count == 0`.
   - *(Retry rule: if a page times out, retry the same `index` once before failing).*

### 5.3 Appending the Optimizer Record (`TableAdd`, Tunnel Sub `0x29` / `0x2A`)
Unlike the CCA (which erases the entire flash table with `0x2B BA BE` and rewrites all records), `TableAdd` **appends a single record** via `0x29` without erasing existing optimizers.

1. **Step 1 — Read Table Before (`read_before`)**:
   - Run `TableWalk` to fetch all existing records.
   - Check if `target_mac` is already in the table:
     - If already present and `(tuid & 0x8000) == 0`: it is already listed and joined; nothing to write.
     - If already present and `(tuid & 0x8000) != 0`: it is already listed at `id = tuid & 0x7FFF`; skip the write and proceed directly to **Section 5.4 (Opening the Discovery Window)**.
   - **Important**: Never rewrite an already-joined MAC with `0x29`, because any `0x29` write resets the slot's persistent flash spare word to `0xFFFF` (unjoined) on the next reboot.
2. **Step 2 — Choose Short Address (`id`)**:
   - If a specific `want_id` (`2..1023`) was requested, verify it is not used in the table (and not used on any other TAP in the system).
   - Otherwise, pick the **lowest free integer `id` starting from `2`** (`0x0002`..`0x03FF`) that does not appear in `[r.id() for r in table]`.
3. **Step 3 — Write Provisional Record (`write`, Tunnel Sub `0x29`)**:
   - Build `SetTable` arguments with `count = 1` and **bit 15 (`0x8000`) set** on the TUID:
     ```text
     ARGS = 01 + <8-byte MAC> + <(0x8000 | id) as 2B BE>   (11 bytes)
     ```
   - Send Tunnel `SetTable` (`sub = 0x29`, timeout `2000 ms`):
     - `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 29 <seq> 01 <8-byte MAC> <0x80 | (id >> 8)> <id & 0xFF>`
   - Parse Tunnel Reply (`CMD = 0x0B10`, `sub == 0x2A`):
     - `ack = reply.payload[5]` (1 byte):
       | `ack` | Meaning |
       | :--- | :--- |
       | `0x00` | Done (record committed to flash) |
       | `0x01` | Address outside `2..1023` |
       | `0x02` | Table is full |
       | `0x03` | TAP couldn't save it (flash commit failed) |
       | `0x04` | Address is already in use |
       | `0x05` | Optimizer MAC is already listed under another address |
     - If `ack > 0`, abort with the corresponding error. (If the reply timed out, do not resend `0x29`; let Step 4's read-back decide whether it took).
4. **Step 4 — Read Table Back (`read_after`)**:
   - Run `TableWalk` again.
   - Verify `target_mac` is now present at `id` and all prior records are intact.

### 5.4 Opening the Discovery Window & Waiting for Join (`WindowKeeper` & `JoinWatch`)
Writing the record with bit 15 set (`0x8000 | id`) only authorizes the optimizer in the TAP's table. **An unjoined optimizer will only join while the TAP's discovery window is open** (and while the optimizer has PV input voltage to power its radio). When the optimizer joins, the TAP automatically clears bit 15 in flash (`0x8000 | id` → `0x0000 | id`).

 While waiting for the optimizer to join (typically 20 seconds to 10 minutes; timeout `30 minutes`):

1. **Check Discovery Window Status every 20 seconds (`SUB_DISCOVERY_STATUS = 0x2E`)**:
   - Request: `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 2E <seq>`
   - Reply (`CMD = 0x0B10`, `sub == 0x2F`, 9+ data bytes after `seq`):
     ```text
     data[0]    : running (0x01 = open, 0x00 = closed)
     data[1..2] : remaining_s (uint16 BE, seconds left on window)
     data[3..4] : requested (uint16 BE, echoed from last renewal)
     data[5..6] : joined (uint16 BE, count of joined nodes in table)
     data[7..8] : node_count (uint16 BE, total nodes in TAP table)
     ```
2. **Open / Renew Discovery Window when `remaining_s == 0` (`SUB_DISCOVERY_START = 0x2D`)**:
   - Requires `node_count > 0` (taken from `data[7..8]` of the `0x2F` status reply, or total records in the table).
   - Args (9 bytes, opens window for `0x0384` = 900 seconds):
     ```text
     BA BE 02 03 84 <node_count_hi> <node_count_lo> 01 00
     ```
   - Request: `CMD = 0x0B0F`, `PAYLOAD = 00 00 00 2D <seq> BA BE 02 03 84 <count_hi> <count_lo> 01 00`
   - Reply (`CMD = 0x0B10`, `sub == 0x2F`): same 9-byte `DiscoveryWindow` layout showing `running == 1` and `remaining_s == 900`.
3. **Keep Polling the Bus (`CMD_POLL = 0x0148`) & Checking the Table (`0x26`)**:
   - Between window checks, poll the TAP (`CMD = 0x0148`, `PAYLOAD = 00 <01 or 00 on retry> <cursor_hi> <cursor_lo> 04`), advancing `cursor` by the number of packets in each `0x0149` reply (or adopting the TAP's full cursor when status bit 4 is clear).
   - Once every **60 seconds** (`JoinWatch::JOIN_CHECK_MS`), walk the table with `0x26`:
     - Inspect the record for `target_mac`.
     - As soon as `(record.tuid & 0x8000) == 0`, **the optimizer has joined!**
4. **Post-Join StartPWM Check (Optional / Recommended for Factory-Fresh Optimizers)**:
   - As noted in `main.cpp` (`new_optimizer_note`) and `tigo_master.h` (`Queries`): when a newly joined optimizer starts reporting in `0x0149` poll replies, querying it via Tunnel `0x06` (`^00Info\r`, payload `00 00 00 06 <seq> <id_hi> <id_lo> 5E 30 30 49 6E 66 6F 0D`) returns an `!Info ...` type `0x07` packet inside a subsequent `0x0149` poll reply.
   - If the 9th whitespace-separated token of `!Info` (index 8, `StartPWM`) is `"00"`, send `#00w255\r` (`PAYLOAD = 00 00 00 06 <seq> <id_hi> <id_lo> 23 30 30 77 32 35 35 0D`) so the optimizer starts producing immediately.

---

## 6. Reference Standalone Python CLI Implementation

Below is a complete, dependency-free Python 3 script (requiring only `pyserial`, or using standard POSIX `termios`/`tty` directly) that implements the exact protocol above for a CLI tool:

```python
#!/usr/bin/env python3
"""
Standalone CLI tool to configure a Tigo TAP network and add an optimizer over RS485 TTY.
Extracted from components/tigo_monitor/{tigo_protocol.h,tigo_master.h,tap_config.h,table_edit.h}.

Usage examples:
  # 1. Inspect TAP at 0x1201 (or discover at 0x1235 and move to 0x1201):
  python3 tigo_cli.py --port /dev/ttyUSB0 status

  # 2. Build a network on the TAP (channel 15, auto PAN, 40-optimizer profile, random key):
  python3 tigo_cli.py --port /dev/ttyUSB0 provision --channel 15 --profile 40

  # 3. Add an optimizer by its printed label (or 16-hex MAC) and wait for it to join:
  python3 tigo_cli.py --port /dev/ttyUSB0 add-optimizer --label 4-9A57A2L --wait
"""

import argparse
import os
import secrets
import select
import struct
import sys
import termios
import time

ESCAPED = [0x7E, 0x24, 0x23, 0x25, 0xA4, 0xA3, 0xA5]
SETUP_KEY = bytes([0x37, 0x24, 0x92, 0x66])
TIGO_PREFIX = bytes([0x04, 0xC0, 0x5B])
LABEL_CHECK_DIGITS = "GHJKLMNPRSTVWXYZ"

FACTORY_ID = 0x1235
DEFAULT_TAP_ID = 0x1201

CMD_VERSION = 0x000A
CMD_RESET = 0x0010
CMD_STORE = 0x0012
CMD_SET_485_ID = 0x0014
CMD_GET_MAC = 0x0038
CMD_GET_MAC_NOW = 0x003A
CMD_SET_ID_BY_MAC = 0x003C
CMD_POLL = 0x0148
CMD_SESSION = 0x0B00
CMD_TUNNEL = 0x0B0F
CMD_TUNNEL_REPLY = 0x0B10


def crc16_wire(data: bytes) -> bytes:
    crc = 0x8408
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def encode_frame(dest: int, cmd: int, payload: bytes = b"") -> bytes:
    body = struct.pack(">HH", dest, cmd) + payload
    body += crc16_wire(body)
    out = bytearray(b"\x00\xFF\xFF\x7E\x07")
    for b in body:
        if b in ESCAPED:
            out.extend([0x7E, ESCAPED.index(b)])
        else:
            out.append(b)
    out.extend(b"\x7E\x08")
    return bytes(out)


def decode_frame(raw_between_delimiters: bytes):
    out = bytearray()
    i = 0
    n = len(raw_between_delimiters)
    while i < n:
        b = raw_between_delimiters[i]
        if b == 0x7E and i + 1 < n and raw_between_delimiters[i + 1] < 7:
            out.append(ESCAPED[raw_between_delimiters[i + 1]])
            i += 2
        else:
            out.append(b)
            i += 1
    if len(out) < 6 or crc16_wire(out[:-2]) != bytes(out[-2:]):
        return None
    dest, cmd = struct.unpack(">HH", out[:4])
    return dest, cmd, bytes(out[4:-2])


def label_check(mac: bytes) -> int:
    crc = 0x2
    for b in mac:
        r = (b ^ (crc << 4)) & 0xFF
        for _ in range(8):
            r = ((r << 1) ^ 0x30) & 0xFF if (r & 0x80) else (r << 1) & 0xFF
        crc = (r >> 4) & 0x0F
    return crc


def label_of(mac: bytes) -> str:
    if len(mac) != 8 or mac[:3] != TIGO_PREFIX:
        return ""
    nibbles = mac[3:].hex().upper()
    rest = nibbles[1:].lstrip("0")
    return f"{nibbles[0]}-{rest}{LABEL_CHECK_DIGITS[label_check(mac)]}"


def mac_of_label(typed: str) -> bytes:
    s = "".join(c.upper() for c in typed if c not in " \t")
    if len(s) == 16 and all(c in "0123456789ABCDEF" for c in s):
        mac = bytes.fromhex(s)
        if not label_of(mac):
            raise ValueError("MAC does not start with Tigo prefix 04C05B")
        return mac
    if len(s) < 3 or s[1] != "-":
        raise ValueError("Invalid label shape (expected N-XXXXXC or 16-hex MAC)")
    nibbles = s[0] + s[2:-1]
    if len(nibbles) > 10 or any(c not in "0123456789ABCDEF" for c in nibbles):
        raise ValueError("Invalid hex characters in label")
    nibbles = nibbles[0] + ("0" * (10 - len(nibbles))) + nibbles[1:]
    mac = TIGO_PREFIX + bytes.fromhex(nibbles)
    if s[-1] != LABEL_CHECK_DIGITS[label_check(mac)]:
        raise ValueError("Label check letter mismatch (check for a typo)")
    return mac


def map_for_profile(capacity: int) -> bytes:
    first = {40: 12, 75: 14, 120: 20, 300: 24}.get(capacity)
    if not first:
        raise ValueError("Profile capacity must be 40, 75, 120, or 300")
    second = 4 if first == 24 else 2
    return bytes([first, second, 2, 1, 0, 0, 0, 0, 28, 1])


def random_network_key() -> bytes:
    out = bytearray()
    while len(out) < 16:
        b = secrets.randbits(8)
        if b not in (0x00, 0xFF):
            out.append(b)
    return bytes(out)


def pan_from_mac(tap_mac: bytes) -> int:
    pan = (tap_mac[-2] << 8) | tap_mac[-1]
    for _ in range(16):
        if pan not in (0x0000, 0xFFFF):
            return pan
        pan = (pan + 0x1000) & 0xFFFF
    return 0x1000


class TapBus:
    def __init__(self, device: str, baud: int = 38400):
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_CLOEXEC | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        # cfmakeraw equivalent
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                      termios.ISTRIP | termios.INLCR | termios.IGNCR |
                      termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                      termios.ISIG | termios.IEXTEN)
        attrs[4] = termios.B38400
        attrs[5] = termios.B38400
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        flags = os.get_blocking(self.fd)
        os.set_blocking(self.fd, True)
        self.seq = 0
        self.rx_buf = bytearray()

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        if self.seq in (0x00, 0xFF):
            self.seq = 0x01
        return self.seq

    def transact(self, dest: int, cmd: int, payload: bytes = b"", timeout_ms: int = 500):
        wire = encode_frame(dest, cmd, payload)
        os.write(self.fd, wire)
        termios.tcdrain(self.fd)
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        while True:
            rem = deadline - time.monotonic()
            if rem <= 0:
                return None
            r, _, _ = select.select([self.fd], [], [], rem)
            if not r:
                return None
            chunk = os.read(self.fd, 512)
            if not chunk:
                return None
            self.rx_buf.extend(chunk)
            while True:
                s = self.rx_buf.find(b"\x7E\x07")
                if s < 0:
                    break
                e = self.rx_buf.find(b"\x7E\x08", s + 2)
                if e < 0:
                    break
                raw_frame = bytes(self.rx_buf[s + 2:e])
                del self.rx_buf[:e + 2]
                decoded = decode_frame(raw_frame)
                if not decoded:
                    continue
                r_dest, r_cmd, r_payload = decoded
                if r_dest != (dest | 0x8000):
                    continue
                if r_cmd == 0x0006:
                    raise RuntimeError(f"TAP refused command 0x{cmd:04X} with NAK (0x0006)")
                if r_cmd == cmd + 1:
                    if cmd == CMD_TUNNEL:
                        if len(r_payload) < 5 or r_payload[4] != payload[4]:
                            continue
                    return r_payload

    def tunnel(self, dest: int, sub: int, args: bytes = b"", timeout_ms: int = 2000):
        seq = self.next_seq()
        req = bytes([0x00, 0x00, 0x00, sub, seq]) + args
        rep = self.transact(dest, CMD_TUNNEL, req, timeout_ms=timeout_ms)
        if rep is None or len(rep) < 5 or rep[3] != (sub + 1) or rep[4] != seq:
            return None
        return rep[5:]


def ensure_tap_addressed(bus: TapBus, target_id: int = DEFAULT_TAP_ID):
    """Finds TAP at target_id or factory 0x1235, assigns target_id, stores it, and reads version."""
    rep = bus.transact(target_id, CMD_GET_MAC_NOW, timeout_ms=300)
    if rep and len(rep) >= 10:
        mac = rep[:8]
    else:
        rep = bus.transact(FACTORY_ID, CMD_GET_MAC_NOW, timeout_ms=300)
        if not rep or len(rep) < 10:
            bus.transact(0x0000, CMD_SET_485_ID, SETUP_KEY + struct.pack(">H", FACTORY_ID), timeout_ms=500)
            rep = bus.transact(FACTORY_ID, CMD_GET_MAC, timeout_ms=800)
        if not rep or len(rep) < 10:
            raise RuntimeError("No TAP answered at configured ID or factory address 0x1235")
        mac = rep[:8]
        print(f"Found TAP {mac.hex().upper()} at factory 0x1235; assigning 0x{target_id:04X}...")
        bus.transact(FACTORY_ID, CMD_SET_ID_BY_MAC, SETUP_KEY + mac + struct.pack(">H", target_id), timeout_ms=500)
        verify = bus.transact(target_id, CMD_GET_MAC_NOW, timeout_ms=500)
        if not verify or verify[:8] != mac:
            raise RuntimeError("TAP did not answer at newly assigned ID")
        bus.transact(target_id, CMD_STORE, SETUP_KEY, timeout_ms=2000)
        print(f"Committed RS485 ID 0x{target_id:04X} to TAP flash with STORE.")

    v_rep = bus.transact(target_id, CMD_VERSION, timeout_ms=500)
    version = v_rep.decode("ascii", errors="replace").strip().replace("\r", " ") if v_rep else ""
    bus.transact(target_id, CMD_SESSION, b"\x01", timeout_ms=500)
    return mac, version


def decode_radio_block(data: bytes) -> dict:
    if len(data) not in (36, 46):
        raise ValueError(f"Unexpected radio block size: {len(data)}")
    wide = len(data) == 46
    o = 10 if wide else 0
    return {
        "channel": (data[0] << 8) | data[1],
        "pan": (data[2] << 8) | data[3],
        "map": data[4:14],
        "slt": data[14],
        "key": data[15:31],
        "wide": wide,
        "extra": data[31:41] if wide else b"",
        "optimization": struct.unpack(">I", data[31 + o:35 + o])[0],
        "slm": data[35 + o],
    }


def encode_radio_block(blk: dict) -> bytes:
    out = bytearray()
    out += struct.pack(">HH", blk["channel"], blk["pan"])
    out += blk["map"]
    out.append(blk["slt"])
    out += blk["key"]
    if blk.get("wide"):
        out += blk["extra"]
    out += struct.pack(">IB", blk["optimization"], blk["slm"])
    return bytes(out)


def provision_tap(bus: TapBus, tap_id: int, tap_mac: bytes, version: str,
                  channel: int, pan: int, profile: int, key_hex: str = None):
    # 1. Verify MAC identity
    id_rep = bus.transact(tap_id, CMD_GET_MAC_NOW, timeout_ms=2000)
    if not id_rep or id_rep[:8] != tap_mac:
        raise RuntimeError("TAP MAC verification failed")

    # 2. Read active radio settings (0D 00 00)
    cur_data = bus.tunnel(tap_id, 0x0D, b"\x00\x00", timeout_ms=2000)
    if not cur_data or len(cur_data) not in (37, 47):
        raise RuntimeError("TAP did not answer radio settings read (0D 00 00)")
    before = decode_radio_block(cur_data[:-1])

    # 3. Build new RadioBlock in the exact form (36B or 46B) the TAP returned
    new_blk = {
        "channel": channel,
        "pan": pan if pan else pan_from_mac(tap_mac),
        "map": map_for_profile(profile),
        "slt": 0x5A,
        "key": bytes.fromhex(key_hex) if key_hex else random_network_key(),
        "wide": before["wide"],
        "extra": before["extra"],
        "optimization": 0x00000200,
        "slm": 0x3C,
    }
    if not (11 <= new_blk["channel"] <= 26):
        raise ValueError("Channel must be 11..26")
    if len(new_blk["key"]) != 16:
        raise ValueError("Network key must be 16 bytes")

    encoded = encode_radio_block(new_blk)

    # 4. One poll turn before write
    bus.transact(tap_id, CMD_POLL, b"\x00\x01\x00\x00\x04", timeout_ms=100)

    # 5. Write active radio block (0D 01 00 + block)
    w_rep = bus.tunnel(tap_id, 0x0D, b"\x01\x00" + encoded, timeout_ms=3000)
    if not w_rep or len(w_rep) != len(encoded) + 1:
        raise RuntimeError("No valid echo from radio settings write")
    w_status = w_rep[-1]
    if w_status not in (0, 1) or w_rep[:-1] != encoded:
        raise RuntimeError(f"TAP refused or corrupted radio write (status={w_status})")

    # 6. Read stored copy (0D 00 01) — arms background flash save on H1
    s_rep = bus.tunnel(tap_id, 0x0D, b"\x00\x01", timeout_ms=2000)
    if not s_rep or len(s_rep) not in (37, 47):
        raise RuntimeError("TAP did not answer save read (0D 00 01)")

    # 7. Check active copy (0D 00 00)
    c_rep = bus.tunnel(tap_id, 0x0D, b"\x00\x00", timeout_ms=2000)
    if not c_rep or c_rep[:-1] != encoded:
        raise RuntimeError("Check read (0D 00 00) differed from written block")

    # 8. On G8 firmware, commit with top-level STORE (0x0012 + key) and re-read 0D 00 01
    is_g8 = version.startswith("G8.") or " G8." in version
    if is_g8:
        st_rep = bus.transact(tap_id, CMD_STORE, SETUP_KEY, timeout_ms=3000)
        if st_rep is None:
            raise RuntimeError("G8 TAP did not answer CMD_STORE (0x0012)")
        sr_rep = bus.tunnel(tap_id, 0x0D, b"\x00\x01", timeout_ms=2000)
        if not sr_rep:
            raise RuntimeError("G8 TAP did not answer stored read after CMD_STORE")
        stored_blk = decode_radio_block(sr_rep[:-1])
        for k in ("channel", "pan", "map", "slt", "key", "optimization", "slm"):
            if stored_blk[k] != new_blk[k]:
                raise RuntimeError(f"G8 stored copy mismatch on {k} after CMD_STORE")

    print(f"Provisioned TAP 0x{tap_id:04X}: channel={new_blk['channel']}, "
          f"PAN=0x{new_blk['pan']:04X}, map={new_blk['map'].hex()}, "
          f"saved_via={'CMD_STORE (G8)' if is_g8 else '0D 00 01 (H1)'}")


def walk_table(bus: TapBus, tap_id: int) -> list:
    records = []
    seen = set()
    index = 0
    for _ in range(110):
        rep = bus.tunnel(tap_id, 0x26, struct.pack(">H", index), timeout_ms=500)
        if rep is None:
            rep = bus.tunnel(tap_id, 0x26, struct.pack(">H", index), timeout_ms=500)
        if rep is None or len(rep) < 4:
            raise RuntimeError("TAP did not answer table read (0x26)")
        start_id, count = struct.unpack(">HH", rep[:4])
        if count == 0:
            break
        if len(rep) < 4 + 10 * count:
            raise RuntimeError("Truncated table page (0x27)")
        last_id = index
        for i in range(count):
            off = 4 + 10 * i
            mac = rep[off:off + 8]
            tuid = struct.unpack(">H", rep[off + 8:off + 10])[0]
            last_id = tuid & 0x7FFF
            if mac not in seen:
                seen.add(mac)
                records.append((mac, tuid))
        index = last_id + 1
    return records


def add_optimizer(bus: TapBus, tap_id: int, opt_mac: bytes, want_id: int = 0, wait_join: bool = False):
    table = walk_table(bus, tap_id)
    existing = next(((m, t) for m, t in table if m == opt_mac), None)
    if existing:
        _, tuid = existing
        node_id = tuid & 0x7FFF
        joined = (tuid & 0x8000) == 0
        print(f"Optimizer {opt_mac.hex().upper()} ({label_of(opt_mac)}) is already in table at "
              f"0x{node_id:04X} ({'joined' if joined else 'provisional/waiting to join'}).")
        if joined or not wait_join:
            return node_id
    else:
        used_ids = {t & 0x7FFF for _, t in table}
        if want_id:
            if not (2 <= want_id <= 1023) or want_id in used_ids:
                raise ValueError(f"Requested ID 0x{want_id:04X} is invalid or already in use")
            node_id = want_id
        else:
            node_id = next((i for i in range(2, 1024) if i not in used_ids), 0)
            if not node_id:
                raise RuntimeError("TAP table is full (no free address in 2..1023)")

        # Write 1 provisional record with bit 15 set (0x8000 | node_id)
        args = bytes([0x01]) + opt_mac + struct.pack(">H", 0x8000 | node_id)
        rep = bus.tunnel(tap_id, 0x29, args, timeout_ms=2000)
        ack = rep[0] if (rep and len(rep) >= 1) else -1
        if ack > 0:
            raise RuntimeError(f"TAP refused table write (0x29) with ack code {ack}")

        after = walk_table(bus, tap_id)
        added = next(((m, t) for m, t in after if m == opt_mac), None)
        if not added or (added[1] & 0x7FFF) != node_id:
            raise RuntimeError("Read-back of table after 0x29 did not list the new optimizer")
        table = after
        print(f"Listed optimizer {opt_mac.hex().upper()} ({label_of(opt_mac)}) "
              f"at 0x{node_id:04X} (provisional: 0x{0x8000 | node_id:04X}).")

    if not wait_join:
        return node_id

    print("Holding discovery window open and waiting for optimizer to join (Ctrl-C to stop)...")
    cursor = 0
    last_window_check = 0.0
    last_table_check = time.monotonic()
    node_count = len(table)

    while True:
        now = time.monotonic()
        if now - last_window_check >= 20.0:
            w_rep = bus.tunnel(tap_id, 0x2E, b"", timeout_ms=1000)
            if w_rep and len(w_rep) >= 9:
                running = w_rep[0] == 1
                rem_s, _, joined_cnt, tbl_cnt = struct.unpack(">HHHH", w_rep[1:9])
                node_count = tbl_cnt or node_count
                if rem_s == 0 and node_count > 0:
                    renew_args = bytes([0xBA, 0xBE, 0x02, 0x03, 0x84,
                                        node_count >> 8, node_count & 0xFF, 0x01, 0x00])
                    bus.tunnel(tap_id, 0x2D, renew_args, timeout_ms=1000)
                    print(f"Opened/renewed discovery window for 900s (table count={node_count}, joined={joined_cnt})")
            last_window_check = now

        if now - last_table_check >= 30.0:
            cur_table = walk_table(bus, tap_id)
            rec = next(((m, t) for m, t in cur_table if m == opt_mac), None)
            if rec and (rec[1] & 0x8000) == 0:
                print(f"SUCCESS: Optimizer {label_of(opt_mac)} joined at 0x{rec[1]:04X}!")
                return rec[1]
            last_table_check = now

        # Poll telemetry to keep bus active
        p_rep = bus.transact(tap_id, CMD_POLL, struct.pack(">BBHB", 0x00, 0x01, cursor, 0x04), timeout_ms=50)
        if p_rep and len(p_rep) >= 5:
            status = (p_rep[0] << 8) | p_rep[1]
            idx = 2
            high_at = None
            for bit, sz in enumerate((1, 1, 2, 2, 1)):
                if not (status & (1 << bit)):
                    if bit == 4:
                        high_at = idx
                    idx += sz
            if idx < len(p_rep):
                if high_at is not None:
                    cursor = (p_rep[high_at] << 8) | p_rep[idx]
                idx += 3  # bit 5 (1B) + bit 6 (2B)
                pkts = 0
                while idx + 7 <= len(p_rep):
                    idx += 7 + p_rep[idx + 6]
                    if idx <= len(p_rep):
                        pkts += 1
                cursor = (cursor + pkts) & 0xFFFF
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description="Standalone Tigo TAP Network & Optimizer Configurator")
    ap.add_argument("--port", required=True, help="Serial TTY port (e.g. /dev/ttyUSB0)")
    ap.add_argument("--tap-id", type=lambda x: int(x, 16), default=DEFAULT_TAP_ID, help="Target TAP RS485 ID in hex (default: 1201)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="Discover TAP, read version, radio config, and optimizer table")

    p_prov = sub.add_parser("provision", help="Configure TAP radio network (channel, PAN, map, key)")
    p_prov.add_argument("--channel", type=int, default=15, help="802.15.4 channel 11..26 (default: 15)")
    p_prov.add_argument("--pan", type=lambda x: int(x, 16), default=0, help="4-digit hex PAN ID (default: derived from MAC)")
    p_prov.add_argument("--profile", type=int, default=40, choices=[40, 75, 120, 300], help="Max optimizer capacity profile")
    p_prov.add_argument("--key", default=None, help="Optional 32-hex-digit network key (default: random)")

    p_add = sub.add_parser("add-optimizer", help="Add an optimizer to the TAP table")
    p_add.add_argument("--label", required=True, help="Optimizer barcode label (e.g. 4-9A57A2L) or 16-hex MAC")
    p_add.add_argument("--id", type=lambda x: int(x, 16), default=0, help="Optional short address 0002..03FF (default: lowest free)")
    p_add.add_argument("--wait", action="store_true", help="Hold discovery window open and wait for join")

    args = ap.parse_args()
    bus = TapBus(args.port)
    mac, version = ensure_tap_addressed(bus, args.tap_id)
    print(f"Connected to TAP 0x{args.tap_id:04X} | MAC: {mac.hex().upper()} | Version: {version}")

    if args.cmd == "status":
        r_data = bus.tunnel(args.tap_id, 0x0D, b"\x00\x00", timeout_ms=2000)
        if r_data and len(r_data) in (37, 47):
            blk = decode_radio_block(r_data[:-1])
            print(f"Radio Config: channel={blk['channel']}, PAN=0x{blk['pan']:04X}, "
                  f"map={blk['map'].hex()}, SLT=0x{blk['slt']:02X}, SLm=0x{blk['slm']:02X}, "
                  f"form={46 if blk['wide'] else 36}B, key_all_zero={all(b == 0 for b in blk['key'])}")
        table = walk_table(bus, args.tap_id)
        print(f"Optimizer Table ({len(table)} entries):")
        for m, tuid in table:
            print(f"  0x{tuid & 0x7FFF:04X} | MAC={m.hex().upper()} | Label={label_of(m):10s} | "
                  f"{'JOINED' if (tuid & 0x8000) == 0 else 'PROVISIONAL (0x8000)'}")
    elif args.cmd == "provision":
        provision_tap(bus, args.tap_id, mac, version, args.channel, args.pan, args.profile, args.key)
    elif args.cmd == "add-optimizer":
        opt_mac = mac_of_label(args.label)
        add_optimizer(bus, args.tap_id, opt_mac, args.id, args.wait)


if __name__ == "__main__":
    main()
```
