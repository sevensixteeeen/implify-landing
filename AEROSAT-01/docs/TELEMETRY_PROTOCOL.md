# AEROSAT-01 — Telemetry Protocol

| | |
|---|---|
| Document | TELEMETRY_PROTOCOL.md |
| Protocol version | 1.0 — DRAFT FOR REVIEW |
| Date | 2026-11-10 |
| Transport | WebSocket text frames, UTF-8 JSON, one JSON object per frame |
| Direction | Server → client only. Commands use the REST API (ARCHITECTURE.md §8.3). |

> All data carried by this protocol is **simulated**. Example values in this document are hand-written illustrations, not output from any run.

---

## 1. Design rules

1. **One object per frame**, no batching, no binary.
2. **SI units, named in the key.** `altitude_m`, `velocity_mps`, `acceleration_mps2`, `pressure_pa`, `temperature_k`, `roll_rate_radps`, `t_sim_s`. Where a unit is carried as a string (the `units` object of `hello`), it is written in ASCII exactly as: `s`, `m`, `m/s`, `m/s^2`, `Pa`, `K`, `rad/s`.
3. **Every message says it is simulated:** `data_origin` is always `"SIMULATED"`. The value `"MEASURED"` is reserved, is never emitted by this system, and must be rejected by this system's clients.
4. **Sim time is the timestamp.** Wall-clock time is informational only.
5. **No non-finite numbers.** `NaN` and `Infinity` are not valid JSON and must never be sent.
6. **Telemetry carries what the vehicle would know:** avionics estimates and raw sensor readings. Truth is never downlinked.

## 2. Message catalogue

| `type` | Origin | Has `seq` | When |
|---|---|---|---|
| `telemetry` | Vehicle (avionics) | yes | Every 50 ms of sim time (20 Hz) |
| `event` | Vehicle (avionics) | yes | When a flight event is detected |
| `hello` | Ground infrastructure (server) | no | First message after connect |
| `status` | Ground infrastructure | no | Mission status change, playback seek, client drop report |
| `heartbeat` | Ground infrastructure | no | Every 1 s of wall time |
| `error` | Ground infrastructure | no | Fault the client should display |

"Vehicle-originated" messages are the simulated downlink. They share one sequence counter and are the only messages recorded in `telemetry.jsonl` and replayed by playback.

## 3. `telemetry` message

### 3.1 Fields

| Field | JSON type | Unit | Required | Definition |
|---|---|---|---|---|
| `type` | string | – | yes | Constant `"telemetry"` |
| `schema_version` | string | – | yes | `"1.0"`. Major.minor. |
| `data_origin` | string | – | yes | Constant `"SIMULATED"` |
| `source` | string | – | yes | `"live"` or `"playback"`. Added by the server at send time. |
| `mission_id` | string | – | yes | `AS01-` + 8 uppercase hex characters, e.g. `AS01-7F3A9C2E`. Generated randomly per mission; not derived from the scenario seed. Playback reuses the recorded value. |
| `seq` | integer | – | yes | Packet sequence number, 0 … 4 294 967 295 (§5) |
| `t_sim_s` | number | s | yes | **Timestamp.** Sim time since scenario start = tick / 1000. For `telemetry`, always a multiple of 0.05. |
| `t_wall_utc` | string | – | yes on the wire | ISO 8601 UTC time the server sent the frame. Informational; not recorded. |
| `altitude_m` | number | m | yes | Estimated altitude above the pad (AGL) |
| `velocity_mps` | number | m/s | yes | Estimated vertical velocity, positive up |
| `acceleration_mps2` | number | m/s^2 | yes | Estimated vertical kinematic acceleration, positive up (bias-corrected specific force minus g₀). About 0 at rest on the pad. |
| `pressure_pa` | number | Pa | yes | Most recent barometer reading |
| `temperature_k` | number | K | yes | Most recent temperature-sensor reading |
| `flight_state` | string | – | yes | `PAD_IDLE`, `BOOST`, `COAST`, `DESCENT`, `LANDED` |
| `roll_rate_radps` | number | rad/s | yes | Most recent z-gyroscope reading. *Addition to the requested field list, needed to drive the 3D view (decision D-1).* |

All of `altitude_m`, `velocity_mps`, `acceleration_mps2` are **estimates**. They are 0.0 until pad calibration completes (§8). `pressure_pa`, `temperature_k`, `roll_rate_radps` are **measurements**. Neither is truth.

### 3.2 Example (illustrative values)

```json
{
  "type": "telemetry",
  "schema_version": "1.0",
  "data_origin": "SIMULATED",
  "source": "live",
  "mission_id": "AS01-7F3A9C2E",
  "seq": 413,
  "t_sim_s": 20.55,
  "t_wall_utc": "2026-11-10T14:02:31.482Z",
  "altitude_m": 2050.3,
  "velocity_mps": 36.2,
  "acceleration_mps2": -9.9,
  "pressure_pa": 79120.0,
  "temperature_k": 275.44,
  "flight_state": "COAST",
  "roll_rate_radps": 0.52,
  "data_quality": "NOMINAL",
  "invalid_sources": []
}
```

### 3.3 JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "aerosat-01/telemetry/1.0",
  "title": "AEROSAT-01 telemetry message (simulated data)",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "type", "schema_version", "data_origin", "source", "mission_id", "seq",
    "t_sim_s", "t_wall_utc", "altitude_m", "velocity_mps", "acceleration_mps2",
    "pressure_pa", "temperature_k", "flight_state", "roll_rate_radps",
    "data_quality", "invalid_sources"
  ],
  "properties": {
    "type":              { "const": "telemetry" },
    "schema_version":    { "type": "string", "pattern": "^1\\.[0-9]+$" },
    "data_origin":       { "const": "SIMULATED" },
    "source":            { "enum": ["live", "playback"] },
    "mission_id":        { "type": "string", "pattern": "^AS01-[0-9A-F]{8}$" },
    "seq":               { "type": "integer", "minimum": 0, "maximum": 4294967295 },
    "t_sim_s":           { "type": "number", "minimum": 0 },
    "t_wall_utc":        { "type": "string", "format": "date-time" },
    "altitude_m":        { "type": "number", "minimum": -1000, "maximum": 25000 },
    "velocity_mps":      { "type": "number", "minimum": -2000, "maximum": 2000 },
    "acceleration_mps2": { "type": "number", "minimum": -500, "maximum": 500 },
    "pressure_pa":       { "type": "number", "minimum": 1000, "maximum": 110000 },
    "temperature_k":     { "type": "number", "minimum": 200, "maximum": 330 },
    "flight_state":      { "enum": ["PAD_IDLE", "BOOST", "COAST", "DESCENT", "LANDED"] },
    "roll_rate_radps":   { "type": "number", "minimum": -35, "maximum": 35 },
    "data_quality":      { "enum": ["NOMINAL", "DEGRADED", "UNRELIABLE"] },
    "invalid_sources":   {
      "type": "array",
      "uniqueItems": true,
      "items": { "enum": ["accel.z", "baro.pressure", "temp.temperature", "gyro.z"] }
    }
  }
}
```

Numeric limits are sanity bounds (sensor ranges for measurements, generous envelopes for estimates), not expected flight values. The authoritative schema file will be exported from the backend message models to `schemas/` in Phase 2; this listing is the specification it must match.

### 3.4 Data quality

Every `telemetry` message carries two required fields that say how far its contents can be trusted.

| Field | JSON type | Required | Definition |
|---|---|---|---|
| `data_quality` | string | yes | `NOMINAL`, `DEGRADED` or `UNRELIABLE` (below) |
| `invalid_sources` | array of strings | yes | Sources whose most recent sample was rejected. Empty when there are none |

**Sources.** A source is one sensor channel the avionics uses: `accel.z`, `baro.pressure`, `temp.temperature`, `gyro.z`. Each appears at most once, in that order.

**Rejection.** The avionics rejects a sample that is saturated (clipped at the sensor's range limit, reason `SATURATED`) or not a finite number (reason `NON_FINITE`). A rejected sample is never used for calibration, estimation or event detection.

**`invalid_sources`.** A source is listed from the moment one of its samples is rejected until a new valid sample from it arrives. A source that has never delivered a valid sample is also listed.

**`data_quality`.** The first rule that applies, in this order:

1. `UNRELIABLE`: a required estimator input is too old (no valid `baro.pressure` sample for more than 1000 ms, or no valid `accel.z` sample for more than 100 ms), **or the barometer/accelerometer disagreement flag has been raised** (F1, ARCHITECTURE §6.4). The flag is latched: once raised, every later message is `UNRELIABLE` for the rest of the flight, even after both sensors deliver valid, agreeing samples again. The estimates cannot be trusted.
2. `DEGRADED`: `invalid_sources` is not empty, or a sample from any source was rejected in the previous 1000 ms. A valid replacement sample removes the source from `invalid_sources`, but `DEGRADED` persists until the 1000 ms have passed.
3. `NOMINAL`: otherwise.

Consequently, if `invalid_sources` is not empty, `data_quality` is never `NOMINAL`.

**What the reading fields carry when their source is invalid.** The field still holds a number, because v1 has no nulls (§8):

- After a saturated sample: the raw clipped reading, exactly as the sensor reported it.
- After a non-finite sample: the last finite reading from that source.
- If the source has never delivered a finite reading: 0.0.

In every one of these cases the source is listed in `invalid_sources`. A reading is valid only if its source is not listed.

**Field-to-source mapping.**

| Telemetry field | Source(s) |
|---|---|
| `altitude_m`, `velocity_mps` | `accel.z` and `baro.pressure` (estimates) |
| `acceleration_mps2` | `accel.z` (estimate) |
| `pressure_pa` | `baro.pressure` |
| `temperature_k` | `temp.temperature` |
| `roll_rate_radps` | `gyro.z` |

**Receiver rules.**

- A reading whose source is listed in `invalid_sources` must be identified as invalid wherever it is shown, and must never be presented or used as a valid measurement (dashboard test DSH-13).
- When `data_quality` is `UNRELIABLE`, `altitude_m`, `velocity_mps` and `acceleration_mps2` must be identified as unreliable.
- `temp.temperature` and `gyro.z` do not feed the estimator. An invalid sample from either makes the message `DEGRADED` but never `UNRELIABLE`.

## 4. Other messages

### 4.1 `event`

```json
{
  "type": "event",
  "schema_version": "1.0",
  "data_origin": "SIMULATED",
  "source": "live",
  "mission_id": "AS01-7F3A9C2E",
  "seq": 62,
  "t_sim_s": 3.07,
  "t_wall_utc": "2026-11-10T14:02:14.007Z",
  "event": "LIFTOFF",
  "flight_state": "BOOST",
  "detail": {},
  "source": "DETECTED",
  "quality": "NOMINAL"
}
```

| Field | Definition |
|---|---|
| `event` | `LIFTOFF`, `BURNOUT`, `APOGEE`, `DRAG_DEVICE_DEPLOYED`, `TOUCHDOWN` |
| `flight_state` | State after the event |
| `t_sim_s` | Sim time of **detection** by the avionics (multiple of 0.005), not the true event time |
| `detail` | Object with event-specific estimates, e.g. `{"altitude_m": 2125.4}` for `APOGEE`. May be empty. |
| `source` | How the avionics arrived at the event: `DETECTED` (the event's primary criterion), `FALLBACK_CRITERION` (its fallback criterion, from measurements), `BACKUP_TIMER` (inferred because a backup deployment timer expired: the timer anchored at detected `BURNOUT`, or the timer anchored at detected `LIFTOFF`; the `reason` says which and, if the disagreement flag restricted deployment to the timers, when and why the flag was raised), `BAROMETRIC_CONFIRMATION` (drag-device deployment confirmed by barometric descent after an unreliable apogee) |
| `quality` | `NOMINAL`, `DEGRADED` or `UNRELIABLE`. The reliability of the evidence used for this particular event, not the quality of the telemetry stream at that moment. It is fixed when the event is recorded and never rewritten |
| `reason` | Optional string. Present for fallback, inferred and confirmed events; says what triggered them |

An event's `quality` follows its own evidence. `LIFTOFF` and `BURNOUT` use valid `accel.z` samples only. A detected `APOGEE` and a primary `TOUCHDOWN` use the estimates, so they take the estimate quality (`accel.z` and `baro.pressure`). `DRAG_DEVICE_DEPLOYED` takes the quality of the path that deployed: the triggering apogee's quality for a normal deployment, `DEGRADED` for `BAROMETRIC_CONFIRMATION`, `UNRELIABLE` for `BACKUP_TIMER`. An invalid `gyro.z` or `temp.temperature` sample does not lower any event's quality.

### 4.2 `hello`

Sent once, first. Lets a client configure itself without hard-coding.

```json
{
  "type": "hello",
  "schema_version": "1.0",
  "data_origin": "SIMULATED",
  "source": "live",
  "mission_id": "AS01-7F3A9C2E",
  "scenario_id": "REF-01",
  "scenario_fictional": true,
  "mission_status": "RUNNING",
  "telemetry_rate_hz": 20,
  "next_seq": 414,
  "units": {
    "t_sim_s": "s", "altitude_m": "m", "velocity_mps": "m/s",
    "acceleration_mps2": "m/s^2", "pressure_pa": "Pa",
    "temperature_k": "K", "roll_rate_radps": "rad/s"
  },
  "notes": [
    "All data is simulated.",
    "altitude/velocity/acceleration are avionics estimates, not truth.",
    "Roll is a scripted kinematic profile, not simulated dynamics."
  ]
}
```

`next_seq` tells a late-joining client where to start counting, so joining mid-flight is not reported as packet loss.

### 4.3 `status`

```json
{
  "type": "status",
  "schema_version": "1.0",
  "data_origin": "SIMULATED",
  "mission_id": "AS01-7F3A9C2E",
  "mission_status": "PAUSED",
  "t_sim_s": 41.3,
  "reason": "operator",
  "dropped_for_client": 0,
  "next_seq": 890
}
```

- `mission_status`: `CREATED`, `RUNNING`, `PAUSED`, `COMPLETED`, `ABORTED`, `FAILED`.
- `reason`: `operator`, `completed`, `fault`, `playback_seek`, `client_backpressure`.
- After a playback seek, or after the server dropped queued messages for this client, `next_seq` tells the client where to resume counting. `dropped_for_client` is the cumulative number of messages the server discarded for this client.

### 4.4 `heartbeat`

```json
{ "type": "heartbeat", "schema_version": "1.0", "data_origin": "SIMULATED",
  "mission_id": "AS01-7F3A9C2E", "mission_status": "RUNNING",
  "t_wall_utc": "2026-11-10T14:02:32.000Z" }
```

Proves the connection is alive when the mission is paused or finished.

### 4.5 `error`

```json
{ "type": "error", "schema_version": "1.0", "data_origin": "SIMULATED",
  "mission_id": "AS01-7F3A9C2E", "code": "MODEL_RANGE",
  "message": "Altitude left the 0-20 km model range.", "t_sim_s": 88.412 }
```

Codes: `MODEL_RANGE`, `NON_FINITE_VALUE`, `RECORDER_FAILURE`, `UNKNOWN_MISSION`, `INTERNAL`.

### 4.6 Field tables

These tables state the fields of each message exactly as shown in the examples above. Every listed field is required unless marked optional. `source` and `t_wall_utc` appear only on the message types that list them. Common fields have the definitions given in §3.1. Each message type has its own schema file in `schemas/`.

**`event`**

| Field | JSON type | Definition |
|---|---|---|
| `type` | string | Constant `"event"` |
| `schema_version`, `data_origin`, `source`, `mission_id`, `seq`, `t_sim_s`, `t_wall_utc` | as §3.1 | As §3.1; `t_sim_s` is the detection time |
| `event` | string | `LIFTOFF`, `BURNOUT`, `APOGEE`, `DRAG_DEVICE_DEPLOYED`, `TOUCHDOWN` |
| `flight_state` | string | State after the event |
| `detail` | object | Event-specific estimates; may be empty |
| `source` | string | `DETECTED`, `FALLBACK_CRITERION`, `BACKUP_TIMER`, `BAROMETRIC_CONFIRMATION` |
| `quality` | string | `NOMINAL`, `DEGRADED`, `UNRELIABLE` |
| `reason` | string | Optional. What triggered a fallback, inferred or confirmed event |

**`hello`**

| Field | JSON type | Definition |
|---|---|---|
| `type` | string | Constant `"hello"` |
| `schema_version`, `data_origin`, `source`, `mission_id` | as §3.1 | As §3.1 |
| `scenario_id` | string | Scenario identifier, e.g. `REF-01` |
| `scenario_fictional` | boolean | The scenario's `fictional` flag |
| `mission_status` | string | See §4.3 |
| `telemetry_rate_hz` | integer | Telemetry frame rate |
| `next_seq` | integer | Sequence number the client should expect next |
| `units` | object | Field name → unit string |
| `notes` | array of strings | Human-readable statements about the data |

**`status`**

| Field | JSON type | Definition |
|---|---|---|
| `type` | string | Constant `"status"` |
| `schema_version`, `data_origin`, `mission_id` | as §3.1 | As §3.1 |
| `mission_status` | string | `CREATED`, `RUNNING`, `PAUSED`, `COMPLETED`, `ABORTED`, `FAILED` |
| `t_sim_s` | number | Sim time of the status change |
| `reason` | string | `operator`, `completed`, `fault`, `playback_seek`, `client_backpressure` |
| `dropped_for_client` | integer | Cumulative messages the server discarded for this client |
| `next_seq` | integer | Optional. Present after a playback seek or a client drop; resets the receiver's expected sequence number |

**`heartbeat`**

| Field | JSON type | Definition |
|---|---|---|
| `type` | string | Constant `"heartbeat"` |
| `schema_version`, `data_origin`, `mission_id` | as §3.1 | As §3.1 |
| `mission_status` | string | See §4.3 |
| `t_wall_utc` | string | As §3.1 |

**`error`**

| Field | JSON type | Definition |
|---|---|---|
| `type` | string | Constant `"error"` |
| `schema_version`, `data_origin`, `mission_id` | as §3.1 | As §3.1 |
| `code` | string | `MODEL_RANGE`, `NON_FINITE_VALUE`, `RECORDER_FAILURE`, `UNKNOWN_MISSION`, `INTERNAL` |
| `message` | string | Text for display |
| `t_sim_s` | number | Sim time of the fault |

## 5. Sequence-number rules

1. One counter per mission, shared by `telemetry` and `event`.
2. Starts at 0, increments by exactly 1 per vehicle-originated message, assigned **before** the link model.
3. Maximum 2³² − 1. At 20 Hz this is far beyond any mission; a mission that reached it would end with a fault rather than wrap.
4. Playback preserves recorded `seq` values.

## 6. Time rules

1. `t_sim_s` = tick / 1000, serialized with the shortest representation that reads back to the same double.
2. `t_sim_s` is non-decreasing with `seq`. An `event` may share its `t_sim_s` with a `telemetry` message.
3. `telemetry` messages are spaced exactly 0.05 s apart in sim time.
4. `t_wall_utc` must not be used for plotting, ordering or analysis.

## 7. Receiver integrity checks

A conforming client performs these on every message, in order, and counts failures by category:

| # | Check | On failure |
|---|---|---|
| 1 | Parses as JSON | Discard, count `malformed` |
| 2 | `schema_version` major is 1 | Discard, show version error |
| 3 | `data_origin` is `"SIMULATED"` | Discard, count `rejected_origin` |
| 4 | Passes validation for its `type` | Discard, count `invalid` |
| 5 | `mission_id` matches the subscribed mission | Discard, count `wrong_mission` |
| 6 | (`telemetry`, `event`) `seq` equals expected | See below |

A message whose `type` is not known to the client, within major version 1, is ignored after checks 1–3 (§10). It is not validated and is not counted as `invalid`.

Sequence handling, with `expected` initialised from `hello.next_seq`:

- `seq == expected` → accept; `expected += 1`.
- `seq > expected` → accept; `lost += seq − expected`; `expected = seq + 1`.
- `seq < expected` → discard; `duplicate_or_late += 1`.
- A `status` message with a `next_seq` resets `expected` without counting loss.

## 8. Encoding rules

- UTF-8, no byte-order mark. Field order is not significant.
- Numbers are JSON numbers, never strings. Integers (`seq`) have no fractional part.
- The producer refuses to serialize non-finite numbers (`NON_FINITE_VALUE` fault).
- No `null` values in v1: every required field always has a value. A reading that is not valid is still sent as a number and is marked through `invalid_sources` (§3.4). Sensors sample at tick 0, so readings exist from the first frame. Until pad calibration completes (ARCHITECTURE §6.1), the estimates `altitude_m`, `velocity_mps` and `acceleration_mps2` are sent as 0.0.

## 9. Recorded form

Lines in `telemetry.jsonl` are the wire messages **without** `source` and `t_wall_utc`, which the server adds at send time. This keeps recordings free of wall-clock data.

## 10. Versioning

- **Minor** (1.0 → 1.1): only adds optional fields or enum values for new message types. Consumers must ignore unknown fields and unknown message types within the same major version.
- **Major** (1.x → 2.0): anything else. Consumers must reject an unknown major version.
- The producer-side schema is strict (`additionalProperties: false`) so accidental extra fields are caught in tests.

## 11. Rate and size budget

A `telemetry` message is roughly 400 bytes. At 20 Hz that is about 8 kB/s per client, negligible on localhost. No compression or binary encoding is warranted.

## 12. Optional link-impairment model (decision D-4)

Applied by the server between the sequencer and the WebSocket hub. Seeded from the scenario, so impairments are reproducible.

| Parameter | Meaning | `ideal` profile |
|---|---|---|
| `drop_probability` | Chance a message is not delivered | 0 |
| `duplicate_probability` | Chance a message is delivered twice | 0 |
| `delay_ms_max` | Uniform extra delivery delay; may reorder | 0 |

Recording always captures messages before impairment. Corruption and checksums are deferred: WebSocket/TCP already guarantees byte integrity, so a CRC would only be meaningful together with a corruption model.
