# hp_ukf – ESPHome external component

External ESPHome component **hp_ukf** (HP-UKF): a mini split heat pump model using a time-discrete **Unscented Kalman Filter (UKF)**. Inputs are inlet/outlet air temperature and relative humidity; the filter tracks these and optionally the derivatives of temperature and humidity (dT_in, dT_out, dRH_in, dRH_out) to handle fast dynamics (e.g. ~1°C/s).

## Layout

```
components/
  hp_ukf/
    __init__.py      # Config schema and codegen
    hp_ukf.h         # C++ component header
    hp_ukf.cpp       # C++ component implementation
    hp_ukf_ukf.h     # UKF filter header
    hp_ukf_ukf.cpp   # UKF filter implementation
    example_hp_ukf.yaml
    README.md
```

## Usage

1. In your device YAML (same level or above the `components` folder), add:

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [ hp_ukf ]

# Define or reference your four sensors (inlet/outlet temperature and humidity)
sensor:
  - platform: dht
    # ... id: inlet_dht, temperature: inlet_temperature, humidity: inlet_humidity
  - platform: dht
    # ... id: outlet_dht, temperature: outlet_temperature, humidity: outlet_humidity

hp_ukf:
  id: hp_ukf
  update_interval: 1s
  inlet_temperature: inlet_temperature
  inlet_humidity: inlet_humidity
  outlet_temperature: outlet_temperature
  outlet_humidity: outlet_humidity
  track_temperature_derivatives: true
```

2. The component runs at `update_interval` (e.g. 1s): each tick it computes elapsed time since the last run, runs a UKF predict step with that `dt`, then updates with current sensor values and publishes filtered state to its output sensors.

3. Output sensors (created by the component): Filtered Inlet/Outlet Temperature and Humidity, and (if enabled) Filtered Inlet/Outlet Temperature Derivative (°C/s) and Filtered Inlet/Outlet Humidity Derivative (%/s).

## Configuration

| Option                         | Type    | Default | Description |
|--------------------------------|---------|---------|-------------|
| `id`                           | string  | (auto)  | Instance ID for automations |
| `update_interval`              | time    | `1s`    | Interval for predict/update and sensor reads |
| `inlet_temperature`            | sensor  | (none)  | Sensor ID for inlet air temperature (°C) |
| `inlet_humidity`               | sensor  | (none)  | Sensor ID for inlet air relative humidity (%) |
| `outlet_temperature`           | sensor  | (none)  | Sensor ID for outlet air temperature (°C) |
| `outlet_humidity`              | sensor  | (none)  | Sensor ID for outlet air relative humidity (%) |
| `air_flow`                     | sensor  | (none)  | Optional. Sensor ID for air flow in **L/s** (liters per second). When set, UKF state includes air flow and delivered power; optional `filtered_air_flow`, `delivered_power`, and `delivered_power_lag` sensors can be exposed. |
| `delivered_power_lag_tau_s`    | float   | `30`    | Optional. First-order lag time constant in **seconds** for the delivered-power UKF state (compressor start to indoor heating/cooling). Set to `0` for no lag (instantaneous). Only applies when `air_flow` is set. |
| `virtual_coil`                 | boolean | `false` | Optional. When `true` and `air_flow` is set, adds a **Virtual Coil Temperature (Tvcoil)** state to the UKF. Tvcoil models the indoor coil as a thermal reservoir (first-order lag toward a steady-state G(f_comp, T_outside)). When derivatives are *not* tracked, predicted outlet air temperature is coupled toward Tvcoil; when derivatives *are* tracked, outlet is evolved only by dT_out so the outlet temperature derivative reflects the actual outlet sensor. |
| `coil_tau_s`                   | float   | `60`    | Optional. Time constant in **seconds** for the virtual coil thermal lag (Tvcoil toward G). Only applies when `virtual_coil` is true. Represents the "heaviness" of the heat exchanger; EM auto-tune can adapt Tvcoil’s process noise. |
| `outlet_air_tau_s`             | float   | `20`    | Optional. Time constant in **seconds** for outlet air temperature first-order lag toward Tvcoil. Only applies when `virtual_coil` is true and derivatives are *not* tracked (n=7). When derivatives are tracked (n=11), outlet is not lagged toward Tvcoil. |
| `climate`                      | climate | (none)  | Optional. Climate entity ID (e.g. heat pump). The component reads **climate action** (OFF, HEATING, COOLING, IDLE, DRYING, FAN) each update and passes it as a control input to the UKF so the predict step can adapt (e.g. force delivered_power to 0 when OFF/IDLE). |
| `compressor_frequency`         | sensor  | (none)  | Optional. Sensor ID for compressor frequency in **Hz**. When set together with `climate`, the UKF uses it as a control input (e.g. when compressor is 0 Hz and action is OFF/IDLE, delivered_power is forced to 0). With [MitsubishiCN105ESPHome](https://github.com/echavet/MitsubishiCN105ESPHome), give the climate’s `compressor_frequency_sensor` an `id` and reference it here. |
| `power_sensor`                  | sensor  | (none)  | Optional. Sensor ID for **input power in W** (e.g. from PZEM or CN105 `input_power_sensor`). Power is correlated to compressor speed; when power is below ~10 W the UKF treats it as no delivered power. Used as a control input like climate action and compressor frequency. |
| `outside_temperature`           | sensor  | (none)  | Optional. Sensor ID for ambient/outside temperature (°C). Passed as UKF control input for process model use. |
| `outside_coil_temperature_before` | sensor | (none)  | Optional. Sensor ID for outdoor unit pipe temperature before coil/condenser/evaporator (°C). Passed as UKF control input. |
| `outside_coil_temperature_after`  | sensor | (none)  | Optional. Sensor ID for outdoor unit pipe temperature after coil (°C). Passed as UKF control input. |
| `inside_room_temperature`      | sensor  | (none)  | Optional. Sensor ID for inside room temperature (°C). Passed as UKF control input. |
| `inside_room_humidity`         | sensor  | (none)  | Optional. Sensor ID for inside room relative humidity (%). Passed as UKF control input. |
| `track_temperature_derivatives`| boolean | `true`  | If true, state is 8D or 10D (with air flow: T_in, RH_in, T_out, RH_out, air_flow, delivered_power, dT_in, dT_out, dRH_in, dRH_out); if false, 4D or 6D (no derivatives). When both `virtual_coil` and this are true, outlet temperature is evolved only by dT_out (not lagged toward Tvcoil) so the outlet temperature derivative reflects the actual outlet sensor. |
| `atmospheric_pressure`        | float   | `1013.25` | Atmospheric pressure in hPa. Used for psychrometric calculations (absolute humidity, enthalpy) and delivered power (density). Range 10–1200 hPa. |
| `em_autotune`                 | boolean | `false` | Enable EM (Expectation-Maximization) auto-tune for process (Q) and measurement (R) noise with forgetting factors. |
| `em_lambda_q`                | float   | `0.995` | Forgetting factor for Q (process variance). Range (0, 1]; higher = slower adaptation. |
| `em_lambda_r_inlet`          | float   | `0.998` | Forgetting factor for R of inlet T and RH. Inlet changes little; use higher value. |
| `em_lambda_r_outlet`         | float   | `0.98`  | Forgetting factor for R of outlet T and RH. Outlet can change ~1°C/s, 1%/s; use lower value for faster adaptation. |
| `em_inflation`               | float   | `0.5`   | Inflation factor applied to EM estimates before smoothing (non-cumulative). Default 0.5 yields R and Q 50% larger. Range [0, 2]. |
| `em_warmup_steps`            | int     | `20`    | Number of updates over which the EM weight ramps from 0 to full (1−λ). Reduces initial R/Q spike. Set to 0 to disable warmup. Range 0–1000. |

Optional EM sensors (only created if configured): `em_q_t_in`, `em_q_rh_in`, `em_q_t_out`, `em_q_rh_out`, `em_q_dt_in`, `em_q_dt_out`, `em_q_drh_in`, `em_q_drh_out` (process noise diagonal, variance units); `em_r_t_in`, `em_r_rh_in`, `em_r_t_out`, `em_r_rh_out` (measurement noise diagonal); `em_lambda_q_sensor`, `em_lambda_r_inlet_sensor`, `em_lambda_r_outlet_sensor` (current lambda values for debugging).

**Optional air flow and delivered power** (only when `air_flow` is set): `filtered_air_flow` (L/s) publishes the UKF-filtered air flow; `delivered_power` (W) publishes the UKF state (delivered power), which by default lags toward the instantaneous P = m_dot × (h_out − h_in) with time constant `delivered_power_lag_tau_s` (set to `0` for no lag). Optional `delivered_power_lag` (W) also publishes the same lagged UKF state when configured with a name. When `virtual_coil: true`, optional `filtered_virtual_coil_temperature` (°C) publishes the UKF Tvcoil state for logging/debugging. All these sensors require a name. Example:

```yaml
hp_ukf:
  # ...
  air_flow: air_flow
  delivered_power_lag_tau_s: 30  # 0 = no lag
  virtual_coil: true
  coil_tau_s: 60
  outlet_air_tau_s: 20
  filtered_air_flow:
    name: "${name} Filtered Air Flow"
  delivered_power:
    name: "${name} Delivered Power"
  delivered_power_lag:
    name: "${name} Delivered Power Lag"
  filtered_virtual_coil_temperature:
    name: "${name} Virtual Coil Temperature"
```

**Optional climate, compressor frequency, and power sensor** (control inputs for the UKF): When `climate` is set, the component reads the climate’s current **action** (OFF, HEATING, COOLING, IDLE, DRYING, FAN) each update and passes it to the filter. When the state machine is configured (climate plus power/inlet/outlet), the **inferred state** (e.g. `heating_active`, `defrosting`) is also passed to the UKF as a control input; defrost states force **delivered_power** to 0 in the predict step. When action is OFF, IDLE, FAN, or DRY—or when `compressor_frequency` is 0 or unknown—or when `power_sensor` (in W) is below ~10 W—the UKF also forces **delivered_power** to 0. Power is correlated to compressor speed. With [MitsubishiCN105ESPHome](https://github.com/echavet/MitsubishiCN105ESPHome), give the climate’s `compressor_frequency_sensor` an `id` and reference it in `compressor_frequency`; you can also use `input_power_sensor` as `power_sensor`. Example:

```yaml
climate:
  - platform: cn105
    id: hp
    compressor_frequency_sensor:
      id: compressor_freq
      name: "Compressor frequency"
      # ...
    input_power_sensor:
      id: hp_power
      name: "Input Power"
      # ...
hp_ukf:
  # ...
  climate: hp
  compressor_frequency: compressor_freq
  power_sensor: hp_power
```

**Optional environment/room control inputs**: You can pass outside temperature, outdoor coil pipe temperatures (before/after coil), and inside room temperature and humidity as UKF control inputs. When configured, the component reads these sensors each update and passes the values to the filter; they are stored for process model use (the current process model does not use them yet). Unavailable or unconfigured sensors are passed as NAN. Example:

```yaml
hp_ukf:
  # ...
  outside_temperature: outdoor_temp
  outside_coil_temperature_before: pipe_before_coil
  outside_coil_temperature_after: pipe_after_coil
  inside_room_temperature: room_temp
  inside_room_humidity: room_humidity
```

**State machine** (optional): When `climate`, `power_sensor`, `compressor_frequency`, `inlet_temperature`, and `outlet_temperature` are set, you can optionally add a **state** text_sensor and a **defrosting** binary_sensor. The component then infers heat pump operational state (heating idle/ramp-up/active, defrosting, cooling, etc.) from climate action, input power (W), compressor frequency, and inlet/outlet temperatures. The inferred state is passed to the UKF as a control input and used to force no delivered power during defrost (and optionally for future ramp-up/idle tuning). A power histogram (0–`power_max_w` W, `num_bins` bins, decay half-life `histogram_half_life_sec`) provides percentile thresholds: ramp-up → active uses the **50th percentile** (median); defrost detection uses **25th/75th** percentiles. When both UKF filtering and state/defrosting are configured, one component provides both. Options:

| Option                    | Type   | Default | Description |
|---------------------------|--------|---------|-------------|
| `state`                   | block  | (none)  | Optional. Text sensor for state string (e.g. `heating_active`, `defrosting`). |
| `defrosting`              | block  | (none)  | Optional. Binary sensor for defrost on/off. Use e.g. in air-volume templates so flow is low during defrost. |
| `histogram_half_life_sec` | int    | `600`   | Histogram decay half-life (seconds). |
| `power_max_w`             | int    | `3500`  | Power range upper bound (W); bins span 0 to this. |
| `num_bins`                | int    | `35`    | Number of histogram bins (5–100). |
| `delta_t_margin`           | float  | `1.0`   | Min \|outlet − inlet\| (°C) for “active” heating/cooling. |
| `compressor_low_hz`       | float  | `5.0`   | Compressor below this (Hz) is treated as off. |

States: `off`, `fan_only`, `heating_idle`, `heating_ramp_up`, `heating_active`, `defrosting`, `defrost_end`, `defrost_ramp_up`, `cooling_idle`, `cooling_ramp_up`, `cooling_active`, `stabilizing`, `unknown`. Ramp-up states (`heating_ramp_up`, `cooling_ramp_up`, `defrost_ramp_up`) are capped at **5 minutes**; after that the state is forced to the corresponding active state. Defrost detection: power was above high (75th percentile), then drops below low (25th) with compressor down; within 2 min power rises with compressor up and outlet &lt; inlet → `defrosting`. Defrost end: while defrosting, power drops from above high to below low within 10 min → `defrost_end`, then `defrost_ramp_up`, then heating states when outlet &gt; inlet.

```yaml
hp_ukf:
  # ... climate, power_sensor, compressor_frequency, inlet_temperature, outlet_temperature ...
  state:
    name: "${name} HP State"
  defrosting:
    id: hp_defrosting
    name: "${name} Defrosting"
```

**Optional psychrometric sensors** (only created if you add the block with a name): computed from filtered inlet/outlet temperature and relative humidity (UKF state); not part of the UKF state. Use `atmospheric_pressure` for absolute humidity and enthalpy. Options: `inlet_absolute_humidity` (g/m³), `inlet_dew_point` (°C), `inlet_enthalpy` (kJ/kg), `inlet_humidity_ratio` (g/kg); `outlet_absolute_humidity`, `outlet_dew_point`, `outlet_enthalpy`, `outlet_humidity_ratio`. Example:

```yaml
hp_ukf:
  # ...
  atmospheric_pressure: 1013.25
  inlet_dew_point:
    name: "${name} Inlet Dew Point"
  outlet_enthalpy:
    name: "${name} Outlet Enthalpy"
```

All four input sensors are optional; missing or unavailable readings are handled via a measurement mask (predict-only or update with available measurements).

## Time-discrete behaviour and missing samples

- **Variable frequency**: The predict step uses the actual elapsed time `dt` (seconds) since the last update, so varying sample rate is handled correctly.
- **Missing samples**: If a sensor has no valid state (e.g. NAN or not yet updated), that measurement is skipped in the update step; the filter still runs with the other measurements.

## Tuning (internal defaults)

Process and measurement noise are set inside the UKF with defaults suitable for typical mini-split sensors:

- **Process noise (Q)**: Small for T/RH; larger for derivatives (random-walk: dT_in, dT_out, dRH_in, dRH_out). Diagonal Q and R can be exposed as optional sensors when using EM auto-tune.
- **Measurement noise (R)**: Based on typical accuracy (e.g. ±0.3°C for temperature, ±2% for RH). R can be tuned from sensor specs or adapted by EM.

## EM auto-tune (Expectation-Maximization)

When `em_autotune: true`, the filter recursively adapts the diagonal of Q (process variance) and R (measurement variance) using forgetting factors (lambdas). **Delta** here refers to the derivative (rate) state components (dT_in, dT_out, dRH_in, dRH_out). Separate lambdas allow:

- **lambda_Q**: One forgetting factor for all of Q; higher (e.g. 0.995) = slower adaptation of process noise.
- **lambda_R_inlet**: For inlet temperature and humidity (indices 0, 1). Inlet is expected to change much less than outlet; use a higher value (e.g. 0.998) so R adapts slowly.
- **lambda_R_outlet**: For outlet temperature and humidity (indices 2, 3). Outlet can change significantly (e.g. ~1°C/s, ~1%/s); use a lower value (e.g. 0.98) so R adapts faster.
- **em_inflation**: Multiplier applied to the raw EM estimate (r_est, q_est) before exponential smoothing; not cumulative. Default 0.5 makes the smoothed R and Q 50% larger than the uninflated estimate.
- **em_warmup_steps** (default 20): Ramps the effective (1−λ) over the first N updates so R and Q stay near their initial values at startup, then transition smoothly to full EM. Set to 0 to disable warmup (full EM from first update).

Recommended: keep `em_lambda_r_inlet` > `em_lambda_r_outlet` so outlet measurement noise adapts faster than inlet. All lambdas must be in (0, 1]. Optional sensors (e.g. `em_q_t_out`, `em_r_t_out`) expose the current Q/R diagonal and lambda values for verification.

## Numeric types (single precision only)

All arithmetic uses **single-precision `float`** or **integers** only—no `double`. This keeps code fast and lean on ESP32/ESP8266. Use `float` and `1.0f`-style literals; avoid `double` and bare `1.0` when the value is used as float.

## Extending

- **Python** (`__init__.py`): extend `CONFIG_SCHEMA` and `to_code()` to add options (e.g. Q/R) and C++ wiring.
- **C++** (`hp_ukf.h` / `hp_ukf.cpp`): sensor reads, UKF predict/update, output publish.
- **UKF** (`hp_ukf_ukf.h` / `hp_ukf_ukf.cpp`): sigma points, time-discrete predict, measurement update with mask.

## License

Same as the parent esphome-snippets project.
