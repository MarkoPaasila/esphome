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
| `track_temperature_derivatives`| boolean | `true`  | If true, state is 8D (T_in, RH_in, T_out, RH_out, dT_in, dT_out, dRH_in, dRH_out); if false, 4D (no derivatives). |
| `em_autotune`                 | boolean | `false` | Enable EM (Expectation-Maximization) auto-tune for process (Q) and measurement (R) noise with forgetting factors. |
| `em_lambda_q`                | float   | `0.995` | Forgetting factor for Q (process variance). Range (0, 1]; higher = slower adaptation. |
| `em_lambda_r_inlet`          | float   | `0.998` | Forgetting factor for R of inlet T and RH. Inlet changes little; use higher value. |
| `em_lambda_r_outlet`         | float   | `0.98`  | Forgetting factor for R of outlet T and RH. Outlet can change ~1°C/s, 1%/s; use lower value for faster adaptation. |

Optional EM sensors (only created if configured): `em_q_t_in`, `em_q_rh_in`, `em_q_t_out`, `em_q_rh_out`, `em_q_dt_in`, `em_q_dt_out`, `em_q_drh_in`, `em_q_drh_out` (process noise diagonal, variance units); `em_r_t_in`, `em_r_rh_in`, `em_r_t_out`, `em_r_rh_out` (measurement noise diagonal); `em_lambda_q_sensor`, `em_lambda_r_inlet_sensor`, `em_lambda_r_outlet_sensor` (current lambda values for debugging).

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

Recommended: keep `em_lambda_r_inlet` > `em_lambda_r_outlet` so outlet measurement noise adapts faster than inlet. All lambdas must be in (0, 1]. Optional sensors (e.g. `em_q_t_out`, `em_r_t_out`) expose the current Q/R diagonal and lambda values for verification.

## Numeric types (single precision only)

All arithmetic uses **single-precision `float`** or **integers** only—no `double`. This keeps code fast and lean on ESP32/ESP8266. Use `float` and `1.0f`-style literals; avoid `double` and bare `1.0` when the value is used as float.

## Extending

- **Python** (`__init__.py`): extend `CONFIG_SCHEMA` and `to_code()` to add options (e.g. Q/R) and C++ wiring.
- **C++** (`hp_ukf.h` / `hp_ukf.cpp`): sensor reads, UKF predict/update, output publish.
- **UKF** (`hp_ukf_ukf.h` / `hp_ukf_ukf.cpp`): sigma points, time-discrete predict, measurement update with mask.

## License

Same as the parent esphome-snippets project.
