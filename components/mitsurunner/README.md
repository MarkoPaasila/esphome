# MitsuRunner

## 1. Description

MitsuRunner is an ESPHome component that **smart-controls defrost** on Mitsubishi (and compatible) heat pumps. It drives a relay that switches the outdoor unit’s defrost NTC thermistor: either the **real NTC** (unit’s own defrost logic) or a **fixed 33 kΩ resistor** (so the unit “sees” a warmer coil which prevents defrost). Using heat-exchanger and outdoor temperature sensors, a state machine decides when to allow defrost — reducing unnecessary defrosts and ensuring defrost when the coil is frosted.

---

## 2. Example YAML configuration

```yaml
mitsurunner:
  ## Necessary options:
  heat_exchanger_temperature_sensor_id: heat_exchanger_temp   # Outdoor coil temperature (e.g. DS18B20 on coil)
  outdoor_temperature_sensor_id: outdoor_temp                  # Ambient outdoor air temperature
  defrost_relay_id: allow_defrost_relay                        # Switch that selects NTC vs 33kΩ (see Hardware)
  ## Optional keys. Default values shown.
  # update_interval: 60s                                       # How often the state machine runs (default 60s)
  # smart_defrost_logic_state:                                 # Optional: expose defrost logic state as text sensor (name default: "Smart Defrost Logic State")
  #   name: "Smart Defrost Logic State"
  # defrost_now:                                               # Optional: button to request manual defrost now (only created if configured)
  #   name: "Defrost Now"                                      # default
  #   id: defrost_now                                          # optional id
  # temperature_delta_to_defrost: -5.0                           # Coil vs outdoor delta below which we consider defrost (default -5.0)
  # outdoor_temperature_to_enter_off_state: 3.0                   # Outdoor temp above this: disable defrost logic (default 3.0)
  # outdoor_temperature_to_exit_off_state: 2.0                   # Outdoor temp below this: re-enable (hysteresis) (default 2.0)
  # heat_exchanger_max_temperature: 10.0                        # Coil temp above this: enter OFF (default 10.0)
  # temperature_delta_defrosting_started: 1.0                   # Delta at which we consider defrost cycle started (default 1.0)
  # temperature_delta_excess_time_min: 8                        # Minutes delta must be bad before allowing defrost (default 8)
  # temperature_delta_decreasing_excess_time_min: 5             # Same when delta is improving (default 5)
  # max_heating_time_min: 180                                    # Max heating before forcing defrost allowed (default 180)
  # min_heating_time_min: 50                                     # Min heating time between defrosts (default 50)
  # defrost_duration_min: 30                                     # Assumed defrost cycle length in minutes (default 30)
  # defrost_timeout_min: 10                                      # Wait for unit to start defrost before "long defrost" (default 10)
  # reset_sensor_delay_sec: 25                                   # Delay after boot before using sensors (default 25)
  # initialize_delay_sec: 60                                     # Delay before state machine runs after boot (default 60)
  # defrost_allowed_sensor:                                    # Optional: binary sensor true when defrost is currently allowed
  #   name: "Defrost Allowed"
  # temperature_delta_sensor:                                   # Optional: heat_exchanger − outdoor (negative = frost condition)
  #   name: "Outdoor Coil Temperature Delta"
  # smart_defrost_logic:                                       # Optional: switch to enable/disable the smart logic (relay follows it)
  #   name: "Smart Defrost Logic"
```

---

## 3. Logic (state machine)

- **Temperature delta** = heat exchanger temperature − outdoor temperature. Negative means the coil is colder than ambient (frost condition).

- **Startup:** Relay is off (unit sees real NTC). After `initialize_delay_sec`, the component waits `reset_sensor_delay_sec` in **Reset**, then goes to **Off**, **Defrosting** (if delta already high), or **Idle**.

- **Off**  
  Enter when: outdoor > `outdoor_temperature_to_enter_off_state`, or heat exchanger > `heat_exchanger_max_temperature`, or smart-defrost switch is off.  
  Relay off. Exit when outdoor < `outdoor_temperature_to_exit_off_state`, coil < max temp, and smart-defrost is on → **Idle**.

- **Idle**  
  Relay off. A “max heating” timer runs; if it reaches `max_heating_time_min` or user triggers manual defrost → **Start defrosting**. If delta ≤ `temperature_delta_to_defrost` → **Temp exceeded**.

- **Temp exceeded**  
  Delta is below threshold (frost likely). Relay still off. Waits `temperature_delta_excess_time_min`. Then: if delta is improving (min over a short window) and time passed → **Temp exceeded (decreasing)**; if not improving and time passed (or max heating / manual) → **Start defrosting**; if delta rises above threshold → **Idle**.

- **Temp exceeded (decreasing)**  
  Shorter wait `temperature_delta_decreasing_excess_time_min`. If delta > threshold → **Idle**. If time passed or max heating or manual → **Start defrosting**.

- **Start defrosting**  
  Relay **on** (unit sees 33 kΩ, can start defrost). If delta rises to ≥ `temperature_delta_defrosting_started` → **Defrosting started**. If `defrost_timeout_min` passes without that → **Long defrosting started** (relay off again; unit may defrost on its own).

- **Defrosting started / Long defrosting started**  
  After `defrost_duration_min` → **Forced heating** (relay off, minimum heating time).

- **Forced heating**  
  Relay off for `min_heating_time_min - defrost_duration_min` to avoid immediate re-defrost. Then → **Idle**. If max heating time is reached during this period → **Start defrosting** again.

- **Sensor fault**  
  If sensors are missing, out of range, or outdoor sensor is stale (e.g. > 5 min), defrost is disallowed (relay off). After 60 minutes in fault, relay is forced off. Recovery goes to **Off** or **Reset** depending on temperatures.

- **Smart-defrost switch**  
  When the optional “Smart Defrost Logic” switch is off, component enters **Off** (relay off): unit always sees the real NTC and uses its built-in defrost logic.

---

## 4. Hardware setup (MitsuRunner-specific)

- **Relay:** Single-pole (or equivalent) so the heat pump’s defrost NTC input is either connected to the **real NTC** or to a **33 kΩ resistor**.
  - **Common:** Connect to the outdoor unit’s defrost NTC input (where the original NTC was or in series as required by your unit).
  - **NC (normally closed):** Connect the **NTC thermistor** here.
  - **NO (normally open):** Connect a **33 kΩ resistor** here.

  So when the relay is **not energized** (e.g. ESP off or MitsuRunner off), the unit sees the **NTC** and uses its original defrost behavior. When the relay **is energized**, the unit sees the **33 kΩ** and MitsuRunner controls when defrost is allowed.

- **ESPHome relay switch:** Configure the GPIO switch for this relay as **inverted** so that when MitsuRunner turns the switch **on**, the relay energizes and the unit sees the 33 kΩ (defrost allowed). Example:

  ```yaml
  switch:
    - platform: gpio
      pin: { number: 23, inverted: true }
      name: "Allow Defrost Relay"
      id: allow_defrost_relay
  ```

- **Sensors:** You need two temperature sensors (e.g. DS18B20 or similar):
  - **Heat exchanger:** Mounted on or near the outdoor coil (used for delta and “defrost started”).
  - **Outdoor:** Ambient outdoor air (used for delta and OFF-state hysteresis).

- No other MitsuRunner-specific hardware is required; the rest is standard ESP/ESPHome (power, enclosure, etc.).

---

## 5. Getting started with ESPHome

- [ESPHome Getting Started](https://esphome.io/guides/getting_started_command_line.html)  
- [ESPHome Configuration](https://esphome.io/guides/configuration-types.html)  
- [Custom components (local)](https://esphome.io/components/external_components.html) — use `type: local` and point `path` to the folder containing the `mitsurunner` component.
