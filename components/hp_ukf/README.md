# hp_ukf – ESPHome external component

External ESPHome component **hp_ukf** (HP-UKF). Use it from a device YAML by loading it as a local external component.

## Layout

```
components/
  hp_ukf/
    __init__.py    # Config schema and codegen
    hp_ukf.h       # C++ header
    hp_ukf.cpp     # C++ implementation
    example_hp_ukf.yaml
    README.md
```

## Usage

1. In your device YAML (same level or above the `components` folder), add:

```yaml
external_components:
  - source:
      type: local
      path: components   # path to folder containing hp_ukf
    components: [ hp_ukf ]

hp_ukf:
  id: my_hp_ukf
  update_interval: 60s
```

2. Optional: set `update_interval` (default `60s`).

3. Compile and upload as usual. The component runs `setup()` once and `update()` at `update_interval`.

## Configuration

| Option             | Type   | Default | Description                |
|--------------------|--------|---------|----------------------------|
| `id`               | string | (auto)  | Instance ID for automations |
| `update_interval`  | time   | `60s`   | Interval for `update()`    |

## Extending

- **Python** (`__init__.py`): extend `CONFIG_SCHEMA` and `to_code()` to add options and generated C++.
- **C++** (`hp_ukf.h` / `hp_ukf.cpp`): add state, sensors, or other logic in `setup()` and `update()`.

## License

Same as the parent esphome-snippets project.
