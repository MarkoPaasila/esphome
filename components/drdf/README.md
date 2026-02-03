# DRDF – Dynamic Reversals based Deadband Filter

ESPHome sensor filter that implements a **dynamic deadband** based on observed trend reversals. The deadband size adapts from the sensor’s reversal pattern (often related to noise); the output is always the midpoint of the sliding deadband.

## Configuration

Use `drdf` in any sensor’s `filters` list like other built-in filters.

### Minimal (defaults)

```yaml
sensor:
  - platform: dallas_temp
    # ...
    filters:
      - drdf: {}
```

### With options

```yaml
filters:
  - drdf:
      alpha: 0.01
      ema_multiplier: 3.82
```

| Option           | Default | Description |
|------------------|--------|-------------|
| `alpha`          | `0.01` | EMA smoothing factor for reversal differences. Smaller = slower adaptation. |
| `ema_multiplier` | `3.82` | Multiplier for deadband size (deadband = EMA × multiplier). Smaller = tighter band, more noise in output. |

## Behavior

- Tracks trend (up / down / neutral) and detects **consecutive reversals** (e.g. up→down→up).
- On two consecutive reversals, computes the absolute difference between reversal points and updates an EMA of these differences.
- Deadband size = EMA × `ema_multiplier` (default 3.82 is aimed at ~99% of reversals if roughly normal).
- Deadband **slides** with the raw value (no extra latency); output is always the center: `(upper_bound + lower_bound) / 2`.

## Filter order

- **Before DRDF:** Avoid median, EMA, sliding window average, throttle_average, quantile, debounce, heartbeat, throttle, min, max, round — they can reduce accuracy or add latency. `filter_out` for known bad values is fine.
- **After DRDF:** Optional: calibrate, clamp, delta, heartbeat, throttle, max, min, multiply, offset, quantile, round. No extra smoothing usually needed.
- Calibration can be before or after DRDF.

## Example

```yaml
sensor:
  - platform: dht
    temperature:
      name: "Living Room Temperature"
      filters:
        - filter_out: 0.0   # optional: drop known bad raw values
        - drdf: {}
        - heartbeat: 500s  # optional: after DRDF
```

Internal state (`ema_value`, `deadband_size`) resets on reboot; persistence is not implemented in this component.
