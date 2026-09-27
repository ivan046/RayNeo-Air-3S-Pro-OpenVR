# GT Probe

Diagnostic example for the additive GT/Gemini API introduced in SDK 1.5.
It does not change the existing event API or the OpenVR driver.

```text
RayNeoGtProbe              # 6D VQF
RayNeoGtProbe --auto-9d    # magnetometer-assisted VQF with disturbance rejection
RayNeoGtProbe --force-9d   # diagnostic: disable magnetic disturbance rejection
```

The probe prints the `0x3C` factory transform/accelerometer offset,
checks collection of the complete `0x3E` temperature-indexed gyro-bias table,
and then streams the separate `RAYNEO_GtOrientation` channel.
