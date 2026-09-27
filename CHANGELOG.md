# 1.5.0

- Keep the existing event ABI and Air behavior while adding a separate GT/Gemini orientation channel.
- Add automatic GT HID interface/endpoint selection on libusb platforms.
- Correct the GT `0x65` trailer parse without changing the existing Air parse path.
- Add GT `0x3C` factory calibration and `0x3E` multi-page temperature gyro-bias retrieval.
- Add GT runtime-axis preparation and optional VQF 2.1.2 6D/9D host-side orientation.
- Add `examples/gt_probe` and VQF third-party notices.

# 0.1.0
Introduce semver for convience
And any oldest changes (im will DO NOT describe here)

...