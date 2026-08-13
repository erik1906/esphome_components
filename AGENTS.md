# Agent Notes

## Scope
- This repository is an ESPHome external component: `components/jsdrive/` is entire implementation.
- `__init__.py` defines YAML schema and ESPHome code generation; `jsdrive.h` exposes `JSDrive`; `jsdrive.cpp` implements UART relay, frame parsing, desk movement, and optional GPIO wake/passthrough.
- Component requires ESPHome `uart` and auto-loads `sensor` and `binary_sensor`. Keep Python schema, generated setter calls, and C++ setters synchronized.

## Protocol Safety
- Desk frames start with `0x5a`; remote frames start with `0xa5`. Both parsers accumulate bytes after header and validate checksum before publishing state or forwarding button data.
- Desk frame body length is configurable only from 5 through 6; checksum position and type-byte validation depend on that setting. Preserve both formats when changing parsing.
- Do not publish height or button state from malformed/incomplete frames. Clear receive buffers after every completed or rejected frame.
- `move_to()` sends repeated up/down remote-format frames until decoded height reaches target; it needs `desk_uart`. Keep movement state and `current_operation` consistent when changing this path.

## Validation
- No tests, CI, formatter, or device YAML are committed. Validate against a local ESPHome device config that imports this component:
  ```yaml
  external_components:
    - source:
        type: local
        path: /home/erde1931/src/esphome_components/components
      components: [jsdrive]
  ```
- Run `esphome config path/to/device.yaml` before `esphome compile path/to/device.yaml`; compile is regression gate for both Python codegen and C++.
