# esp-ax25 Unit Tests

This directory contains the Unity-based unit test suite for the **esp-ax25** component library. Tests are structured as an ESP-IDF application that links against the component and runs on target hardware or QEMU.

## Directory Structure

```
test/
├── CMakeLists.txt                      # Top-level ESP-IDF project CMake
├── sdkconfig.defaults                  # Default sdkconfig for test builds
├── README.md                           # This file
├── main/
│   ├── CMakeLists.txt                  # Component registration (lists all test sources)
│   ├── Kconfig.projbuild               # Test-specific Kconfig options
│   ├── idf_component.yml               # Component dependency manifest
│   └── test_main.c                     # Unity runner entry point (app_main)
├── test_ax25_address.c                 # Address encoding/decoding tests
├── test_ax25_agwpe.c                   # AGWPE encoder/decoder tests
├── test_ax25_agwpe_server.c            # AGWPE server module tests  ← NEW
├── test_ax25_beacon.c                  # Beacon module tests
├── test_ax25_buffer.c                  # Buffer pool tests
├── test_ax25_conn.c                    # Connected-mode session tests
├── test_ax25_frame.c                   # Frame parsing/building tests
├── test_ax25_kiss.c                    # KISS framing tests
├── test_ax25_log_tcp_server.c          # Log TCP server tests
├── test_ax25_monitor_tcp_server.c      # Monitor TCP server tests
├── test_ax25_phy_kiss_tcp_client.c     # PHY TCP client tests
├── test_ax25_phy_tcp_server.c          # Base TCP server tests
├── test_ax25_phy_kiss_uart.c           # PHY UART KISS tests
├── test_ax25_phy_kiss_tcp_server.c     # PHY TCP server tests
├── test_ax25_print.c                   # Pretty-printing tests
├── test_ax25_router.c                  # Router forwarding tests
├── test_ax25_uart.c                    # UART helper tests
└── test_ax25_wifi.c                    # Wi-Fi bootstrap tests
```

Memory-intensive stress tests were moved to the dedicated `test_stress/` app to
keep the regular unit-test binary within DRAM limits.

## Prerequisites

- **ESP-IDF v5.x** (v5.1+ recommended) installed and sourced (`$IDF_PATH` set)
- A target board (ESP32, ESP32-S3, etc.) connected via USB, **or** QEMU for host testing
- The `esp-ax25` component accessible at `../../` relative to this directory (already configured in `CMakeLists.txt`)

## Building & Running

### On Real Hardware

```bash
# From the test/ directory:
cd /path/to/esp-ax25/test

# Set target (adjust for your board)
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

Unity test output appears on the serial console. All `TEST_CASE` macros are auto-discovered by `unity_run_all_tests()`.

### Running a Subset of Tests

Use Unity's tag-based filtering. Each test is tagged (e.g., `[ax25_agwpe_server]`). To run only the AGWPE server tests from the serial monitor, type:

```
[ax25_agwpe_server]
```

at the Unity prompt (if interactive mode is enabled in `sdkconfig`).

### With QEMU (Host Testing)

```bash
idf.py set-target esp32
idf.py build
idf.py qemu monitor
```

## Test Coverage — `test_ax25_agwpe_server.c`

The AGWPE server test file (`test_ax25_agwpe_server.c`) provides comprehensive coverage of the `ax25_agwpe_server` module:

| Category | Tests | Description |
|----------|-------|-------------|
| **Initialization** | 6 | Valid init, NULL args, custom config, deinit safety |
| **Version/Info** | 3 | `'R'` version, `'G'` port info, `'g'` capabilities |
| **Registration** | 4 | `'X'` register, register with SSID, `'x'` unregister, re-register |
| **Raw/Monitor Mode** | 5 | `'k'` toggle, `'m'` toggle, I/S frame types, dual mode |
| **Transmit** | 5 | `'M'` unproto, `'V'` via digis, `'K'` raw, edge cases |
| **Connection** | 6 | `'C'` connect, `'v'` via, `'c'` PID, duplicate, slots exhausted |
| **Data/Disconnect** | 2 | `'D'`/`'d'` to non-existing connection |
| **Outstanding** | 2 | `'y'` port, `'Y'` connection |
| **Error Handling** | 5 | NULL pointers, router port query |
| **Integration** | 6 | Frame routing, incoming SABM, rapid commands, port propagation |
| **Total** | **~40** | |

### Test Architecture

Tests use a **capture callback** pattern instead of full TCP mocking:

1. A `capture_cb` function is installed as the server's `on_agwpe_frame` callback
2. Every AGWPE frame the server would send to a TCP client is recorded in a ring buffer
3. Tests inject AGWPE commands via `ax25_agwpe_server_agwpe_in()` and AX.25 frames via `ax25_agwpe_server_ax25_in()`
4. After a short delay (to let the FreeRTOS TX task drain the queue), tests inspect the captured frames

Helper functions simplify test construction:
- `make_request()` — build minimal AGWPE request frames
- `make_ui_frame()`, `make_i_frame()`, `make_rr_frame()`, `make_sabm_frame()` — build AX.25 frames
- `find_captured()` — search captured output by frame kind
- `wait_for_frames()` — wait for the async TX task to deliver frames

### Adding New Tests

1. Add a new `TEST_CASE(...)` in `test_ax25_agwpe_server.c` with the tag `[ax25_agwpe_server]`
2. Use `create_server()` / `destroy_server()` for setup/teardown
3. Use `capture_reset()` before the code under test
4. Inject input, wait, then assert on captured output

No changes to `CMakeLists.txt` are needed — Unity auto-discovers `TEST_CASE` macros at link time (enabled by `WHOLE_ARCHIVE`).

## Troubleshooting

- **Stack overflow**: Increase `AX25_AGWPE_SERVER_TASK_STACK_SIZE` or the test task stack in `sdkconfig`
- **Timeout in `wait_for_frames`**: The TX queue task may be starved; increase `tskIDLE_PRIORITY` or reduce other task priorities
- **Router not initialised**: Ensure `ax25_router_init()` is called before `ax25_agwpe_server_init()` — the `create_server()` helper does this automatically
