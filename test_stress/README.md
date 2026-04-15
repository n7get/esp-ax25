# esp-ax25 Stress Tests

This app contains memory-intensive stress tests that are split out from the
regular `test/` suite so normal unit tests stay within DRAM limits.

## Build

```bash
cd test_stress
idf.py build
```

## Run

```bash
idf.py -p <PORT> flash monitor
```
