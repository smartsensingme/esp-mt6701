# Dual I2C driver hardware test

This project is a passive standalone MT6701 diagnostic. Do not add robot,
calibration, LUT, PID or bridge dependencies. Board pins/addresses are local
Kconfig values, not library defaults. The C3 target is single-core.

The measured release is the 200-Hz GPTimer auto-reload event (integer 5000 us at
1 MHz). Owner priority 8; bounded sensor transactions 1 ms each. Complete each
pair/accounting within the measured 5-ms cycle; stage budgets are 1 ms per sensor.
The diagnostic records wake jitter/deadline/backlog violations instead of
claiming a hard guarantee or repeatedly attempting to catch up. Pass the whole
notification count and acquire one pair per wake. Inferred elapsed-period losses
use the startup software epoch, not an external timing reference.

Initialize state before enabling releases. ISR only captures timestamp/notifies.
The one owner reads both sensors, records actual I2C-midpoint sample times and
numeric statistics. Failed samples count/reset derivative history; no fictitious
zero-speed replacement. There is no actuation/fault latch to clear or bypass.

No allocation, logging or USB formatting in the periodic phase. Stop/delete the
timer before reporting, release bus/device handles, keep watchdogs enabled.
Normal sensor reads can fail without ending this diagnostic; setup/resource API
failures abort. No NVS or motor command is permitted. Hardware validation remains
separate from native regression and ESP-IDF compilation.
