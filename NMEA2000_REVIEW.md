# NMEA2000 Integration — Production Readiness Review

Thorough code review of the NMEA2000 branch across all 4 repos (`rusefi`, `NMEA2000`, `NMEA2000_rusefi`, `libfirmware`). The code works in testing but has issues that should be resolved before considering it production-ready.

Guiding principle: **zero regression risk** — every fix must be scoped carefully to avoid breaking what already works.

---

## CRITICAL — Safety / Correctness

### 1. Global `operator new`/`delete` override affects ALL firmware
**File:** `NMEA2000_rusefi.cpp` (tobsec/NMEA2000_rusefi)

The global `operator new` and `operator delete` are overridden with an 8KB bump allocator. This means *every* `new` in the entire rusEFI firmware — not just NMEA2000 — uses this allocator. If any other firmware code uses `new`, it silently consumes NMEA2000 heap. Worse, `delete` calls `NVIC_SystemReset()`, rebooting the ECU.

**Fix:** Scope the allocator to NMEA2000 only (placement new, or class-specific `operator new`).
**Regression risk of fix:** Low.

### 2. `#if EFI_CAN_SUPPORT` guard commented out
**File:** `firmware/controllers/can/can_dash.cpp` lines 12, 1704

The entire `can_dash.cpp` body (all dashboard types, not just NMEA2000) compiles unconditionally, including on builds where CAN is disabled. This breaks any board config with `EFI_CAN_SUPPORT=0`.

**Fix:** Restore the guard; wrap only the NMEA2000 includes/function separately.
**Regression risk of fix:** Low.

### 3. Bump allocator returns wrong pointer
**File:** `NMEA2000_rusefi.cpp` — `NMEA2kHeap::alloc()`

```cpp
myHeap_pos += n;
return &myHeap_p[myHeap_pos];  // BUG: returns pointer PAST the allocated block
```

Returns the address *after* the allocated region instead of at its start. Works now by accident due to sequential allocation patterns, but is actively wrong and will corrupt data under different allocation sizes.

**Fix:** Save `old_pos` before incrementing, return `&myHeap_p[old_pos]`.
**Regression risk of fix:** Low.

### 4. `m_memoryUsed` never updated
**File:** `NMEA2000_rusefi.cpp` — `NMEA2kHeap::alloc()`

`m_memoryUsed` is never incremented. `used()` always returns 0. Heap monitoring is non-functional.

**Regression risk of fix:** None.

---

## HIGH — Functional Issues

### 5. `TransmitMessages[]` PGN list doesn't match actual transmissions
**File:** `firmware/controllers/can/can_dash.cpp` line 26

Declared: `{130310L, 130311L, 130312L, 0}`
Actually sent: **127488, 127489, 127493, 130311**

PGNs 130310 and 130312 are never sent. PGNs 127488, 127489, 127493 are missing. Other N2K devices querying "what do you transmit?" get wrong metadata.

**Fix:** Update to `{127488L, 127489L, 127493L, 130311L, 0}`.
**Regression risk of fix:** None.

### 6. `CANSendFrame` always returns `true`
**File:** `NMEA2000_rusefi.cpp`

`CanTxMessage` sends via RAII in the destructor — there's no way to capture the result. The NMEA2000 library thinks every message succeeded, so it can't retry or report errors.

**Fix:** Medium effort — would require changing the send path to capture results.
**Regression risk of fix:** Medium.

### 7. CAN transmit timeout reduced from 100ms to 10ms globally
**File:** `firmware/hw_layer/drivers/can/can_msg_tx.cpp` line 83

This affects ALL CAN messages across all dashboard types, not just NMEA2000. Under bus load or error conditions, legitimate messages may be dropped.

**Fix:** Revert to 100ms globally, or make timeout configurable per-message/bus.
**Regression risk of fix:** Low.

### 8. `osalDbgAssert((&m_frame != nullptr))` is always true
**File:** `firmware/hw_layer/drivers/can/can_msg_tx.cpp` line 78

`&m_frame` takes the address of a member variable — can never be null. This assert is a no-op.

**Fix:** Remove.
**Regression risk of fix:** None.

---

## MEDIUM — Robustness

### 9. No debounce on oil pressure, fuel pressure, water flow, overtemp flags
**File:** `firmware/controllers/can/can_dash.cpp`

Battery voltage has a 5-second debounce (good), but oil pressure, fuel pressure, water flow, and overtemp have none. A single transient sensor glitch on one 1000ms cycle triggers `setError()` and the alarm.

**Fix:** Add similar debounce counters.
**Regression risk of fix:** Low.

### 10. `Sensor::getOrZero()` called repeatedly for same sensor in same cycle
**File:** `firmware/controllers/can/can_dash.cpp`

RPM is read 6+ separate times in the 1000ms block. Each call could return a different value if RPM changes between calls, leading to inconsistent threshold evaluation.

**Fix:** Read each sensor once into a local variable at the top of each cycle block.
**Regression risk of fix:** None.

### 11. Lambda scaling may overflow `uint16_t`
**File:** `firmware/controllers/can/can_dash.cpp` line 1445

`(uint16_t)(Sensor::getOrZero(SensorType::Lambda1)/0.0001)` — dividing by 0.0001 is multiplying by 10000. Lambda of 1.0 = 10000 (fits). Lambda >= 6.5536 overflows uint16_t. A disconnected wideband returning garbage could overflow.

**Fix:** Clamp before cast.
**Regression risk of fix:** None.

### 12. Emergency stop may trigger falsely on engine start
**File:** `firmware/controllers/can/can_dash.cpp` lines 1548-1553

MAP check `(mapValue == 0.0f) || (mapValue >= 101.0f)` has no RPM gate. During cranking, MAP can legitimately be 0 briefly or above 101 kPa at sea level before stabilizing.

**Fix:** Gate behind RPM > 400 like the other diagnostics, or add debounce.
**Regression risk of fix:** Low.

### 13. `static bool doOnce` declared but never used
**File:** `firmware/controllers/can/can_dash.cpp` line 1369

Dead code.

---

## LOW — Code Quality / Maintenance

### 14. Large amount of commented-out code
Old Arduino example code, BMW E46 test code, scheduler templates, debug variables throughout `can_dash.cpp` (~80 lines of comments).

### 15. Debug artifacts in `NMEA2000_rusefi.cpp`
`retValDebug`, `accCountCANOpen`, `accCountCANSendFrame`, `accCountCANGetFrame`, `timeBuf[16]`, `cur_time`, and the `if (id == 0x9f20016)` timestamp capture block. Also commented-out `bkpt()` in `can_msg_tx.cpp`.

### 16. `_README.txt` with personal build notes
`firmware/_README.txt` contains local toolchain path — should be gitignored or removed.

### 17. `ramdisk.image` (1MB binary) committed
Large binary that should likely be generated, not tracked in git.

---

## Recommended Fix Priority

| Priority | # | Description | Regression Risk |
|---|---|---|---|
| 1 | #3 | Bump allocator returns wrong pointer | Low |
| 2 | #1 | Global operator new/delete override | Low |
| 3 | #2 | EFI_CAN_SUPPORT guard commented out | Low |
| 4 | #7 | CAN timeout reduced globally | Low |
| 5 | #5 | TransmitMessages PGN list mismatch | None |
| 6 | #10 | Read sensors once per cycle | None |
| 7 | #12 | Emergency stop false trigger on start | Low |
| 8 | #9 | Missing debounce on alarm flags | Low |
| 9 | #11 | Lambda overflow | None |
| 10 | #14,15 | Clean up commented/debug code | None |

## Repos involved
- `tobsec/rusefi` (branch `NMEA2000`) — main firmware changes
- `tobsec/NMEA2000` (branch `rusefi`) — fork of ttlappalainen/NMEA2000
- `tobsec/NMEA2000_rusefi` (branch `rusefi`) — CAN driver adapter
- `tobsec/libfirmware` (branch `NMEA2000`) — compiler/linker flag changes
