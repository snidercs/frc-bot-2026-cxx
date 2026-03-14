# Real-Time Performance Optimization Areas

This document tracks known performance concerns in the robot code that are
acceptable today but worth cleaning up before they become a problem —
especially anything that runs inside `RobotPeriodic()` or `processResults()`.

The robot loop runs at 60 Hz (16.7 ms per cycle). Every allocation, string
operation, or hash lookup in a hot path is time we're not spending on control.

---

## Known issues

### 1. String-keyed `unordered_map` for per-camera velocity gate state

**File:** `src/vision.hpp` / `src/vision.cpp`  
**Where:** `_cameraState[cameraName]` in `VisionIO::processResults()`  
**Introduced with:** Option E velocity gate

Every accepted result does a heap-allocated string hash and map lookup to
retrieve and update the `CameraGateState` for that camera. For two cameras
running at 60 Hz this is low-impact, but it is unnecessary allocator pressure
and cache unfriendly compared to a direct array index.

**Fix:** Assign each camera a stable integer index (0, 1, 2, …) at construction
time. Replace `std::unordered_map<std::string, CameraGateState>` with
`std::array<CameraGateState, N>` indexed by that integer. The camera name
is already positionally encoded in `vision::kCameraNames` and
`vision::kRobotToCamera` — the index is free.

```cpp
// Instead of:
auto& camState = _cameraState[cameraName];

// Prefer:
auto& camState = _cameraState[cameraIndex];  // cameraIndex passed alongside cameraName
```

This collapses to a pointer offset and eliminates the map entirely.

---

### 2. String concatenation in `SmartDashboard::PutNumber` keys

**File:** `src/vision.cpp`  
**Where:** Every `frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/X (m)", ...)` call  
**Frequency:** Every accepted measurement, every cycle under `BOT_TRACE_VISION`

Each call constructs a temporary `std::string` on the heap just to look up or
create the NT entry. NT does a hash-table lookup on the key every call anyway,
but constructing the key string freshly is avoidable.

**Fix:** Pre-build the full dashboard key strings once per camera at
construction time and store them alongside the camera unit. Then each
`PutNumber` call uses a `std::string_view` or a cached `std::string` reference
with no allocation.

```cpp
struct CameraUnit {
    // ...existing fields...
    std::string keyX;    // "Vision/FL/X (m)"
    std::string keyY;    // "Vision/FL/Y (m)"
    // etc.
};
```

---

### 3. `std::string` passed by value to `processResults()`

**File:** `src/vision.hpp`  
**Where:** `void processResults(const std::string& cameraName, ...)`  
**Frequency:** Once per camera per cycle

Minor, but the camera name should ideally be a `std::string_view` throughout
the pipeline since it always refers to a compile-time string constant in
`vision::kCameraNames`. Avoids any accidental copy.

---

### 4. `getRejectedCounts()` builds a new string every call

**File:** `src/vision.cpp`  
**Where:** `std::string VisionIO::getRejectedCounts()`

Concatenates seven `std::to_string()` conversions and several string literals
into a fresh heap allocation every time it's called. Fine if called rarely
(e.g. on button press), bad if called from a dashboard-update periodic.

**Fix:** Only a concern if this gets wired into a periodic. Log to NT as
individual counters instead of a single formatted string, or build the string
into a pre-allocated buffer.

---

### 5. `_candidates` vector re-used but `_measurements` grows unbounded

**File:** `src/vision.cpp`  
**Where:** `_measurements.push_back(best->measurement)` — never shrinks between calls  
**Note:** `_measurements.clear()` is called at the top of each subclass
`getMeasurements()`, so the *content* is correct. But the vector's *capacity*
can grow if many measurements are accepted over many cycles and the allocator
never reclaims the memory.

The `reserve()` in the constructor mitigates this, but worth confirming the
reserve size matches expected peak usage.

---

## Things that are fine

These are often flagged as "slow" but are not actually a concern here:

- `std::vector<Candidate> _candidates` — cleared and refilled each cycle, but
  the capacity is pre-reserved and no heap allocation happens after warmup
- `std::max_element` over `_candidates` — O(n) over at most a handful of
  candidates per camera; negligible
- `units::math::sqrt` / `pow<2>` for distance — compile-time-typed, no
  overhead beyond the float ops themselves

---

## Priority order

| Priority | Item |
|----------|------|
| Medium | #1 — integral camera IDs replacing string map |
| Low | #2 — pre-cached dashboard key strings |
| Low | #3 — `string_view` for camera name |
| Only if needed | #4 — `getRejectedCounts()` if wired to periodic |
| Monitor | #5 — `_measurements` capacity |

None of these are urgent at 60 Hz with two cameras. Revisit if camera count
increases (FR/BR addition) or loop rate increases above 100 Hz.
