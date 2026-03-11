# PhotonVision Camera Calibration

Camera calibration gives PhotonVision accurate focal length and lens distortion
coefficients so that the PNP (Perspective-n-Point) solver can compute correct
3D poses from AprilTag detections. **Without calibration every pose estimate
will be wrong and no amount of RoboRIO-side filtering will fix it.**

---

## What You Need

- A printed **8×8 checkerboard** calibration target (or the MirrorLake board).  
  Download from: <https://docs.photonvision.org/en/latest/docs/calibration/calibration.html>  
  Print at **exact scale** — do not let the printer scale to fit. Measure a square
  with calipers and confirm it matches the size you will enter in PhotonVision.
- A flat, rigid backing for the printout (foam board, clipboard, etc.).
- Access to the PhotonVision UI on each Pi:
  - Pi 1 (FL camera): `http://10.TE.AM.11:5800` (or `http://photonvision.local:5800`)
  - Pi 2 (BL camera): `http://10.TE.AM.12:5800`

---

## Steps

### 1. Open the PhotonVision UI

Navigate to the Pi's address in a browser. Select the camera you want to
calibrate from the camera dropdown in the top bar.

### 2. Go to the Calibration Tab

Click **"Calibration"** in the left sidebar.

Fill in:
| Field | Value |
|-------|-------|
| Board type | Chessboard |
| Pattern width | 8 (interior corners) |
| Pattern height | 8 (interior corners) |
| Square size | _measure your printout_ (e.g. `25.0 mm`) |

### 3. Capture Snapshots

Click **"Start Calibration"**.

Move the checkerboard in front of the camera and click **"Take Snapshot"** each
time PhotonVision highlights the detected corners in green. Aim for:

- **At least 25 snapshots** (more = better)
- Cover **all regions** of the frame: corners, edges, centre
- Vary the **tilt and rotation** of the board — don't keep it flat-on every time
- Vary the **distance**: close, mid, far
- Avoid motion blur — hold the board still before clicking

The progress bar will fill as you capture. Blurry or partially-visible snapshots
are automatically rejected.

### 4. Finish and Review

Click **"Finish Calibration"**. PhotonVision will compute the intrinsic matrix
and distortion coefficients. Review the reported **reprojection error** (RMS):

| RMS error | Quality |
|-----------|---------|
| < 1.0 px  | Excellent |
| 1.0–2.0 px | Acceptable |
| > 2.0 px  | Redo — too much blur or board flex |

If the error is too high, delete the bad snapshots and recapture.

### 5. Verify the 3D Pipeline

Switch to the **"AprilTag"** pipeline for this camera. In the camera stream you
should see:

- Green tag outlines with tag IDs labelled
- A pose axes overlay (X/Y/Z arrows) on each detected tag
- Reasonable-looking distance readouts in the **"Targets"** panel

If the axes are spinning wildly or the distance is clearly wrong, the
calibration did not apply correctly — repeat from step 2.

### 6. Set the Field Layout

In **Settings → AprilTag Field Layout**, confirm it is set to:

```
2026 Rebuilt AndyMark
```

This must match `frc::AprilTagField::k2026RebuiltAndyMark` used in the
RoboRIO code (`vision.hpp` → `fieldLayout()`).

### 7. Confirm Robot-to-Camera Transform

In the camera's **"3D"** settings, the robot-to-camera transform should be
**left at the default (all zeros)** because the transform is applied on the
RoboRIO side via `vision::kRobotToCamera` in `vision.hpp`. Entering it in
both places will double-apply it and corrupt the pose.

---

## Repeat for Every Camera

Each physical camera has its own lens and must be calibrated independently.
Calibration is stored on the Pi and survives reboots. Re-calibrate if:

- A camera is replaced or repositioned
- The lens focus ring is adjusted
- A Pi is reflashed

---

## Quick Checklist

- [ ] FL camera calibrated (RMS < 1.5 px)
- [ ] BL camera calibrated (RMS < 1.5 px)
- [ ] Field layout set to `2026 Rebuilt AndyMark` on both Pis
- [ ] Robot-to-camera transform is **not** set in PhotonVision UI (handled in code)
- [ ] Both cameras show correct tag IDs and stable pose axes in the stream
