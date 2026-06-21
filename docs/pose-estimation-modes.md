# Pose-estimation modes: Trackers (C) and Feature Matching (D)

Both modes answer the same question — *given only what the camera sees, where
is the camera?* — by building 2D–3D correspondences and solving
Perspective-n-Point. They differ only in **where the correspondences come
from**: Mode C plants known landmarks in the scene; Mode D uses what the
terrain itself looks like.

## Shared backbone: `PoseComparisonState`

`src/state/PoseComparisonState.{h,cpp}` is the common base. Both modes:

- fly the player camera freely (same controls as NAVIGATION);
- on **B**, capture a *timestep*: the camera's pose right now is recorded as
  ground truth, and the mode's `computePose` estimates that same pose from
  the rendered frame alone — both go into a `PoseLog`, computed may be empty;
- on **N / M**, step the review cursor through captured timesteps, snapping
  the camera to that timestep's true pose;
- draw the comparison identically: the global view shows both fly-through
  paths (true = blue with red/green waypoint dots, computed = orange), and
  the player view overlays the computed pose as a translucent orange ghost
  while the camera sits on the reviewed true pose. The closer the ghost
  aligns with the real terrain, the better that timestep's estimate.

The one pure virtual is `computePose` — each mode's correspondence pipeline.

## Mode C — Trackers (`T`)

Pipeline (`src/state/TrackersState.cpp` + `src/vision/TrackerDetection.cpp`):

1. **Placement** — on entry the mode prompts for a tracker count (1–20,
   default 7) and scatters that many spheres on random terrain vertices,
   re-rolling for minimum separation (clumped trackers hand PnP a
   near-degenerate configuration). Each sphere has a unique palette color;
   every palette pair differs by ≥ 0.5 in at least one channel.
2. **Capture pass** — on B, `Renderer::captureTrackersFrame` re-renders the
   player view with the terrain flat black and the spheres in their flat,
   unlit palette colors (depth-tested, so occlusion matches the visible
   frame), and reads the pixels back.
3. **Detection** — `findTrackerCentroids` classifies every pixel against the
   palette (tolerance ±3 per channel) and returns each blob's centroid; blobs
   under 6 pixels are dropped as untrustworthy.
4. **Solve** — each visible centroid pairs with its sphere's known 3D center;
   with ≥ 4 pairs, `computeCameraPose` (SQPnP) recovers the pose.

The colored spheres are artificial fiducials: the unique color *is* the data
association, so "which 2D point matches which 3D point" — usually the hard
part — is answered by a single pixel scan.

## Mode D — Feature Matching (`F`, requires recorded waypoints)

No markers; the salient points come from ORB features
(`src/vision/FeatureMatching.cpp`). Like Mode B, the 2D→3D correspondence is
**manual** — ORB only *suggests* where to look; the user supplies the 3D. `F`
prompts for the number of features per view (default 5). Two phases:

**Pre-phase (G) — interactive, by hand.** The build steps through each recorded
waypoint: it poses the player (right) view at that waypoint and runs ORB on it
(`detectTopFeatures`), keeping the strongest N keypoints. They are presented
**one at a time** — the active suggestion is a white screen-space marker in the
player view — and the user **color-picks its matching 3D point on the global
(left) map** (`pickVertex(globalView)` → `mesh.worldPos`, the same color-pick
Mode B uses). Press **X** to skip an unplaceable suggestion, and use the global-map
mouse controls (scroll = zoom, middle-drag = pan, right-drag = orbit) to frame
hard-to-see spots, including behind mountains. Each placement stores
`(descriptor, hand-picked 3D anchor)` in the `FeatureDb`; when every view is
done the database is built. The 2D ORB position is *only* on-screen guidance — it
is never stored or fed to PnP; the database's 3D comes entirely from the user.

**Run-phase (B)** — unchanged and automatic: capture the current view, detect
ORB features, brute-force Hamming `knnMatch` against the hand-built database,
keep matches passing Lowe's ratio test (0.75), and solve with
`computeCameraPoseRansac`. Because the hand-built database is small (≈N per
view), the inlier floor is modest (6) rather than the automatic version's 25.

The console reports keypoints → confident matches → RANSAC inliers per capture.

## The lighting experiment (PDF p. 54) — reframed around ORB detection

The scene has one directional light (Lambert + ambient, `src/core/Lighting.h`)
with four presets cycled by **L**: late-morning sun, noon sun, low warm sun,
overcast. The light deliberately survives terrain swaps and menu round-trips.

Because Mode D's anchoring is now manual, the experiment is about **what ORB
detects**, not about automatic descriptor matching. ORB keys on local intensity
gradients, and on this shading-driven terrain those gradients *are* the relief
lit by the sun — so moving the light moves the salient points themselves.

Procedure:

1. Record a flight with several waypoints (R, B, B, ...).
2. Press F, then G, and note where ORB highlights its suggestions for the first
   view (their on-screen positions).
3. Press Escape back, change the light with **L**, re-enter F → G on the *same*
   waypoints. Compare: the suggested points land in **different places** (and
   different counts survive), because the corners ORB finds shifted with the
   shading.

Observed: a light-*direction* change relocates most suggestions and changes
which features are even detected; only a brightness/ambient change (same
direction) leaves them roughly stable. The historical automatic-matching
collapse (database built under one light, run-phase frame re-lit → confident
matches fall from tens–hundreds to ≈2–14, pose refused) is recorded in
`lighting-experiment.md`; it has the same root cause — shading-driven, not
texture-driven, appearance.

Run it on at least two terrains (the menu lists everything in
`assets/terrains/`).

## Pose stability vs. number of correspondences

Mode C's configurable tracker count (the 1–20 prompt on **T**) doubles as a
stability experiment: capture the same scene with different numbers of trackers
in view and watch the pose error fall as correspondences are added.

Measured (terrain1, mountainous → well-spread, non-coplanar trackers):

| Trackers visible | Position error (world units, terrain ≈ 633 wide) |
| --- | --- |
| 4 (PnP minimum) | ~24 |
| 5 | ~2.3 |
| 6 | ~0.2 |

Why: PnP recovers six degrees of freedom, so four correspondences is the bare
minimum with no redundancy to average out centroid/pixel noise — the solution
is well-determined in principle but ill-conditioned in practice. Each extra
correspondence over-determines the system and least-squares averages the noise
down quickly. Accuracy depends on *geometry*, not just count: collinear points
(e.g. picks along one ridge) or trackers clustered into a small image region
stay ill-conditioned however many there are, because they fail to constrain all
six DOF. The instability is a real property of PnP, and observing it is the
point of the experiment — not a defect to suppress.

A note on metrics: the `position error` printed per capture is available only
because this is a simulation — we know the true pose. A real system never does;
its honest self-confidence signal is the **reprojection error** (how well the
recovered pose re-predicts the observed pixels), which is exactly what
`computeCameraPoseRansac` uses internally as its inlier test. The marker/pick
solver `computeCameraPose` reports only the simulation's position error and runs
no reprojection sanity check, so a degenerate solve is presented the same way as
a precise one — fine for an experiment that *wants* to see the instability, but
worth knowing when reading the numbers.

## Keyboard reference

| Key | Where | Action |
| --- | --- | --- |
| Esc | anywhere | back to the terrain menu |
| Ctrl+Q | anywhere | quit |
| L | anywhere | cycle the light preset |
| R | anywhere | RECORD mode (fresh recording) |
| Ctrl+R | anywhere | PLAYBACK mode (needs waypoints) |
| P | anywhere | PICK mode (needs waypoints) |
| T | anywhere | TRACKERS mode (prompts for count) |
| F | anywhere | FEATURE MATCH mode (needs waypoints; prompts for feature count) |
| W/A/S/D + arrows | moving modes | fly / look |
| Q / E | moving modes | altitude up / down |
| scroll wheel | over global view | zoom the global map in / out |
| middle-drag | over global view | pan the global map |
| right-drag | over global view | rotate (orbit) the global map — see behind mountains |
| B | RECORD | drop a waypoint |
| B | TRACKERS / FEATURE MATCH | capture a timestep (true + computed pose) |
| N / M | TRACKERS / FEATURE MATCH | review next / previous timestep |
| G | FEATURE MATCH | start the manual database build (steps through each view) |
| left-click | FEATURE MATCH build | color-pick the active suggestion's 3D point on the global map |
| X | FEATURE MATCH build | skip the active (unplaceable) suggestion |
| click | PICK | pick a 2D–3D correspondence |
| C | PICK | solve pose |

## Headless checks

`make check` builds `tests/headless_checks.cpp` against the vision + loader
objects only and verifies on synthetic inputs: terrain normals against the
analytic plane normal, blob centroids (including the not-visible and
too-small cases), and both PnP solvers round-tripping a known camera — the
RANSAC variant with 3 of 11 correspondences corrupted. Exit code = number of
failed checks.
