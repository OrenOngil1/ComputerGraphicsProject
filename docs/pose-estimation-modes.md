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

No markers; correspondences come from ORB features
(`src/vision/FeatureMatching.cpp`). Two user-driven phases:

**Pre-phase (G)** — for each recorded waypoint, render that view *under the
current light* twice through the Renderer: once lit
(`captureSceneFrame`) and once through the pick shader decoded per pixel into
vertex ids (`captureVertexIdFrame`). ORB keypoints detected on the lit frame
are anchored to 3D by looking up the vertex id under each keypoint pixel
(keypoints over background drop out). Everything lands in one `FeatureDb`:
stacked descriptors + a 3D anchor per row. G rebuilds from scratch — mixing
two lightings in one database would muddy the experiment.

**Run-phase (B)** — capture the current view (again under the current light),
detect ORB features, brute-force Hamming `knnMatch` against the database,
keep matches that pass Lowe's ratio test (best clearly beats the runner-up,
0.75), and hand the surviving 2D–3D pairs to `computeCameraPoseRansac`.
RANSAC instead of plain least squares because descriptor matching always lets
some wrong pairs through: candidate poses are fitted on small random subsets
and the one most correspondences agree with (8 px reprojection band) wins.

The console reports keypoints → confident matches → RANSAC inliers per
capture — the numbers the lighting experiment reads.

## The lighting experiment (PDF p. 54)

The scene has one directional light (Lambert + ambient, `src/core/Lighting.h`)
with four presets cycled by **L**: late-morning sun, noon sun, low warm sun,
overcast. The light deliberately survives terrain swaps and menu round-trips.

Procedure:

1. Record a flight with several waypoints (R, B, B, ...).
2. Press F, then G — the database snapshots the terrain's appearance under
   the *current* preset.
3. Press L one or more times — the scene now looks different, but the
   database still remembers the old appearance.
4. Fly near the recorded views and press B at several poses; compare the
   match/inlier counts and position errors against a control run where the
   light never changed.

Measured outcome (terrain1 — see `lighting-experiment.md`): with the light
unchanged, matching is strong (tens to hundreds of confident matches, sub-
percent pose error). Changing the light *direction* at all collapses matching
almost completely — confident matches fall to a handful (≈2–14), RANSAC
usually cannot reach its inlier floor, and the run-phase capture is **refused**
rather than logged. The failure is near-total, not gradual: this terrain's
appearance is shading-driven (relief lit by the directional light), not
texture-driven, so moving the light rewrites the local intensity patterns ORB
keys on everywhere at once.

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
| F | anywhere | FEATURE MATCH mode (needs waypoints) |
| W/A/S/D + arrows | moving modes | fly / look |
| Q / E | moving modes | altitude up / down |
| B | RECORD | drop a waypoint |
| B | TRACKERS / FEATURE MATCH | capture a timestep (true + computed pose) |
| N / M | TRACKERS / FEATURE MATCH | review next / previous timestep |
| G | FEATURE MATCH | (re)build the feature database under the current light |
| click | PICK | pick a 2D–3D correspondence |
| C / Z | PICK | solve pose / undo last pick |

## Headless checks

`make check` builds `tests/headless_checks.cpp` against the vision + loader
objects only and verifies on synthetic inputs: terrain normals against the
analytic plane normal, blob centroids (including the not-visible and
too-small cases), and both PnP solvers round-tripping a known camera — the
RANSAC variant with 3 of 11 correspondences corrupted. Exit code = number of
failed checks.
