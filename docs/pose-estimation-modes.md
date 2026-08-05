# Pose-estimation modes: Trackers (C) and Feature Matching (D)

Both modes answer the same question — *given only what the camera sees, where
is the camera?* — by building 2D–3D correspondences and solving
Perspective-n-Point. They differ only in **where the correspondences come
from**: Mode C plants known landmarks in the scene; Mode D uses what the
terrain itself looks like.

## Shared backbone: `PoseComparisonState`

`src/state/PoseComparisonState.{h,cpp}` is the common base. Both modes:

- fly the player camera freely (same controls as NAVIGATION);
- on **B**, take a *capture*: the camera's pose right now is recorded as
  ground truth, and the mode's `computePose` estimates that same pose from
  the rendered frame alone — both go into a `PoseLog`, computed may be empty;
- on **N / M**, step the review cursor back and forth through the captures,
  snapping the camera to that capture's true pose;
- draw the comparison identically: the global view shows both fly-through
  paths (true = blue with red/green waypoint dots, computed = orange), and
  the player view overlays the computed pose as a translucent orange ghost
  while the camera sits on the reviewed true pose. The closer the ghost
  aligns with the real terrain, the better that capture's estimate.

The one pure virtual is `computePose` — each mode's correspondence pipeline.

### What the console says

Every result is reported through one shared phrase (`describe` in
`PoseComparisonState.cpp`, over `poseError` in `src/core/PoseLog.h`), so a
capture, a review, and a row of the `Ctrl+B` table are directly comparable:

```
CAPTURE 3 of 3: position error 8.4 units (1.3% of terrain), heading off by 2.1 deg
REVIEW 2 of 3: position error 8.4 units (1.3% of terrain), heading off by 2.1 deg
        (the player view is back on this capture's true pose; the orange ghost
         is the pose PnP computed from it)
```

Two things are deliberate. The **percentage** is the scale the raw number lacks —
8 units is tight on a 633-unit DEM and hopeless on a 60-unit one, and the number
alone reads the same either way. The **heading** is there because a pose is two
things: PnP solves position and orientation together, and a contaminated
consensus can trade one against the other, so a camera in the right place looking
the wrong way would otherwise be reported as a success. The parenthetical after
the first `REVIEW` prints once per session — stepping the cursor otherwise just
changes a number, with the thing it explains off in the other pane.

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
prompts for the number of features per view (default 8). Two phases:

**Pre-phase (G) — interactive, by hand.** The build steps through each recorded
waypoint: it poses the player (right) view at that waypoint and runs ORB on it
(`detectSpreadFeatures`), keeping N strong keypoints **spread across the frame**.
Spreading matters because ORB's strongest responses cluster — several fire on one
corner a few pixels apart — and a clustered set is bad twice over: the dots are
indistinguishable on the map, and points crowded into one spot barely constrain a
pose however well they are placed.

They are presented **one at a time** — the active suggestion is a large red
screen-space marker in the player view — and the user **color-picks its matching
3D point on the global (left) map** (`pickVertex(globalView)` → `mesh.worldPos`,
the same color-pick Mode B uses). The map draws the aids described below to make
that placement findable. Press **X** to skip an unplaceable suggestion and **U**
to take back the last one, and use the global-map mouse controls (scroll = zoom,
middle-drag = pan, right-drag = orbit) to frame hard-to-see spots, including
behind mountains.

**The click is snapped onto the suggestion's viewing ray** before it is stored
(`snapToViewRay`, `src/core/Camera.h`). The camera pose here is a recorded
waypoint and the keypoint's pixel is exact, so the point being anchored *is* on
that ray; only the distance along it is unknown, and that is the one thing the
user is really being asked for. Whatever sideways offset the click has is aim
error, and dropping it moves the anchor strictly closer to the truth. This is
also the half that matters — error along the ray reprojects onto the same pixel
and costs the solve almost nothing, while the same error across it reprojects
tens of pixels away and gets a perfectly good correspondence voted out as an
outlier. (This is why the placed violet markers always sit exactly on their own
sight lines.)

Each placement stores `(descriptor, snapped 3D anchor)` in the `FeatureDb`. The
2D ORB position is *only* on-screen guidance and the ray it defines — it is never
stored or fed to PnP; the depth, which is what a single view cannot recover,
comes entirely from the user.

**When the build finishes, each placed point collects its appearance in the other
recorded views** (`addOtherViewAppearances`). A human places a point *once*, in
*one* view, so the hand-build leaves exactly one descriptor per point — one
appearance, from one angle. ORB is not viewpoint-invariant and this terrain's
appearance is shading rather than texture, so from anywhere else that single
appearance frequently fails to match at all. It is precisely why capturing while
sitting on a recorded waypoint worked and free flight between them did not.

Nothing about this is automatic anchoring. The recorded poses are known exactly
and the point is the user's own, so *where that point falls in every other
recorded view* is plain projection — no mesh is read, and no 3D is chosen by
machine. The same hand-placed point simply gains the descriptors of its other
appearances, which is what the removed automatic variant had for free.

It is self-limiting, which is what makes it safe: an appearance counts only if
there is a keypoint within a few pixels of the projection *and* its descriptor
still resembles the anchored one (`resemblesAnchoredPoint`). A point whose depth
was misjudged projects somewhere else in every other view, finds nothing that
looks like it, and gains nothing — a bad placement cannot poison the database.

So a `FeatureDb` row is one **appearance**, not one place: `anchors` repeats a
position once per appearance, and `places()` is what the user actually placed.

**Ctrl+S / Ctrl+O** write the database to `captures/featuredb_<terrain>.yml` and
read it back (`src/vision/FeatureDbIo.cpp`, `cv::FileStorage` YAML). The recorded
waypoints travel with it, since the anchors were placed from those views; a file
saved under a different terrain is refused. Placing thirty points by hand is
minutes of work, and without this every measurement would start by redoing it.
A file holding only one appearance per point — one saved before those were
collected — has them collected on load, so an old database is upgraded rather
than re-placed.

**Run-phase (B)** — fully automatic: capture the current view, detect ORB
features, and match **database → frame, cross-checked** (`matchFeaturesToDb`),
asking "where is each of my anchors in this view, and does that keypoint agree?".

The direction is load-bearing: asked the other way round, once per keypoint, a
20-anchor database can answer "yes" 140 times, claiming one anchor is in dozens
of places at once and handing PnP a set of contradictions to build a consensus
from. Querying the database caps the result at **one correspondence per anchor**;
cross-checking adds one per keypoint; and the per-place collapse (a place owns
several appearances) keeps it at **one claim per place**.

Cross-checking replaced **Lowe's ratio test**, which is the wrong tool for this
scene and was the direct cause of a total failure to localise. The ratio test
asks whether the best match beats the *second best in the same image*, so it only
passes where a feature is locally unique — and on a shading-driven terrain every
ridge shoulder looks like every other ridge shoulder. Measured on a 30-point
hand-built database it kept **1–4 of the 10–15 anchors that were genuinely in
frame**, which is far below what any pose needs. Mutual agreement asks something
this scene can answer, and outlier rejection is left to RANSAC, which is the
stage that belongs to it. A single absolute bar remains
(`kMaxDescriptorDistance`, 96 of 256 bits): past that, the "best" match is no
better than the nearest of a bag of random descriptors.

The surviving pairs go to `computeCameraPoseRansac`. Two settings there are sized
for hand-placed anchors rather than machine-precise ones: the inlier gate is
**40 px** (`kHandPlacedReprojErrorPx`) so a click that is tens of world units off
can still be recognised as correct, and the consensus floor is **a quarter of the
matches** (clamped to 6–25) instead of a fixed number that is a high bar at 20
anchors and trivial at 9 000. Scaled to the *matches* rather than the database,
deliberately: a frame only sees the anchors of the views near it, so any fraction
of the database total becomes unmeetable once the database spans more views than
one frame can contain.

**Ctrl+B** runs a capture at *every* recorded waypoint and prints the errors as a
table with the mean and the terrain's size — one keypress instead of flying the
path again, which is what makes "change one thing and measure" practical here.
It works in TRACKERS too.

### Reading the map in the run phase

The whole database is drawn on the global map while you fly, and the anchors the
**current view can actually use** are highlighted. With the view cone, that
answers the question a capture otherwise answers only after the fact: *is this a
good place to press B?*

"Can use" is `isInFrame` (`src/core/Camera.h`, the clip volume of the matrix the
renderer draws with) **and** unoccluded — the existing `raycastTerrain` marches
from the eye and stops short of the anchor itself, because ORB matches what was
drawn and a feature behind a ridge was not. (An anchor buried well *under* the
surface also reads as hidden. That is honest rather than a false negative: it
means the depth was misjudged when it was placed, and the run phase will not
match it well either.) `FeatureMatchState::tick` recomputes the split once per
frame and prints the count when it changes, at most twice a second.

Four kinds of thing share the map, so they share one rule: **grey is where the
database came from, violet is what the database knows, blue/red is truth measured
now, orange is what was computed.**

| On the global map | Look |
| --- | --- |
| pre-phase flight path + the views the anchors were placed from | dim grey path, small grey dots |
| database anchor **usable from this view** | large violet dot |
| database anchor not usable (out of frame, or behind a ridge) | small dim violet dot |
| captured true poses (`B`) | blue path, red dots — green on the one under review |
| computed poses | orange path + dots |

Violet, not the green it used to be: green already means "the camera is standing
on this waypoint" in `Renderer::drawWaypoints`, and it vanishes into a green
elevation ramp. `V` still toggles only the cone and sight lines — the anchors are
the mode's data, not an aid.

The console reports anchors in frame → matched → RANSAC inliers → error per
capture, so a poor result separates into a positioning problem and a matching
one:

```
FEATURES: 9 of 24 anchors in frame
FEATURES: 7 anchors matched (from 998 keypoints in the frame)
PnP (RANSAC): 6 of 7 correspondences are inliers
CAPTURE 3 of 3: position error 8.4 units (1.3% of terrain), heading off by 2.1 deg
```

## Reading the map: the view aids (V)

Both manual modes ask the same awkward thing: *find, on a map of the whole
terrain, the point you are looking at in the other pane.* The global view draws
three aids for it. **V** toggles them all; they are on by default.

**The view cone** (pale cyan, drawn in every mode) is the player camera's
frustum: apex at the eye, four edges out to where its center ray meets the
terrain, closed by a rectangle. It answers "which wedge of the map is the other
pane showing", turning a whole-terrain search into a search inside the cone.
Because it is not mode-specific it is drawn by `Renderer::renderGlobalView`
itself rather than by any `State` overlay — it is also useful while flying in
RECORD, to see what a waypoint will capture.

**The sight line** is the observation you are currently placing, drawn from the
eye in the same color as its marker in the player pane (white for PICK's pending
pick, red for FEATURE MATCH's active suggestion). Its 3D point lies **somewhere
along that line** — which collapses the search from a 2D area to a 1D one.

This is the one place the design deliberately stops short of being helpful: the
lines are **never intersected with the terrain**. A pixel fixes a direction, not
a point; choosing the depth along it *is* the manual correspondence both modes
exist to have the user supply (and what the brief means by "map features to 3D
using picking"). Drawing the hit would hand over the answer. Right-drag to orbit
the map is how you judge that depth — from a side-on angle it is obvious which
ridge the line grazes.

Mode D's snap does not cross that line. It moves the click *onto* the ray the
click was already aiming at, using only the camera pose and the keypoint pixel —
the depth along the ray, the part the terrain would have answered, is still
entirely the user's.

**Dim sight lines** mark observations already matched. In PICK they are a
self-check: a 3D marker that does not sit on its own line was mis-picked. In
FEATURE MATCH the snap puts every anchor on its line by construction, so what
they show instead is the *spread* of what has been placed — anchors bunched along
one line constrain a pose far less than the same number fanned across the view.
**U** undoes the last one (in FEATURE MATCH, within the current view), and **X**
cancels a half-finished pair in PICK. Before these existed a misclick was
permanent, and a single bad correspondence is enough to wreck a solve.

A note on Mode B: the cone reveals the seeded camera's *orientation*, which is
nominally part of what PnP is recovering. Its position is already on the map (the
green waypoint dot), and the exercise is building correspondences and watching
PnP converge, not guessing where the camera is — but press **V** for a clean run
if you want the mode fully blind.

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
| V | anywhere | toggle the global map's view aids (cone + sight lines) |
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
| B | TRACKERS / FEATURE MATCH | take a capture (true + computed pose) |
| Ctrl+B | TRACKERS / FEATURE MATCH | capture at every recorded view; print the error table |
| N / M | TRACKERS / FEATURE MATCH | review next / previous capture |
| G | FEATURE MATCH | start the manual database build (steps through each view) |
| left-click | FEATURE MATCH build | color-pick the active suggestion's 3D point on the global map |
| X | FEATURE MATCH build | skip the active (unplaceable) suggestion |
| U | FEATURE MATCH build | undo the last anchor placed in this view |
| Ctrl+S | FEATURE MATCH run-phase | save the database (+ waypoints) to `captures/` |
| Ctrl+O | FEATURE MATCH run-phase | load it back |
| click | PICK | pick a 2D–3D correspondence |
| X | PICK | cancel the pending 2D pick |
| U | PICK | undo the last completed correspondence |
| C | PICK | solve pose |

## Headless checks

`ctest --preset linux` (or `--preset windows`) builds and runs the checks in `tests/` against the
GL-free code (no window, no GPU) and verifies on synthetic inputs:
terrain normals against the analytic plane normal, blob centroids (including
the not-visible and too-small cases), the camera-control verbs, the color-pick
id encoding round trip, the split-screen viewport layout, the pose-review log
cursor, ORB feature suggestion (ordering and descriptor alignment), the
render↔vision camera-model agreement (projection and viewing rays), and both
PnP solvers round-tripping a known camera — the RANSAC variant with 3 of 11
correspondences corrupted. Exit code = number of failed checks.
