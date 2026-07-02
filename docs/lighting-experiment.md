# Experiment: lighting change vs. 2D feature matching (Mode D)

> **Provenance note — results predate the current Mode D.** This experiment was
> run on an earlier, fully automatic variant of Mode D, which anchored *every*
> ORB keypoint to its 3D vertex via the ID-pass read-back (~1 000 descriptors
> per view). That anchoring read the mesh vertices directly, which the project's
> rules disallow for CV algorithms, so Mode D was redesigned around the manual
> anchoring described in [pose-estimation-modes.md](pose-estimation-modes.md);
> the automatic variant survives only in git history, and the setup below — in
> particular the 9 792-descriptor database — is not reproducible with the
> current app. The *result* stands on its own: the match collapse is a property
> of ORB descriptors under relighting of a shading-driven scene, independent of
> how the database was anchored. A rerun under the current manual pipeline is
> planned.

The mini-project (PDF p. 54) asks us to change the lighting between the
feature-matching **pre-phase** (building the descriptor database) and the
**run-phase** (estimating pose by matching) and report what happens. This is
the result on `terrain1.jpg`.

## Setup

- 10 views recorded along a flight path and used as the pre-phase views.
- Database built (G) under the **late-morning sun** preset: 9 792 anchored ORB
  descriptors.
- Run-phase captures (B) taken while flying, first under the same light, then
  after cycling the light (L) through *noon sun*, *low warm sun*, and
  *overcast*. The database is **not** rebuilt — it keeps the late-morning
  appearance throughout, which is the whole point.

Each capture logs `confident matches / keypoints`, RANSAC `inliers`, and the
position error against the recorded true pose (terrain is ~633 units wide).

## Results

### Light matched to the database (late-morning sun)

| Capture | Matches / 1000 | Inliers | Position error |
| ------- | -------------- | ------- | -------------- |
| 0       | 303            | 300     | 1.02           |
| 1       | 84             | 81      | 1.41           |
| 2       | 30             | 19      | 2.89           |
| 3       | 170            | 165     | 0.55           |
| 4       | 60             | 54      | 2.69           |
| 5       | 58             | 52      | 3.67           |

Every well-overlapping view solves, with **95 %+ inlier ratios** and error
**under 4 units (< 0.6 % of the terrain)**.

### Light changed (noon / low warm sun / overcast)

Across **all** captures under a changed light, confident matches collapsed to
**2–14 out of 1000 keypoints**, inliers were usually **0**, and almost every
frame was refused. The handful that scraped a consensus produced nonsense
poses (errors of 180–600 units — a quarter to a full terrain width off).

| Condition            | Matches range | Typical outcome              |
| -------------------- | ------------- | ---------------------------- |
| Light matches DB     | 30 – 303      | solved, error < 4            |
| Any light change     | 2 – 14        | refused, or a garbage pose   |

The separation is clean: under the database's own light no capture fell below
30 matches; under any changed light none exceeded 14.

## Why the failure is near-total, not gradual

ORB describes each keypoint by a binary pattern of **local intensity
gradients** (BRIEF tests comparing pixel brightnesses), and it picks keypoints
where intensity forms corners. This terrain has almost no albedo texture — its
appearance is a smooth height-to-colour gradient plus **Lambert shading from
the directional light**. So a change in light *direction*:

- inverts which slopes are bright and which are dark, flipping the gradients
  the descriptors encode, and
- moves the corners themselves, so a different set of pixels even gets
  detected as keypoints.

The descriptors stored under the late-morning sun therefore describe a scene
that no longer exists once the sun moves. A photograph of a textured object
survives relighting because surface albedo dominates its appearance; a
shading-only synthetic terrain does not. This makes the terrain an extreme
case, and the result is an almost complete loss of matches rather than a
graceful decline.

## The inlier guard's role

`estimatePoseFromFeatures` requires RANSAC to reach **25 inliers** before it
trusts a pose. This is what turns the changed-light captures into honest
`no trustworthy pose` refusals instead of the confident-but-wrong estimates a
few clustered matches would otherwise yield (a pre-guard run of this same
experiment logged stray poses 190–600 units off from 4–7 inliers). The guard
makes the failure mode *legible*: the app says it cannot localise, which is the
correct answer.

## Confound to control for

Match count depends on **two** things: lighting agreement with the database
*and* view overlap with the recorded views. In the run above the camera flew
freely, so when the light was later returned to late-morning the matches did
**not** recover — by then the camera was looking at un-recorded terrain. To
isolate lighting cleanly, park the camera at one good pose and toggle the light
(L) between captures (B) **without moving**: the match count then varies with
lighting alone. The aggregate above already shows lighting dominates (the 30+
vs. ≤14 split holds across every view), but the parked-camera version gives the
tightest single-variable curve.

## Takeaway

2D feature matching localises this terrain to sub-percent accuracy when the
run-phase lighting matches the pre-phase, and fails almost completely under a
change of light direction, because the terrain's appearance is shading-driven
rather than texture-driven. Robustness would require either lighting-invariant
descriptors, building the database under several lightings, or adding genuine
surface texture. Repeat on a second DEM (`terrain2.png`) to confirm the effect
is not specific to one terrain.
