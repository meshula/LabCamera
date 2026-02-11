
# Introduction to Practical On-Set and Virtual Production Camera

This article explains how real camera measurements made on set map into optical models, and how those models map into virtual cameras used in OpenUSD and RenderMan pipelines. The goal is to provide reliable, first-order results for layout, matchmove, and rendering, bridging the gap between physical cinematography and virtual production.

There are two regimes of interest:

**Practical model** — what you can measure externally and assume safely
**Technical model** — what you can compute when a lens datasheet provides optical reference planes

This article will cover the concepts and mathematics of both, and will provide useful Python illustrations that can be immediately incorporated into tools and workflows.

---

## Real World Measurements

On set, only a few distances are directly observable without disassembling the lens:

* Sensor mark to subject distance (focus pull distance)
* Focal length and f-stop from the lens barrel
* Sensor or filmback dimensions
* Sensor mark to lens front distance (tape measure)
* Physical lens length

Focus is measured from the sensor plane because it is the only invariant, externally marked reference. Optical principal planes are not marked and cannot be measured relative to the lens barrel in any obvious way.

For parallax-free rotation (nodal pans), the operator will rotate the camera around the entrance pupil, having determined where that is by a simple parallax slide test. This point is not at the sensor nor the lens center.

These measurements are sufficient to build a thin-lens equivalent camera that is useful for most virtual production tasks.

---

## Introduction to Gaussian (Paraxial) Optics

First-order centered optical systems are modeled using Gaussian optics. A complex lens system is replaced by an equivalent thin lens defined by:

**Effective focal length**

Two principal planes ***H1*** and ***H2***

Object distance is measured from ***H1***.
Image distance is measured from ***H2***.

They satisfy the Gaussian imaging law:

1/f = 1/s + 1/s′

***H1*** and ***H2*** are non-physical reference planes that make the math work; they can't be measured relative to the equipment. Depending on lens design, they may lie inside the lens, between elements, or outside the housing.

If we collapse ***H1*** and ***H2*** to the same plane, we obtain the Gauss-Seidel thin-lens model, the standard practical approximation.

> **Note on Sign Convention**: To maintain consistency with the Gaussian imaging law, object distance (s) is measured as a positive value from the first principal plane (***H1***) toward the subject, while image distance (s′) is measured as a positive value from the second principal plane (***H2***​) toward the sensor.

---

## Real World to Gaussian Model

On set we measure the distance from the sensor to the subject, as the sensor is typically indicated on the camera body with a `Φ` mark. Gaussian optics is based on the distance from ***H1*** to the subject. This requires the principal plane offset, which is usually unknown without a datasheet.

Therefore two workflows exist.

### Practical workflow:

Assume a thin lens. Treat the measured focus distance as the object distance. Assume the principal plane exists near the lens center. This yields stable first-order behavior for FOV, focus, and DOF.

### Technical workflow:

Use datasheet values for:

* H1 offset
* H2 offset
* Entrance pupil offset

Convert measured distances into true Gaussian distances before computing focus and DOF. This removes bias from retrofocus and telephoto designs.

---

## Depth of Field and Circle of Confusion

Depth of field comes from the thin-lens model plus an acceptable blur diameter on the sensor, called the circle of confusion.

Given focal length ***f***, f-number ***N***, focus distance ***s***, and CoC ***c***:

The hyperfocal distance is:

H = f² / (N c) + f

Near and far DOF limits are:

Dnear = (H s) / (H + (s − f))
Dfar  = (H s) / (H − (s − f))  or infinity if s ≥ H

If near/far DOF limits are measured from a plate, these equations can be inverted to estimate effective f-number or effective CoC. That is useful in matchmove and lens report reconstruction.

---

### Anamorphic Lenses

While the paraxial models described above assume spherical elements, anamorphic lenses introduce **dual focal lengths** due to their cylindrical squeeze factor. In these systems:

* **Focal Length:** The effective focal length differs between the horizontal and vertical axes; for example, a 50mm anamorphic lens with a 2x squeeze behaves like a 25mm lens horizontally while remaining a 50mm lens vertically.
* **Circle of Confusion (CoC):** Because the image is stretched during de-squeeze, the acceptable CoC also becomes asymmetric. To maintain consistent depth-of-field logic in a virtual renderer, the horizontal CoC must be scaled by the inverse of the lens squeeze factor.
* **Virtual Mapping:** When emitting to USD or RenderMan, ensure the squeeze factor is explicitly defined in the `horizontalApertureOffset` or `pixelAspectRatio` to prevent distortion in the bokeh and projection.

---

## Entrance Pupil and Rotation Pivot

Parallax-free rotation does not occur about the sensor plane. It occurs about the entrance pupil (often called the nodal point on set).

When a datasheet is available, use the provided entrance pupil offset.

Without one, estimate from external measurements:

* Measure the distance from the sensor to the front of the lens
* Measure the lens length
* Assume the entrance pupil lies roughly 30–50% of lens length behind the front element

Virtual cameras should rotate about this pivot for matchmove and nodal pan accuracy.

---

## Introduction to GfCamera and Prman Camera

USD GfCamera is filmback + focal length based. Key parameters:

* Focal length
* Horizontal and vertical aperture
* Focus distance
* F-stop

Field of view is derived from focal length and aperture.

RenderMan cameras are projection driven. Key parameters:

* Field of view
* Screen window
* F-stop
* Focal distance

The FOV is primary; filmback is implicit via the screen window.

Both ultimately encode the same paraxial projection, but with different parameterizations.

---

## Mapping the Real World to the Virtual

The mapping pipeline is:

1. On-set measurements
2. Choose model
3. Solve Gaussian distances
4. Compute DOF
5. Create the renderer camera

### Practical path:

1. Use thin-lens solver
2. Use measured focus distance directly
3. Derive FOV from focal length and sensor
4. Estimate entrance pupil for pivot
5. Emit GfCamera and PrMan parameters

### Technical path:

1. Use datasheet H1/H2 offsets
2. Convert measured focus to true object distance
3. Compute DOF from Gaussian distance
4. Use datasheet entrance pupil
5. Emit cameras with corrected focus distance

Both paths produce renderer-usable cameras. The technical path reduces systematic error when lens metadata is available.

---

# Appendices


## Appendix A - Deriving the Principal Planes

Reconciling real world measurements and ideal mathematical models requires some attention. One cannot generally derive ***H1*** and ***H2*** locations from film-back focus distance, focal length, f-stop, and sensor size alone. You can derive image distance and magnification, but not the principal plane offsets, unless you also know the lens’s principal plane or nodal point offsets (which are lens design data).

On set, focus pullers measure the distance from the film plane (sensor mark `Φ`) to the focused subject distance, because:

* the film/sensor plane is externally marked
* it is invariant under focusing
* it is mechanically well-defined

This distance is **not** the Gaussian object distance s. Gaussian object distance is measured from ***H1***, not from the sensor plane.

So what you measure on set is:

D_measured = sensor → subject

But the Gaussian wants:

s = ***H1*** → subject
s′ = ***H2*** → sensor

Those differ by principal plane offsets.

Treating that distance as the nodal point for rotation purposes is operationally correct, and standard cinematography practice.

For camera rotation (pan/tilt to avoid parallax):

* You rotate about the **entrance pupil** (often loosely called the nodal point on set).
* That point is close to the front nodal point ***N1***.
* In air, nodal points and principal points are often near each other, but not guaranteed identical.

For parallax-free rotation, we use the entrance pupil and nodal point, not the sensor plane, nor the principal plane strictly speaking.

This is a different constraint than Gaussian distance measurement. So, given focal length, f-stop, sensor size — what can we derive?

From:

* focal length f
* sensor size
* focus distance (sensor → subject)
* f-stop N

we can derive:

* Field of view (from focal length + sensor size)
* Magnification (if object distance is known relative to principal plane)
* Image distance s′ (if object distance s is known relative to principal plane)
* Circle of confusion / DOF behavior (approx)

But we still need principal plane offsets to connect the measured distance to s and s′.

---

### A.1 Why H1 and H2 cannot be derived from that data alone

Gaussian relations give:

1/f = 1/s + 1/s′

But s and s′ are measured from H1 and H2.

On set measurement gives:

D = sensor → subject

Let Δ2 = distance from H2 to sensor
Let Δ1 = distance from H1 to some mechanical reference (unknown externally)

Then:

s = D − Δ1 − Δ2   (depending on sign convention)

Those offsets are properties of the lens optical design. They are not determined by:

* focal length
* f-stop
* sensor size

Two lenses can share the same focal length and f-stop but have very different principal plane locations (retrofocus wide angle vs telephoto are classic counterexamples).

Therefore: the system is underdetermined without lens design metadata.

---

### A.2 What lens makers sometimes provide

High-end lens data sheets sometimes include:

* entrance pupil position vs focus distance
* principal plane offsets
* nodal point locations
* focus breathing curves

With that data, one can reconstruct H1 and H2 positions in a Gaussian equivalent model; without that data it's not possible.

In many practical cine / VFX pipelines, people assume:

* thin lens model
* principal planes coincide
* that plane is near the aperture stop

Under that approximation:

* measured focus distance ≈ s + s′
* one can back-solve a thin-lens layout
* errors are usually small for mid-focus distances

That’s a workable approximation, but it is an assumption.
---

Bottom line

On set you measure sensor → subject because it is the only invariant external datum — correct.

For parallax rotation you use the entrance pupil — correct.

From focal length, f-stop, and sensor size you can derive projection and DOF behavior — correct.

But you cannot uniquely derive H1 and H2 locations from those alone. You need principal plane or nodal offset data from the lens design, or you must assume a thin-lens surrogate model where they coincide.


---

## Appendix - On Set Calculators

Function A — Practical On-Set Thin-Lens Calculator

Assumptions:

* Thin lens model (H1 = H2).
* Marked focus distance is measured from sensor plane.
* That distance ≈ object distance s.
* Principal plane located at lens entrance pupil ≈ lens center (user can measure).
* Good for mid–far focus, weaker for macro and telephoto retrofocus extremes.

Ask the user to measure (non-invasive):

* sensor_to_subject_mm  (focus pull distance)
* focal_length_mm
* f_number
* sensor_width_mm, sensor_height_mm
* sensor_to_lens_front_mm (tape measure)
* lens_length_mm (physical barrel length)

We estimate principal plane near barrel midpoint.

```python
import math

def onset_thin_lens_calculator(
    focal_length_mm,
    f_number,
    sensor_width_mm,
    sensor_height_mm,
    sensor_to_subject_mm,
    sensor_to_lens_front_mm,
    lens_length_mm,
):
    # Thin lens assumption: s ≈ measured distance
    s = sensor_to_subject_mm
    f = focal_length_mm

    # Image distance from thin lens equation
    # 1/f = 1/s + 1/s'
    s_prime = 1.0 / (1.0/f - 1.0/s)

    # Estimate principal plane location (thin lens) near lens center
    H_offset_from_sensor = sensor_to_lens_front_mm - lens_length_mm * 0.5

    # Derived quantities
    magnification = s_prime / s
    aperture_diameter_mm = focal_length_mm / f_number

    hfov = 2 * math.degrees(math.atan(sensor_width_mm / (2*f)))
    vfov = 2 * math.degrees(math.atan(sensor_height_mm / (2*f)))

    return {
        "model": "thin_lens_onset",
        "assumptions": [
            "principal planes coincident",
            "focus distance treated as object distance",
            "principal plane near lens center"
        ],
        "object_distance_s_mm": s,
        "image_distance_s_prime_mm": s_prime,
        "estimated_principal_plane_from_sensor_mm": H_offset_from_sensor,
        "magnification": magnification,
        "aperture_diameter_mm": aperture_diameter_mm,
        "hfov_deg": hfov,
        "vfov_deg": vfov,
    }
```

---

Extra on-set measurements that improve Function A (no disassembly)

You can ask crew to:

* Measure sensor mark → lens front
* Measure lens physical length
* Find entrance pupil by parallax test (slide camera sideways vs near/far object)
* Measure subject distance at two focus marks → estimate focus breathing

Those refine the principal plane estimate.

---

Function B — Datasheet Gaussian Calculator

Assumes lens datasheet provides:

* focal_length_mm (effective focal length)
* H1_offset_mm  (from a mechanical reference plane, usually mount flange)
* H2_offset_mm  (from same reference plane)
* sensor_offset_mm (sensor plane from same reference — flange focal distance)
* marked_focus_distance_mm (sensor → subject, on-set reading)

This lets us compute true Gaussian distances.

Sign convention here:

Positive distances toward subject from the reference plane.

```python
import math

def datasheet_gaussian_calculator(
    focal_length_mm,
    H1_offset_mm,
    H2_offset_mm,
    sensor_offset_mm,
    marked_focus_distance_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
):
    f = focal_length_mm

    # Convert measured focus distance (sensor→subject)
    # to object distance from H1
    # subject_pos = sensor_pos + marked_distance
    subject_pos = sensor_offset_mm + marked_focus_distance_mm
    s = subject_pos - H1_offset_mm

    # Gaussian thin lens equation
    s_prime = 1.0 / (1.0/f - 1.0/s)

    # Predicted sensor position from H2
    predicted_sensor_pos = H2_offset_mm + s_prime

    focus_error_mm = predicted_sensor_pos - sensor_offset_mm

    magnification = s_prime / s
    aperture_diameter_mm = focal_length_mm / f_number

    hfov = 2 * math.degrees(math.atan(sensor_width_mm / (2*f)))
    vfov = 2 * math.degrees(math.atan(sensor_height_mm / (2*f)))

    return {
        "model": "datasheet_gaussian",
        "object_distance_from_H1_mm": s,
        "image_distance_from_H2_mm": s_prime,
        "predicted_sensor_position_mm": predicted_sensor_pos,
        "sensor_focus_error_mm": focus_error_mm,
        "magnification": magnification,
        "aperture_diameter_mm": aperture_diameter_mm,
        "hfov_deg": hfov,
        "vfov_deg": vfov,
        "principal_planes": {
            "H1_offset_mm": H1_offset_mm,
            "H2_offset_mm": H2_offset_mm,
        }
    }
```

Practical distinction

Function A answers:
“What camera should I simulate for blocking, previs, matchmove bootstrap?”

Function B answers:
“Given real optical reference planes, where is the Gaussian equivalent system and what is the exact conjugate geometry?”

---

## Appendix - Constructing Virtual Cameras from Optics

Next, we wire the optics math to camera construction, in an implementation-oriented and renderer-facing manner.

Assumptions:

* We don’t instantiate real pxr objects here (environment unknown).
* We return parameter dictionaries you can feed into:

  * UsdGeomCamera / GfCamera setters
  * RenderMan camera/projection settings
* The four builders call the two calculators we defined earlier.

All distances in millimeters unless noted.

---

Helper calculators (compact forms)

```python
import math

def calc_thin_lens(f_mm, focus_dist_mm):
    s = focus_dist_mm
    s_prime = 1.0 / (1.0/f_mm - 1.0/s)
    return s, s_prime


def calc_datasheet_gaussian(f_mm, H1_mm, H2_mm, sensor_mm, marked_focus_mm):
    subject_pos = sensor_mm + marked_focus_mm
    s = subject_pos - H1_mm
    s_prime = 1.0 / (1.0/f_mm - 1.0/s)
    sensor_pred = H2_mm + s_prime
    return s, s_prime, sensor_pred
```

---

GfCamera builders

GfCamera cares about:

* projection
* focalLength
* horizontalAperture / verticalAperture
* focusDistance
* fStop
* clippingRange

USD uses tenths of a millimeter for apertures by convention.

Practical = thin-lens, principal planes collapsed.
Technical = datasheet principal planes respected for focus distance.

---

GfCamera — practical on-set

```python
def build_gfcamera_practical(
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
    marked_focus_distance_mm,
    clip_near_mm=100.0,
    clip_far_mm=1e7,
):
    s, s_prime = calc_thin_lens(focal_length_mm, marked_focus_distance_mm)

    return {
        "projection": "perspective",
        "focalLength": focal_length_mm,
        "horizontalAperture": sensor_width_mm * 10.0,
        "verticalAperture": sensor_height_mm * 10.0,
        "focusDistance": marked_focus_distance_mm,
        "fStop": f_number,
        "clippingRange": (clip_near_mm, clip_far_mm),
        "notes": "thin-lens practical onset model"
    }
```

---

GfCamera — technical (datasheet)

Here we convert measured focus to true Gaussian object distance from H1 and use that as focusDistance.

```python
def build_gfcamera_technical(
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
    sensor_offset_mm,
    H1_offset_mm,
    H2_offset_mm,
    marked_focus_distance_mm,
    clip_near_mm=100.0,
    clip_far_mm=1e7,
):
    s, s_prime, sensor_pred = calc_datasheet_gaussian(
        focal_length_mm,
        H1_offset_mm,
        H2_offset_mm,
        sensor_offset_mm,
        marked_focus_distance_mm
    )

    return {
        "projection": "perspective",
        "focalLength": focal_length_mm,
        "horizontalAperture": sensor_width_mm * 10.0,
        "verticalAperture": sensor_height_mm * 10.0,
        "focusDistance": s,   # true object distance from H1
        "fStop": f_number,
        "clippingRange": (clip_near_mm, clip_far_mm),
        "notes": "datasheet gaussian model"
    }
```

---

RenderMan camera builders

RenderMan cameras are FOV + screenwindow driven. We convert:

FOV = 2 * atan(sensor_dim / (2f))

We output:

* projection = perspective
* fov (horizontal, typical)
* screenwindow
* fstop
* focaldistance
* near/far

Screenwindow is symmetric unless lens shift is provided.

---

RenderMan — practical on-set

```python
def build_prman_camera_practical(
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
    marked_focus_distance_mm,
    clip_near_mm=100.0,
    clip_far_mm=1e7,
):
    hfov = 2 * math.degrees(
        math.atan(sensor_width_mm / (2*focal_length_mm))
    )

    aspect = sensor_height_mm / sensor_width_mm
    half_w = 1.0
    half_h = aspect

    return {
        "projection": "perspective",
        "fov": hfov,
        "screenwindow": (-half_w, half_w, -half_h, half_h),
        "fstop": f_number,
        "focaldistance": marked_focus_distance_mm,
        "clip": (clip_near_mm, clip_far_mm),
        "notes": "thin-lens practical onset model"
    }
```

---

RenderMan — technical (datasheet)

Use Gaussian object distance s (from H1) as focaldistance.

```python
def build_prman_camera_technical(
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
    sensor_offset_mm,
    H1_offset_mm,
    H2_offset_mm,
    marked_focus_distance_mm,
    clip_near_mm=100.0,
    clip_far_mm=1e7,
):
    s, s_prime, sensor_pred = calc_datasheet_gaussian(
        focal_length_mm,
        H1_offset_mm,
        H2_offset_mm,
        sensor_offset_mm,
        marked_focus_distance_mm
    )

    hfov = 2 * math.degrees(
        math.atan(sensor_width_mm / (2*focal_length_mm))
    )

    aspect = sensor_height_mm / sensor_width_mm
    half_w = 1.0
    half_h = aspect

    return {
        "projection": "perspective",
        "fov": hfov,
        "screenwindow": (-half_w, half_w, -half_h, half_h),
        "fstop": f_number,
        "focaldistance": s,
        "clip": (clip_near_mm, clip_far_mm),
        "notes": "datasheet gaussian model"
    }
```

---

Practical variants collapse principal planes and treat marked focus as object distance. That corresponds to the Gaussian thin-lens reduction where H1 = H2.

Technical variants respect a separated H1/H2 and convert sensor-mark focus distance into true Gaussian distances before emitting camera parameters.

---

# Appendix - Circle of Confusion and DOF Ranges

We extend the model in a renderer-useful way and keep the optics consistent with the two regimes we defined:

Practical model → thin-lens, principal planes coincident
Technical model → datasheet Gaussian, H1/H2 respected

We’ll add:

* circle of confusion (CoC) limit (input or derived)
* hyperfocal distance
* near DOF limit
* far DOF limit
* total DOF span

All formulas are first-order thin-lens DOF relations. Even in the technical model (separated principal planes), DOF math is still thin-lens based — we just feed it the correct Gaussian object distance s.

---

Core DOF math (shared)

Conventions:

f = focal length (mm)
N = f-number
c = acceptable circle of confusion on sensor (mm)
s = object distance from principal plane H1 (mm)

Hyperfocal:

H = f² / (N c) + f

Near / far limits:

D_near = (H s) / (H + (s − f))
D_far  = (H s) / (H − (s − f))   if s < H, else ∞

Implementation:

```python
import math

def coc_from_sensor(sensor_width_mm, sensor_height_mm, divisor=1500.0):
    # common cine/photography heuristic
    diag = math.hypot(sensor_width_mm, sensor_height_mm)
    return diag / divisor


def dof_from_object_distance(f_mm, f_number, coc_mm, s_mm):
    f = f_mm
    N = f_number
    c = coc_mm
    s = s_mm

    H = (f*f) / (N*c) + f

    near = (H * s) / (H + (s - f))

    if s >= H:
        far = math.inf
    else:
        far = (H * s) / (H - (s - f))

    total = (far - near) if math.isfinite(far) else math.inf

    return {
        "hyperfocal_mm": H,
        "near_mm": near,
        "far_mm": far,
        "total_mm": total,
        "coc_mm": c,
    }
```

---

Tie DOF to Practical (thin-lens) camera

Here object distance is just the marked focus distance (our earlier assumption).

We also return the CoC used so render and layout agree.

```python
def onset_practical_dof(
    focal_length_mm,
    f_number,
    sensor_width_mm,
    sensor_height_mm,
    marked_focus_distance_mm,
    coc_mm=None,
):
    if coc_mm is None:
        coc_mm = coc_from_sensor(sensor_width_mm, sensor_height_mm)

    s = marked_focus_distance_mm

    dof = dof_from_object_distance(
        focal_length_mm,
        f_number,
        coc_mm,
        s
    )

    dof["model"] = "practical_thin_lens"
    return dof
```

---

Tie DOF to Technical (datasheet Gaussian) camera

Here we first compute true Gaussian object distance from H1, then feed that into DOF math.

```python
def onset_technical_dof(
    focal_length_mm,
    f_number,
    sensor_width_mm,
    sensor_height_mm,
    sensor_offset_mm,
    H1_offset_mm,
    H2_offset_mm,
    marked_focus_distance_mm,
    coc_mm=None,
):
    if coc_mm is None:
        coc_mm = coc_from_sensor(sensor_width_mm, sensor_height_mm)

    s, s_prime, sensor_pred = calc_datasheet_gaussian(
        focal_length_mm,
        H1_offset_mm,
        H2_offset_mm,
        sensor_offset_mm,
        marked_focus_distance_mm
    )

    dof = dof_from_object_distance(
        focal_length_mm,
        f_number,
        coc_mm,
        s
    )

    dof["model"] = "technical_gaussian"
    dof["object_distance_from_H1_mm"] = s
    return dof
```

---

Augment GfCamera builders with DOF + CoC

We just merge results into the returned dicts.

Example — practical GfCamera:

```python
def build_gfcamera_practical_with_dof(
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    f_number,
    marked_focus_distance_mm,
):
    cam = build_gfcamera_practical(
        focal_length_mm,
        sensor_width_mm,
        sensor_height_mm,
        f_number,
        marked_focus_distance_mm,
    )

    dof = onset_practical_dof(
        focal_length_mm,
        f_number,
        sensor_width_mm,
        sensor_height_mm,
        marked_focus_distance_mm,
    )

    cam["dof"] = dof
    return cam
```

Technical variant is analogous — call `onset_technical_dof` and attach.

Same pattern applies to the two PrMan builders.

---

Renderer interpretation notes (important)

USD / GfCamera:

* CoC is not stored directly in schema — but you should record it in metadata if you want deterministic DOF reproduction across delegates.
* focusDistance + fStop + focalLength are what Hydra delegates use for lens blur.
* Different delegates assume different CoC criteria internally — your stored CoC makes validation possible.

RenderMan:

* DOF is driven by fstop + focaldistance + focalLength/FOV.
* CoC is emergent from sampling, not an explicit knob.
* Your computed near/far DOF limits are validation targets, not inputs.

---

## Appendix - Inverting Matchmove and Lens Forensics

Next, we add the inverse layer used in matchmove and lens forensics: solve for effective f-number or acceptable CoC from measured DOF brackets. This is standard first-order optics inversion and fits cleanly on top of the paraxial model.

We proceed from the same thin-lens DOF equations. Start from hyperfocal:

H = f² / (N c) + f

Near/far limits for focus distance s:

D_near = (H s) / (H + (s − f))
D_far  = (H s) / (H − (s − f))

If you measure near and far acceptable focus distances on set (or from plate analysis), you can solve for H first, then recover either N or c.

From the near equation we can isolate H directly:

H = D_near (s − f) / (s − D_near)

That avoids nonlinear solving and is numerically stable if near ≠ s.

Once H is known:

N = f² / ( c (H − f) )
c = f² / ( N (H − f) )

That gives us two inverse solvers.

All distances in mm.

Core inversion helpers:

```python
import math

def hyperfocal_from_near(f_mm, focus_dist_mm, near_mm):
    f = f_mm
    s = focus_dist_mm
    Dn = near_mm
    return (Dn * (s - f)) / (s - Dn)
```

Estimate f-number from measured DOF and assumed CoC:

```python
def estimate_fnumber_from_dof(
    focal_length_mm,
    focus_distance_mm,
    near_mm,
    coc_mm,
):
    H = hyperfocal_from_near(
        focal_length_mm,
        focus_distance_mm,
        near_mm
    )

    f = focal_length_mm
    N = (f*f) / (coc_mm * (H - f))

    return {
        "estimated_f_number": N,
        "hyperfocal_mm": H,
        "inputs": {
            "f_mm": f,
            "focus_mm": focus_distance_mm,
            "near_mm": near_mm,
            "coc_mm": coc_mm,
        }
    }
```

Estimate acceptable CoC from measured DOF and known f-number:

```python
def estimate_coc_from_dof(
    focal_length_mm,
    focus_distance_mm,
    near_mm,
    f_number,
):
    H = hyperfocal_from_near(
        focal_length_mm,
        focus_distance_mm,
        near_mm
    )

    f = focal_length_mm
    N = f_number

    coc = (f*f) / (N * (H - f))

    return {
        "estimated_coc_mm": coc,
        "hyperfocal_mm": H,
        "inputs": {
            "f_mm": f,
            "focus_mm": focus_distance_mm,
            "near_mm": near_mm,
            "f_number": N,
        }
    }
```

Optional consistency check using far distance if available. This flags bad measurements or strong non-thin-lens behavior (focus breathing, pupil shift, internal focusing).

```python
def far_from_hyperfocal(f_mm, s_mm, H_mm):
    if s_mm >= H_mm:
        return math.inf
    return (H_mm * s_mm) / (H_mm - (s_mm - f_mm))


def dof_consistency_error(
    focal_length_mm,
    focus_distance_mm,
    near_mm,
    far_measured_mm,
):
    H = hyperfocal_from_near(
        focal_length_mm,
        focus_distance_mm,
        near_mm
    )

    far_pred = far_from_hyperfocal(
        focal_length_mm,
        focus_distance_mm,
        H
    )

    if math.isinf(far_pred) and math.isinf(far_measured_mm):
        err = 0.0
    else:
        err = far_pred - far_measured_mm

    return {
        "hyperfocal_mm": H,
        "far_predicted_mm": far_pred,
        "far_measured_mm": far_measured_mm,
        "far_error_mm": err,
    }
```

How this ties back to our two camera models.

Practical camera path:

Use marked focus distance directly as s. Feed that into the inverse DOF solver. This yields an effective f-number or CoC consistent with the thin-lens onset model. Good for quick lens reports and matchmove bootstrap.

Technical (datasheet) path:

First convert marked focus distance to true Gaussian object distance from H1 using the datasheet calculator:

s, s′, _ = calc_datasheet_gaussian(...)

Then pass that s into the inverse DOF solver. That removes principal-plane bias and gives a physically cleaner estimate.

Operationally useful workflows:

You know f, focus mark, near/far acceptable focus from plate → solve effective f-number → compare to lens ring (detect T-stop vs F-stop drift, filtration, diffusion).

You know f, focus mark, f-number → solve effective CoC → choose renderer CoC threshold so CG blur matches plate blur.

You know near/far and f-number but not exact focus mark → solve H from near, then invert near equation for s.

---

## Appendix - Single Lens Report Solver

Next, we may construct a single **lens report solver** that ties everything together and emits a structured payload suitable for pipeline ingestion.

Design goals:

* Standalone; no renderer imports, just parameter dictionaries.
* Works in both modes: practical (thin-lens) and technical (datasheet).
* Accepts either forward inputs (f-stop, CoC) or inverse inputs (measured DOF).
* Emits:

  * normalized optical solution
  * DOF + hyperfocal
  * inferred parameters (if solved)
  * GfCamera block
  * RenderMan block

All distances in mm.

---

Core optics helpers

```python
import math

INF = float("inf")

def calc_thin_lens(f_mm, s_mm):
    return s_mm, 1.0 / (1.0/f_mm - 1.0/s_mm)


def calc_datasheet_gaussian(f_mm, H1_mm, H2_mm, sensor_mm, marked_focus_mm):
    subject_pos = sensor_mm + marked_focus_mm
    s = subject_pos - H1_mm
    s_prime = 1.0 / (1.0/f_mm - 1.0/s)
    sensor_pred = H2_mm + s_prime
    return s, s_prime, sensor_pred


def coc_from_sensor(w_mm, h_mm, divisor=1500.0):
    return math.hypot(w_mm, h_mm) / divisor


def hyperfocal_from_params(f_mm, N, coc_mm):
    return (f_mm*f_mm)/(N*coc_mm) + f_mm


def hyperfocal_from_near(f_mm, s_mm, near_mm):
    return (near_mm * (s_mm - f_mm)) / (s_mm - near_mm)


def dof_from_H(f_mm, s_mm, H_mm):
    near = (H_mm * s_mm) / (H_mm + (s_mm - f_mm))
    if s_mm >= H_mm:
        far = INF
    else:
        far = (H_mm * s_mm) / (H_mm - (s_mm - f_mm))
    total = far - near if math.isfinite(far) else INF
    return near, far, total
```

---

Camera parameter builders

```python
def gf_params(f_mm, sensor_w, sensor_h, fstop, focus_mm):
    return {
        "projection": "perspective",
        "focalLength": f_mm,
        "horizontalAperture": sensor_w * 10.0,
        "verticalAperture": sensor_h * 10.0,
        "focusDistance": focus_mm,
        "fStop": fstop,
    }


def prman_params(f_mm, sensor_w, sensor_h, fstop, focus_mm):
    hfov = 2 * math.degrees(math.atan(sensor_w/(2*f_mm)))
    aspect = sensor_h / sensor_w
    return {
        "projection": "perspective",
        "fov": hfov,
        "screenwindow": (-1, 1, -aspect, aspect),
        "fstop": fstop,
        "focaldistance": focus_mm,
    }
```

---

Main lens report solver

Modes:

model="practical" → thin lens
model="technical" → needs H1/H2/sensor offsets

Parameter solving priority:

1. If near DOF provided → solve hyperfocal → solve missing (fstop or CoC).
2. Else if fstop + CoC known → forward DOF.
3. Else CoC defaults from sensor diagonal.

```python
def lens_report_solver(
    model,                      # "practical" | "technical"
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
    marked_focus_distance_mm,

    # optional forward inputs
    f_number=None,
    coc_mm=None,

    # optional inverse inputs
    near_dof_mm=None,
    far_dof_mm=None,

    # technical model only
    sensor_offset_mm=None,
    H1_offset_mm=None,
    H2_offset_mm=None,
):
    f = focal_length_mm

    # --- object distance s ---
    if model == "practical":
        s = marked_focus_distance_mm
        s_prime = None

    elif model == "technical":
        if None in (sensor_offset_mm, H1_offset_mm, H2_offset_mm):
            raise ValueError("technical model requires sensor_offset_mm, H1_offset_mm, H2_offset_mm")
        s, s_prime, sensor_pred = calc_datasheet_gaussian(
            f, H1_offset_mm, H2_offset_mm, sensor_offset_mm, marked_focus_distance_mm
        )
    else:
        raise ValueError("model must be practical or technical")

    # --- CoC default ---
    if coc_mm is None:
        coc_mm = coc_from_sensor(sensor_width_mm, sensor_height_mm)

    inferred = {}

    # --- inverse solve if DOF bracket given ---
    if near_dof_mm is not None:
        H = hyperfocal_from_near(f, s, near_dof_mm)

        if f_number is None and coc_mm is not None:
            f_number = (f*f)/(coc_mm*(H - f))
            inferred["f_number"] = f_number

        elif coc_mm is None and f_number is not None:
            coc_mm = (f*f)/(f_number*(H - f))
            inferred["coc_mm"] = coc_mm

    # --- need f_number by now ---
    if f_number is None:
        raise ValueError("Need f_number or near_dof_mm to solve it")

    # --- forward DOF ---
    H = hyperfocal_from_params(f, f_number, coc_mm)
    near, far, total = dof_from_H(f, s, H)

    # --- consistency check if far measured ---
    consistency = None
    if far_dof_mm is not None and math.isfinite(far):
        consistency = far - far_dof_mm

    # --- camera blocks ---
    gf = gf_params(f, sensor_width_mm, sensor_height_mm, f_number, s)
    pr = prman_params(f, sensor_width_mm, sensor_height_mm, f_number, s)

    return {
        "model": model,
        "inputs": {
            "f_mm": f,
            "sensor_w_mm": sensor_width_mm,
            "sensor_h_mm": sensor_height_mm,
            "marked_focus_mm": marked_focus_distance_mm,
        },
        "object_distance_mm": s,
        "image_distance_mm": s_prime,
        "f_number": f_number,
        "coc_mm": coc_mm,
        "hyperfocal_mm": H,
        "dof": {
            "near_mm": near,
            "far_mm": far,
            "total_mm": total,
        },
        "consistency_far_error_mm": consistency,
        "inferred": inferred,
        "gfcamera": gf,
        "prman_camera": pr,
    }
```

---

What this gives you in practice

On-set quick pass:

You know focal length, focus mark, T-stop, sensor → get GfCamera + PrMan + DOF immediately.

Matchmove forensic pass:

You measure near DOF limit from plate, know focal length + focus mark → solver estimates effective f-number or CoC and emits renderer-ready cameras consistent with observed blur.

Lens datasheet pass:

Provide H1/H2 offsets → solver switches to true Gaussian distances and removes principal-plane bias from focus and DOF.

---

## Appendix — Entrance pupil and nodal rotation pivot extension

Operational facts we rely on:

On set, the parallax-free rotation point is the entrance pupil (often loosely called the nodal point). Lens datasheets sometimes provide entrance pupil offset vs focus distance. If not, we estimate it from external measurements.

We support two modes again.

Practical pivot estimate:

User can measure without disassembly:

* sensor → lens front distance
* physical lens length

We approximate entrance pupil near a fraction of lens length behind the front element. Empirically, 0.3–0.5 works for many lenses; we expose it as a parameter.

Technical pivot estimate:

If datasheet gives entrance pupil offset from mount flange (or another reference), we use it directly.

Add these helpers.

```python
def estimate_entrance_pupil_practical(
    sensor_to_lens_front_mm,
    lens_length_mm,
    pupil_fraction_from_front=0.4,
):
    # entrance pupil measured from sensor plane
    return sensor_to_lens_front_mm - pupil_fraction_from_front * lens_length_mm


def entrance_pupil_technical(
    entrance_pupil_offset_mm_from_sensor
):
    return entrance_pupil_offset_mm_from_sensor


def recommended_rotation_pivot(camera_origin_world, view_dir_world, pupil_offset_mm):
    # pivot = camera origin shifted forward along view direction
    return (
        camera_origin_world[0] + view_dir_world[0] * pupil_offset_mm,
        camera_origin_world[1] + view_dir_world[1] * pupil_offset_mm,
        camera_origin_world[2] + view_dir_world[2] * pupil_offset_mm,
    )
```

Now extend the lens_report_solver to emit pivot data.

Add optional inputs:

* sensor_to_lens_front_mm
* lens_length_mm
* entrance_pupil_offset_mm   (datasheet)

Add block inside solver:

```python
    # --- entrance pupil / pivot ---
    pivot = None

    if model == "technical" and entrance_pupil_offset_mm is not None:
        pupil_mm = entrance_pupil_technical(entrance_pupil_offset_mm)

    elif model == "practical" and \
         sensor_to_lens_front_mm is not None and \
         lens_length_mm is not None:
        pupil_mm = estimate_entrance_pupil_practical(
            sensor_to_lens_front_mm,
            lens_length_mm
        )
    else:
        pupil_mm = None

    report["entrance_pupil_from_sensor_mm"] = pupil_mm
    report["rotation_pivot_note"] = (
        "Translate camera along view axis by this distance for parallax-free rotation"
        if pupil_mm is not None else
        "No pupil estimate available"
    )
```

For DCCs and matchmove tools, you shift the camera transform forward by this offset and rotate about that pivot instead of the sensor origin.

That closes the loop from optics to rigging behavior.

---

## Appendix - Python Library

```python
#!/usr/bin/env python3

"""
lens_report_camera

A small practical/technical optics helper module for virtual production and
camera matching workflows. It converts on-set measurements into thin-lens or
principal-plane–corrected optical quantities and emits camera parameters
suitable for USD GfCamera and RenderMan-style cameras.

Design goals:
- First-order (Gaussian) optics only
- Explicit practical vs technical modes
- Depth-of-field and circle-of-confusion support
- Entrance pupil / rotation pivot estimation
- Simple CLI entry points

Units: millimeters unless otherwise noted.
"""

import math
import argparse
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any


# -----------------------------------------------------------------------------
# Core optics
# -----------------------------------------------------------------------------


def thin_lens_image_distance(f_mm: float, object_distance_mm: float) -> float:
    if object_distance_mm <= f_mm:
        raise ValueError("Object distance must be greater than focal length")
    return 1.0 / (1.0 / f_mm - 1.0 / object_distance_mm)


def effective_f_number(f_mm: float, aperture_diameter_mm: float) -> float:
    return f_mm / aperture_diameter_mm


def hyperfocal_distance(f_mm: float, f_number: float, coc_mm: float) -> float:
    return (f_mm * f_mm) / (f_number * coc_mm) + f_mm


def dof_limits(f_mm: float, f_number: float, coc_mm: float, focus_distance_mm: float):
    H = hyperfocal_distance(f_mm, f_number, coc_mm)
    s = focus_distance_mm

    d_near = (H * s) / (H + (s - f_mm))

    if s >= H:
        d_far = math.inf
    else:
        d_far = (H * s) / (H - (s - f_mm))

    return d_near, d_far, H


# -----------------------------------------------------------------------------
# Field of view and camera mappings
# -----------------------------------------------------------------------------


def fov_from_focal_and_aperture(
    f_mm: float, 
    aperture_mm: float, 
    squeeze: float = 1.0
) -> float:
    # Anamorphic lenses increase the horizontal field of view 
    # by the squeeze factor
    effective_f = f_mm / squeeze
    return math.degrees(2.0 * math.atan((aperture_mm * 0.5) / effective_f))

def focal_from_fov_and_aperture(fov_deg: float, aperture_mm: float) -> float:
    return (aperture_mm * 0.5) / math.tan(math.radians(fov_deg) * 0.5)


def dof_limits_anamorphic(
    f_mm: float, 
    f_number: float, 
    coc_mm: float, 
    focus_distance_mm: float, 
    squeeze: float = 1.0
):
    H = hyperfocal_distance(f_mm, f_number, coc_mm)
    s = focus_distance_mm

    d_near = (H * s) / (H + (s - f_mm))
    
    if s >= H:
        d_far = math.inf
    else:
        d_far = (H * s) / (H - (s - f_mm))

    # Note: Depth of field is physically determined by the vertical bokeh 
    # (spherical equivalent), but horizontal blur is perceived differently 
    # after de-squeeze.
    return d_near, d_far, H


def gf_camera_from_physical(
    focal_length_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: float,
    focus_distance_mm: float,
    f_stop: float,
) -> Dict[str, Any]:
    return {
        "model": "GfCamera",
        "focalLength_mm": focal_length_mm,
        "horizontalAperture_mm": sensor_width_mm,
        "verticalAperture_mm": sensor_height_mm,
        "focusDistance_mm": focus_distance_mm,
        "fStop": f_stop,
        "horizontalFOV_deg": fov_from_focal_and_aperture(
            focal_length_mm, sensor_width_mm
        ),
        "verticalFOV_deg": fov_from_focal_and_aperture(
            focal_length_mm, sensor_height_mm
        ),
    }


def prman_camera_from_physical(
    focal_length_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: float,
    focus_distance_mm: float,
    f_stop: float,
) -> Dict[str, Any]:
    hfov = fov_from_focal_and_aperture(focal_length_mm, sensor_width_mm)
    aspect = sensor_width_mm / sensor_height_mm

    return {
        "model": "PrmanCamera",
        "fov_deg": hfov,
        "screenWindow": [-1.0, 1.0, -1.0 / aspect, 1.0 / aspect],
        "focalDistance_mm": focus_distance_mm,
        "fStop": f_stop,
    }


# -----------------------------------------------------------------------------
# Principal planes and entrance pupil
# -----------------------------------------------------------------------------


def object_distance_from_sensor(
    sensor_to_subject_mm: float,
    h1_offset_from_sensor_mm: float,
) -> float:
    return sensor_to_subject_mm - h1_offset_from_sensor_mm


def image_distance_to_sensor(
    image_distance_from_h2_mm: float,
    h2_offset_from_sensor_mm: float,
) -> float:
    return image_distance_from_h2_mm + h2_offset_from_sensor_mm


def estimate_entrance_pupil_practical(
    sensor_to_lens_front_mm: float,
    lens_length_mm: float,
    pupil_fraction_from_front: float = 0.4,
) -> float:
    return sensor_to_lens_front_mm - pupil_fraction_from_front * lens_length_mm


# -----------------------------------------------------------------------------
# Solver report
# -----------------------------------------------------------------------------


@dataclass
class LensReport:
    mode: str
    focal_length_mm: float
    f_stop: float
    focus_distance_mm: float
    coc_mm: float
    image_distance_mm: float
    hyperfocal_mm: float
    dof_near_mm: float
    dof_far_mm: float
    entrance_pupil_from_sensor_mm: Optional[float]
    gf_camera: Dict[str, Any]
    prman_camera: Dict[str, Any]


# -----------------------------------------------------------------------------
# High-level solver
# -----------------------------------------------------------------------------


def solve_lens_report(
    mode: str,
    focal_length_mm: float,
    f_stop: float,
    coc_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: float,
    focus_distance_sensor_to_subject_mm: float,
    anamorphic_squeeze: float = 1.0,
    h1_offset_mm: Optional[float] = None,
    h2_offset_mm: Optional[float] = None,
    entrance_pupil_offset_mm: Optional[float] = None,
    sensor_to_lens_front_mm: Optional[float] = None,
    lens_length_mm: Optional[float] = None,
) -> LensReport:

    if mode not in ("practical", "technical"):
        raise ValueError("mode must be practical or technical")

    # object distance
    if mode == "technical" and h1_offset_mm is not None:
        s = object_distance_from_sensor(
            focus_distance_sensor_to_subject_mm, h1_offset_mm
        )
    else:
        s = focus_distance_sensor_to_subject_mm

    # image distance from principal plane
    s_prime = thin_lens_image_distance(focal_length_mm, s)

    # image distance from sensor
    if mode == "technical" and h2_offset_mm is not None:
        image_distance_mm = image_distance_to_sensor(s_prime, h2_offset_mm)
    else:
        image_distance_mm = s_prime

    # DOF limits using the spherical equivalent (vertical axis)
    d_near, d_far, H = dof_limits_anamorphic(
        focal_length_mm, f_stop, coc_mm, s, anamorphic_squeeze
    )

    # entrance pupil / Pivot
    pupil_mm = None
    if mode == "technical" and entrance_pupil_offset_mm is not None:
        pupil_mm = entrance_pupil_offset_mm
    elif (
        mode == "practical"
        and sensor_to_lens_front_mm is not None
        and lens_length_mm is not None
    ):
        pupil_mm = estimate_entrance_pupil_practical(
            sensor_to_lens_front_mm, lens_length_mm
        )

    # GfCamera Mapping: USD utilizes horizontal/vertical aperture to define FOV
    # We maintain focal length but the horizontal FOV will expand via squeeze
    hfov = fov_from_focal_and_aperture(focal_length_mm, 
                                       sensor_width_mm, anamorphic_squeeze)
    vfov = fov_from_focal_and_aperture(focal_length_mm, sensor_height_mm, 1.0)

    gf = {
        "model": "GfCamera",
        "focalLength_mm": focal_length_mm,
        "horizontalAperture_mm": sensor_width_mm,
        "verticalAperture_mm": sensor_height_mm,
        "focusDistance_mm": focus_distance_sensor_to_subject_mm,
        "fStop": f_stop,
        "horizontalFOV_deg": hfov,
        "verticalFOV_deg": vfov,
        "anamorphicSqueeze": anamorphic_squeeze
    }

    # RenderMan Mapping: Uses FOV and Screen Window
    aspect = (sensor_width_mm * anamorphic_squeeze) / sensor_height_mm
    pr = {
        "model": "PrmanCamera",
        "fov_deg": hfov,
        "screenWindow": [-1.0, 1.0, -1.0 / aspect, 1.0 / aspect],
        "focalDistance_mm": focus_distance_sensor_to_subject_mm,
        "fStop": f_stop,
    }
    
    # GfCamera Mapping: USD utilizes horizontal/vertical aperture to define FOV
    # We maintain focal length but the horizontal FOV will expand via squeeze
    hfov = fov_from_focal_and_aperture(focal_length_mm, 
                                       sensor_width_mm, anamorphic_squeeze)
    vfov = fov_from_focal_and_aperture(focal_length_mm, sensor_height_mm, 1.0)

    return LensReport(
        mode=mode,
        focal_length_mm=focal_length_mm,
        f_stop=f_stop,
        focus_distance_mm=focus_distance_sensor_to_subject_mm,
        coc_mm=coc_mm,
        image_distance_mm=image_distance_mm,
        hyperfocal_mm=H,
        dof_near_mm=d_near,
        dof_far_mm=d_far,
        entrance_pupil_from_sensor_mm=pupil_mm,
        gf_camera=gf,
        prman_camera=pr,
    )


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------


def _add_common_args(p):
    p.add_argument("--focal", type=float, required=True)
    p.add_argument("--fstop", type=float, required=True)
    p.add_argument("--focus", type=float, required=True,
                   help="sensor-to-subject distance (mm)")
    p.add_argument("--sensor-width", type=float, required=True)
    p.add_argument("--sensor-height", type=float, required=True)
    p.add_argument("--coc", type=float, default=0.03)


def main():
    ap = argparse.ArgumentParser(prog="lens_report_camera")
    sub = ap.add_subparsers(dest="mode", required=True)

    p_prac = sub.add_parser("practical")
    _add_common_args(p_prac)
    p_prac.add_argument("--sensor-front", type=float)
    p_prac.add_argument("--lens-length", type=float)

    p_tech = sub.add_parser("technical")
    _add_common_args(p_tech)
    p_tech.add_argument("--h1", type=float, required=True)
    p_tech.add_argument("--h2", type=float, required=True)
    p_tech.add_argument("--pupil", type=float)

    args = ap.parse_args()

    if args.mode == "practical":
        rep = solve_lens_report(
            mode="practical",
            focal_length_mm=args.focal,
            f_stop=args.fstop,
            coc_mm=args.coc,
            sensor_width_mm=args.sensor_width,
            sensor_height_mm=args.sensor_height,
            focus_distance_sensor_to_subject_mm=args.focus,
            sensor_to_lens_front_mm=args.sensor_front,
            lens_length_mm=args.lens_length,
        )
    else:
        rep = solve_lens_report(
            mode="technical",
            focal_length_mm=args.focal,
            f_stop=args.fstop,
            coc_mm=args.coc,
            sensor_width_mm=args.sensor_width,
            sensor_height_mm=args.sensor_height,
            focus_distance_sensor_to_subject_mm=args.focus,
            h1_offset_mm=args.h1,
            h2_offset_mm=args.h2,
            entrance_pupil_offset_mm=args.pupil,
        )

    for k, v in asdict(rep).items():
        print(f"{k}: {v}")


if __name__ == "__main__":
    main()
```
