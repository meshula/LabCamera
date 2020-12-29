# LabCamera

Copyright (c) 2013 Nick Porcino, All rights reserved.

License is MIT: http://opensource.org/licenses/MIT

Version 1.0 alpha

LabCamera has no external dependencies. Include LabCamera.cpp in your
project, and LabCamera.h in your include search path.

The camera is modeled as a set of elements, each with a distinct physical
role in the capture of an image. Each element is modeled by a struct, and
a camera struct is a composition of those elements.

lc_mount describes the pose of the camera. The mount is really a virtual
mount whose origin is at the lens' entrance pupil. It's up the an
application to model a kinematic chain such as a tripod or boom arm. The
rigid transform within the mount should be constrained to that kinematic
chain.

Following light from object in the world to sensor plane, the objects are:

- lc_optics - a mathematical description of a simple lens
- lc_aperture - the lens' exit pupil geometry
- lc_sensor - geometry of the imaging plane

LabCamera also provides an interactive controller that modifies camera
parameters in response to various input signals. It provides popular
cinematic and modeling controls: crane, dolly, tilt-pan, as well as
turntable and arcball control.

The single stick interface responds to an x/y motion vector. This interface
can be used where the control is via mouse interaction with a single
widget, or drags in an rendered viewport.

The dual stick interface responds to x/y motion vectors assigned
independently to position and orientation. This interface can be used where
the control inputs come from a device like a gamepad.

The through-the-lens interface is meant for control via pointing in a
rendered interactive view. Wherever possible, a location under a pointer
will remain under the pointer through the interaction. As a use case, the
pan-tilt interaction mode enables grabbing a visible feature in the image
and pulling that feature to a desired location in image space.

The constrained through-the-lens control extends the through the lens
interface to keep a hit-tested object in world space under the cursor. As a
use case, the crane interaction mode enables clicking an object at an
arbitrary distance from the camera and moving the camera proportional to
the distance to maintain the click point under the cursor as it moves.

LabCameraImgui is a convenient panel implementing various camera controls,
including a lens kit.

examples/Navigator is a demo app demonstrating the use of LabCamera.

_________

## Goals

### Modeling

- Physical camera modeling of optics, aperature, sensor, and mount
- Support for offset frustums for sensor shift and image tiling

### Interaction

- Cinematic interaction controls: pan, dolly, crane
- Modeling controls: arcball and turntable
- Through the lens and joystick style interaction

### Convenience

- Camera raycast helpers
- Sample Dear Imgui camera control panel for easy drop in usage
- Functions to create view and projection matrices

### Implementation

- No dependencies on a UI toolkit or graphics library or event model
- Trivial math types compatible with other libraries via static casting or copy
- No leakage of implementation details, including the math library

_________

## Model


LabCamera models a physical camera.

```Mount``` describes a camera's location and orientation in space, with the
origin of its coordinate system on the sensor plane

```Sensor``` describes the geometry of the sensor plane

```Optics``` is at the moment a simple pinhole lens.

```Camera``` composes a Mount, Sensor, and Optics.

```PanTiltController``` manipulates position, azimuth, and declination controls
to update the camera mount.

________________________________________________________________________________


LabCamera doesn't provide a math library, just these trivial types compatible
with almost any other library via static casting or copying.

```
typedef struct { float x, y; }       lc_v2f;
typedef struct { float x, y, z; }    lc_v3f;
typedef struct { float x, y, z, w; } lc_v4f;
typedef struct { float x, y, z, w; } lc_quatf;
typedef struct { lc_v4f x; lc_v4f y; lc_v4f z; lc_v4f w; } lc_m44f;
typedef struct { lc_v3f pos; lc_v3f dir; } lc_ray;
```

________________________________________________________________________________

```Mount``` is a nodal mount, centered on the camera's sensor
The mount's transform is by default, left handed, y is up, -z is forward

```Sensor``` describes the plane where an image is to be resolved.
The sensor's coordinate system has its center at (0, 0) and
its bounds are -1 to 1.
enlarge is a multiplicative value; and shift is an additive value
in the sensor's coordinate system.

Shift and enlarge can be used to create projection matrices for
subregions of an image. For example, if the default Sensor is to be
rendered as four tiles, a matrix for rendering the upper left quadrant
can be computed by setting enlarge to { 2, 2 }, and
shift to millimeters{-17.5f}, millimeters{-12.25f}.

The default sensor aperture is as for 35mm DSLR.

````Optics````

 A and B are on the sensor plane.

There is a point at infinity, O, on the axis of the lens, whose parallel rays
    converge on B.
````
                                   a
        ---------------------------+---------------+ A
                                 b| |              |
        O--------------------------+---------------+ B
                                 c| |              |
        ---------------------------+---------------+
        infinity                 lens           sensor plane
        C

    h = A-B is half the sensor plane aperture.
    f = b-B is the focal length
````

There is a point C at infinity, whose parallel rays through a, b, and c,
converge on the edge of the sensor plane, at A.

The field of view of the lens, at infinity, can therefore be approximated by
````fov = 2 atan((h/2) / f)````

Given a field of view, the assumption of a focus at infinity, and a sensor
aperture, the focal length may be calculated accordingly:

````f = tan(fov/2) / (h/2)````

an anamorphic lens' horizontal field of view must take squeeze into account:

````fov = 2 atan((s * h/2) / f)````

The hyperfocal distance is useful for determining what ranges are in focus.

If a camera is focused at infinity, objects from the hyperfocal distance to
infinity are in focus.

````
    O----------------H----------------------------|
    Infinity       hyperfocal distance         Sensor plane
````

If a camera is focussed at the hyperfocal distance, then objects from 1/2 H to
infinity are in focus.

````
    O----------------H----------H/2---------------|
    Infinity       hyperfocal distance         Sensor plane
````

Given a focus distance, and the hyperfocal distance, the in focus range is
given by

````
        Dn = hd / (h + (d - f))
        Df = hd / (h - (d - f))
````

Given a circle of confusion, such as 0.002mm as might be typical for 35mm film,
the hyperfocal distance can be determined.

```h = f^2/(fstop * CoC)```

````Camera````

 A Camera is comprised of a Mount, a Sensor, and Optics.

````PanTiltController````

Implements an interactive pan tilt controller
