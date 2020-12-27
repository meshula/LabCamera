# LabCamera

Copyright (c) 2013 Nick Porcino, All rights reserved.

License is MIT: http://opensource.org/licenses/MIT

LabCamera models interactive and computer vision cameras as a collection of
physical components: optics, aperature, sensor, and mount. Parameters are
physical, and compatible with real world measurements.

The LabCamera interactive controller offers many popular control models;
arcball, turntable, a variety of cinematic modes, such as crane and dolly.

LabCamera collects many useful calculations necessary in interactive and
computer vision applications, such as calculating hyperfocal distance, the
circle of confusion, and so on.

LabCamera has no external dependencies, just include LabCamera.cpp in your 
project.

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
    struct v2f { float x, y; };
    struct v3f { float x, y, z; };
    struct v4f { float x, y, z, w; };
    typedef v4f quatf;

    struct m44f {
        union {
            struct { v4f x; v4f y; v4f z; v4f w; };
            v4f array[4];
        };
        constexpr const v4f& operator[] (int j) const { return array[j]; }
        v4f& operator[] (int j) { return array[j]; }
    };

    struct Ray { v3f pos; v3f dir; };

    // wrapping of a float value to communicate the unit of the value
    //
    struct millimeters { float value; };
    struct radians { float value; };
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
