
/*
 Copyright (c) 2013 Nick Porcino, All rights reserved.
 License is MIT: http://opensource.org/licenses/MIT

    This is a single file library; define LAB_CAMERA_DRY in one and only
    one source file to instantiate the library.
*/


// LabCamera models a physical camera.
//
// Mount describes a camera's location and orientation in space, centered on the Sensor
// Sensor describes the image plane, both its size, and simple imaging characteristics
// Optics is at the moment a simple pinhole lens.
// Camera is composed of a Mount, Sensor, and Optics.
//
// The camera interaction function is a free function, and currently implements
// an orbit mode, a truck, and a dolly.


#ifndef LAB_CAMERA_H
#define LAB_CAMERA_H

namespace lab {
namespace camera {
    //-------------------------------------------------------------------------
    // LabCamera doesn't provide a math library, providing only
    // trivial types to give lab::camera opaque math types compatible with 
    // just about any other math library via simple casting or copying.
    //
    struct v2f { float x, y; };
    struct v3f { float x, y, z; };
    struct v4f { float x, y, z, w; };
    typedef v4f quatf;

    struct m44f {
        v4f x, y, z, w;
        constexpr const v4f& operator[] (int j) const { return (&x)[j]; }
        v4f& operator[] (int j) { return (&x)[j]; }
    };

    struct Ray { v3f pos; v3f dir; };

    // simple wrapping of a float value to communicate the unit of the value
    //
    struct millimeters { float value; };
    struct radians { float value; };

    //-------------------------------------------------------------------------
    // Mount is a nodal mount, centered on the camera's sensor
    // The mount's transform is left handed, y is up, -z is forward
    //
    class Mount
    {
        m44f _viewTransform;

    public:
        Mount();

        void setViewTransform(m44f const& t) { _viewTransform = t; }
        void setViewTransform(quatf const& q, float r);
        const m44f& viewTransform() const { return _viewTransform; }
        m44f rotationTransform() const { m44f j = _viewTransform; j[3] = { 0,0,0,1 }; return j; }
        quatf rotation() const;

        constexpr v3f right() const;
        constexpr v3f up() const;
        constexpr v3f forward() const;
        constexpr v3f position() const;

        void lookat(v3f eye, v3f target, v3f up);
    };

    //-------------------------------------------------------------------------
    // Sensor describes the plane where an image is to be resolved.
    // The sensor's coordinate system (SCS) has its center at (0, 0) and 
    // its bounds are -1 to 1.
    // enlarge is a multiplicative value; and shift is an additive value in the SCS
    //
    // lift, gain, knee, and gamma, apply individually to RGB.
    //
    struct Sensor
    {
        // spatial characteristics
        float handedness = -1.f; // left handed
        millimeters aperture_x = { 35.f };
        millimeters aperture_y = { 24.5f };
        v2f enlarge = { 1, 1 };
        v2f shift = { 0, 0 };

        // sensing characteristics
        v3f lift = { 0, 0, 0 };
        v3f gain = { 1, 1, 1 };
        v3f knee = { 1, 1, 1 };
        v3f gamma = { 2.2f, 2.2f, 2.2f };
    };

    /* struct Shutter
    5. Lens exit aperture

        @property apertureBladeCount

        The shape of out of focus highlights in a scene is commonly known as "bokeh".
        The aesthetic quality of a lens' bokeh is one of the characteristics that 
        drives the choice of a lens for a particular scene.To a large degree, the
        appearance of bokeh is governed by the shape of the lens aperture.Typical
        lens apertures are controlled by a series of overlapping blades that can be
        irised openand closed.A lens with a five blade aperture will yield a five
        sided bokeh.The default is zero, which is to be interpreted as a perfectly
        round aperture.

        Note that the effect of a filter on the front of the lens can be modeled
        equivalently at the exit aperture.The MIOCamera does not explicitly provide
        specification of such effects, but a simulation could incorporate them at
        this stage.

        @property maximumCircleOfConfusion
        Although the size of an out of focus bokeh highlight can be computed from
        other camera properties, it is often necessary to limit the size of the
        circle of confusion for aesthetic reasons.The circle of confusion is
        specified in mm, and the default is 0.05mm.The units are mm on the sensor
        plane.

        @property shutterOpenInterval

        The length of time in seconds the shutter is open, impacting the amount of
        light that reaches the sensorand also the length of motion blur trails.The
        shutter time is not the same thing as scene frame rate.The rule of thumb for
        movies is that the shutter time should be half the frame rate, so to achieve
        a "filmic" look, the shutter time choice might be 1 / 48 of a second, since
        films are usually projected at 24 frames per second.Shutter time is
        independent of simulation frame rate because motion blur trails and exposure
        times should be held constant in order to avoid flicker artifacts.
        */

    /*
    ---------------------------------------------------------------------------
     Optics

    A and B are on the sensor plane.

    There is a point at infinity, O, on the axis of the lens, whose parallel rays
    converge on B.
                                   a
        ---------------------------+---------------+ A
                                 b| |              |
        O--------------------------+---------------+ B
                                 c| |              |
        ---------------------------+---------------+
        infinity                 lens           sensor plane
        C

    A-B is half the sensor plane aperture.
    b-B is the focal length

    There is a point C at infinity, whose parallel rays through a, b, and c,
    converge on the edge of the sensor plane, at A.

    The field of view of the lens, at infinity, can therefore be approximated by

        fov = 2 atan((h/2) / f)

    Given a field of view, the assumption of a focus at infinity, and a sensor
    aperture, the focal length may be calculated accordingly:

        f = tan(fov/2) / (h/2)

    */

    struct Optics
    {
        millimeters focalLength = { 50.f };
        float zfar = 1e5f;
        float znear = 0.1f;
        float squeeze = 1.f; // w/h

        //float focusDistance = 10.f;
        //float barrelDistortion = 0.f;
        //float fisheyeDistortion = 0.f;
        /*
            Illuminated objects result in scene luminance, which passes through the lens.
            All lenses impose some amount of radial distortion which can be computed from
            focal length.However, some lenses introduce error, and radial distortion can
            be used as an aesthetic control as well.Therefore radial distortion is
            provided as a property.If r is the radial distance of a pixel from the center
            of the projection, then radial distortion is computed as

            r' = r * (1 + barrelDistorion * r^2 + fisheyeDistortion * r^4)
            radialDistortion sufficiently describes the distortion characteristic of most
            lenses.In order to simulate certain other lenses, such as those found in
            security cameras, fisheye lenses, plastic toy lenses, sport cameras, or some
            VR headsets, radialDistortion2 is introduced.

            The default for the radial distortion parameters is zero, resulting in a
            rectilinear projection.
            */

            // float opticalVignetting = 0;
        /*

            Optical vignetting occurs to some degree in all lenses.It results from light
            at the edge of an image being blocked as it travels past the lens hoodand
            the internal lens apertures.It is more prevalent with wide apertures.A
            value of zero indicates no optical vignetting is occuring, and a value of one
            indicates that vignetting affects all locations in the image according to
            radial distance.Optical vignetting also occurs in head mounted displays, and
            the value here can be used as an intended amount of vignetting to apply to an
            image.
            */

            //float chromaticAberration = 0.f;
/*
            Chromatic aberration occurs to some degree in all lenses.It results from a
            lens bringing different wavelengths of light to focus at different places on
            the image plane.A value of zero indicates no chromatic aberration is
            occurring, and one indicates maximum.Chromatic aberration affects all
            locations in the image according to radial distance.Chromatic aberration
            also occurs in head mounted displays, and the value here can be used as an
            intended amount of chromatic aberration to apply to an image.
            */

        //float fStop = 5.6f;
        /*
    The f-stop is the ratio of the lens' focal length to the diameter of the 
    entrance pupil. The default is 5.6. It controls the amount of light that 
    reaches the sensor, as well as the size of out of focus parts of the image.
    The diameter of the entrance pupil, is therefore obtained
    by dividing the fStop by the focalLength.*/
    };

    m44f        perspective(const Sensor& sensor, const Optics& optics, float aspect = 1.f);
    m44f        inv_perspective(const Sensor& sensor, const Optics& optics, float aspect = 1.f);
    radians     verticalFOV(const Sensor& sensor, const Optics& optics);
    millimeters focal_length_from_FOV(millimeters sensor_aperture, radians fov);

    //-------------------------------------------------------------------------
    // Camera

    class Camera
    {
    public:
        Mount  mount;
        Sensor sensor;
        Optics optics;

        v3f position{ 0, 0, 0 };
        v3f worldUp{ 0, 1, 0 };
        v3f focusPoint{ 0, 0, -10 };

        Camera() {
            updateViewTransform();
        }

        // Creates a matrix suitable for an OpenGL style MVP matrix
        // Be sure to invert the view transform if your graphics engine pre-multiplies.
        //
        void updateViewTransform() {
            mount.lookat(position, focusPoint, worldUp);
        }

        void frame(v3f bound1, v3f bound2);
        void autoSetClippingPlanes(v3f bound1, v3f bound2);

        float planeIntersect(v3f planePoint, v3f planeNormal);

        // Returns a world-space ray through the given pixel, originating at the camera
        Ray get_ray_from_pixel(v2f pixel, v2f viewport_origin, v2f viewport_size) const;

        m44f viewProj(float aspect = 1.f) const;
        m44f inv_viewProj(float aspect = 1.f) const;
    };

    enum class CameraRigMode
    {
        Dolly, Crane, TurnTableOrbit, Gimbal
    };

    // delta is the 2d motion of a mouse or gesture in the screen plane,
    // typically computed as scale * (currMousePos - prevMousePos);
    //
    void cameraRig_interact(Camera& camera, CameraRigMode mode, v2f delta);
}
} // lab::camera

#endif
