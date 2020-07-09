
/*
 Copyright (c) 2013 Nick Porcino, All rights reserved.
 License is MIT: http://opensource.org/licenses/MIT

 LabCamera has no external dependencies. Include LabCamera.cpp in your project.
*/


// LabCamera models a physical camera.
//
// Mount describes a camera's location and orientation in space, with the origin of
//  its coordinate system on the sensor plane
//
// Sensor describes the geometry of the sensor plane
//
// Optics is at the moment a simple pinhole lens.
//
// Camera composes a Mount, Sensor, and Optics.
//
// Conceptually it adds the notion of a camera pose, which is the location and
// orientation of the camera, and the notion of constraints. The camera can be
// controlled via the pose, or by imposing constraints on the pose.
//
// The camera interaction function is a stateless free function, implementing
// an orbit mode, a crane, a dolly, and a gimbal.
//
// A future version of LabCamera will have a conversion from geometry coordinates
// to millimeters to support calculation of depth effects, bokeh, and other
// calculations where the physical size of a lens affects how it interacts with
// the environment
//

#ifndef LAB_CAMERA_H
#define LAB_CAMERA_H

namespace lab {
namespace camera {

    //-------------------------------------------------------------------------
    // LabCamera doesn't provide a math library, just these trivial types
    // compatible with almost any other library via static casting or copying.
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

    // wrapping of a float value to communicate the unit of the value
    //
    struct millimeters { float value; };
    struct radians { float value; };

    constexpr radians radians_from_degrees(float degrees)
    {
        return { degrees * 0.01745329251f };
    }
    constexpr float degrees_from_radians(radians r)
    {
        return { r.value * 57.2957795131f };
    }

    //-------------------------------------------------------------------------
    // Mount is a nodal mount, centered on the camera's sensor
    // The mount's transform is left handed, y is up, -z is forward
    //
    class Mount
    {
        m44f _view_transform;

    public:
        Mount();

        // matrix
        m44f const& view_transform() const { return _view_transform; }
        m44f inv_view_transform() const;
        m44f rotation_transform() const { m44f j = _view_transform; j[3] = { 0,0,0,1 }; return j; }
        m44f inv_rotation_transform() const;

        // components
        quatf rotation() const;
        constexpr v3f right() const;
        constexpr v3f up() const;
        constexpr v3f forward() const;
        constexpr v3f position() const;

        // mutation
        void set_view_transform(quatf const& q, v3f const& pos);
        void look_at(v3f const& eye, v3f const& target, v3f const& up);
    };

    //-------------------------------------------------------------------------
    // Sensor describes the plane where an image is to be resolved.
    // The sensor's coordinate system has its center at (0, 0) and 
    // its bounds are -1 to 1.
    // enlarge is a multiplicative value; and shift is an additive value
    // in the sensor's coordinate system.
    //
    // Shift and enlarge can be used to create projection matrices for 
    // subregions of an image. For example, if the default Sensor is to be
    // rendered as four tiles, a matrix for rendering the upper left quadrant
    // can be computed by setting enlarge to { 2, 2 }, and
    // shift to millimeters{-17.5f}, millimeters{-12.25f}.
    //
    struct Sensor
    {
        struct Shift { millimeters x, y; };

        // spatial characteristics
        float handedness = -1.f; // left handed
        millimeters aperture_x = { 35.f };
        millimeters aperture_y = { 24.5f };
        v2f enlarge = { 1, 1 };        
        Shift shift = { millimeters{0}, millimeters{0} };

        // Utility function to derive a focal length for this sensor
        // corresponding to the sensor geometry.
        //
        millimeters focal_length_from_FOV(radians fov);
    };



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
        millimeters focal_length = { 50.f };
        float zfar = 1e5f;
        float znear = 0.1f;
        float squeeze = 1.f; // w/h - squeeze can be used to describe anamorphic pixels
    };

    m44f        perspective(const Sensor& sensor, const Optics& optics, float aspect = 1.f);
    m44f        inv_perspective(const Sensor& sensor, const Optics& optics, float aspect = 1.f);
    radians     vertical_FOV(const Sensor& sensor, const Optics& optics);

    //-------------------------------------------------------------------------
    // Camera

    enum class CameraRigMode
    {
        Dolly, Crane, TurnTableOrbit, Gimbal
    };

    class Camera
    {
        float _declination = 0;
        float _azimuth = 0;

    public:
        Mount  mount;
        Sensor sensor;
        Optics optics;

        v3f position{ 0, 0, 0 };
        v3f world_up{ 0, 1, 0 };
        v3f focus_point{ 0, 0, -10 };

        Camera();

        // cameraRig_interact
        //
        // delta is the 2d motion of a mouse or gesture in the screen plane, or
        // the absolute value of an analog joystick position.
        //
        // This interaction mode is intended for joystick like behaviors that have an
        // explicit neutral zero point. For example, delta could be computed as
        // delta = mousePos - mouseClickPos;
        //
        void rig_interact(CameraRigMode mode, v2f const& delta);

        // This mode is intended for screen space manipulation. 
        // Dolly: the camera will be moved in the view plane to keep initial under current
        // in the horizontal direction, and forward and backward motion will be under a 
        // heuristic
        //
        // Crane: the camera will be moved in the view plane to keep initial under current
        // 
        // TurnTableOrbit: The behavior will be roughly equivalent to traditional arcball
        //
        // Gimbal: The camera will be panned and tilted to keep initial under current

        void rig_interact(CameraRigMode mode,
            v2f const& viewport_size,
            Camera const& initial_camera,
            v2f const& initial, v2f const& current);

        // Creates a matrix suitable for an OpenGL style MVP matrix
        // Be sure to invert the view transform if your graphics engine pre-multiplies.
        //
        void update_view_transform() {
            mount.look_at(position, focus_point, world_up);
        }

        void frame(v3f const& bound1, v3f const& bound2);
        void set_clipping_planes_within_bounds(v3f const& bound1, v3f const& bound2);

        float distance_to_plane(v3f const& planePoint, v3f const& planeNormal);

        // Returns a world-space ray through the given pixel, originating at the camera
        Ray get_ray_from_pixel(v2f const& pixel, v2f const& viewport_origin, v2f const& viewport_size) const;

        m44f view_projection(float aspect = 1.f) const;
        m44f inv_view_projection(float aspect = 1.f) const;
    };


}
} // lab::camera

#endif
