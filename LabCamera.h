
//-------------------------------------------------------------------------
// Copyright (c) 2013 Nick Porcino, All rights reserved.
// License is MIT: http://opensource.org/licenses/MIT
//
// LabCamera has no external dependencies. Include LabCamera.cpp in your project.
//
#ifndef LAB_CAMERA_H
#define LAB_CAMERA_H

#include <stdint.h>

//-------------------------------------------------------------------------
// LabCamera doesn't provide a math library, just these trivial types
// compatible with almost any other library via static casting or copying.
//
typedef struct { float x, y; } lc_v2f;
typedef struct { float x, y, z; } lc_v3f;
typedef struct { float x, y, z, w; } lc_v4f;
typedef struct { lc_v4f x; lc_v4f y; lc_v4f z; lc_v4f w; } lc_m44f;
typedef lc_v4f  lc_quatf;
typedef struct { lc_v3f pos; lc_v3f dir; } lc_ray;

//-------------------------------------------------------------------------
// wrapped units and conversions
//
typedef struct { float m; } lc_meters;
typedef struct { float mm; } lc_millimeters;
inline lc_millimeters m_as_mm(lc_meters m) { return lc_millimeters{ m.m * 1.e3f }; }
inline lc_meters mm_as_m(lc_millimeters m) { return lc_meters{ m.mm * 1.e-3f }; }

typedef struct { float rad; } lc_radians;
typedef struct { float deg; } lc_degrees;
inline lc_radians radians_from_degrees(lc_degrees d) { return { d.deg * 0.01745329251f }; }
inline lc_degrees degrees_from_radians(lc_radians r) { return { r.rad * 57.2957795131f }; }

//-------------------------------------------------------------------------
// rigid transform
//
typedef struct
{
    lc_quatf orientation;
    lc_v3f   position;
    lc_v3f   scale;
} lc_rigid_transform;

void lc_rt_set_identity(lc_rigid_transform*);
void lc_rt_set_ops(lc_rigid_transform*, lc_quatf orientation, lc_v3f position, lc_v3f scale);
void lc_rt_set_op_uniform_scale(lc_rigid_transform*, lc_quatf orientation, lc_v3f position, float scale);
void lc_rt_set_op(lc_rigid_transform*, lc_quatf orientation, lc_v3f position);

lc_v3f  lc_rt_right(const lc_rigid_transform*);
lc_v3f  lc_rt_up(const lc_rigid_transform*);
lc_v3f  lc_rt_forward(const lc_rigid_transform*);

lc_m44f lc_rt_matrix(const lc_rigid_transform*);
lc_v3f  lc_rt_transform_vector(const lc_rigid_transform*, lc_v3f vec);
lc_v3f  lc_rt_transform_point(const lc_rigid_transform*, lc_v3f p);
lc_v3f  lc_rt_detransform_point(const lc_rigid_transform*, lc_v3f p);
lc_v3f  lc_rt_detransform_vector(const lc_rigid_transform*, lc_v3f vec);

inline bool lc_rt_is_uniform_scale(const lc_rigid_transform* rt) { return rt->scale.x == rt->scale.y && rt->scale.x == rt->scale.z; }


#ifdef __cplusplus
namespace lab {
namespace camera {

    typedef lc_v2f v2f;
    typedef lc_v3f v3f;
    typedef lc_v4f v4f;
    typedef lc_quatf quatf;
    typedef lc_m44f m44f;

    //-------------------------------------------------------------------------
    // Mount is a nodal mount, centered on the camera's sensor
    // The mount's transform is left handed, y is up, -z is forward
    //
    class Mount
    {
        lc_rigid_transform _transform;

    public:
        Mount();
        ~Mount();

        // matrix
        m44f gl_view_transform() const;
        m44f gl_view_transform_inv() const;
        m44f model_view_transform(m44f const& view_matrix) const;
        m44f model_view_transform(float const* const view_matrix) const;
        m44f rotation_transform() const;
        m44f inv_rotation_transform() const;

        // components
        const lc_rigid_transform* transform() const { return &_transform; }
        v3f ypr() const;

        float mm_to_world() const { return 1000.f; } // multiply mm by this value to get world values

        // mutation
        void set_view_transform(m44f const&);
        void set_view_transform_quat_pos(quatf const& q, v3f const& eye);
        void set_view_transform_ypr_eye(v3f const& ypr, v3f const& eye);
        void look_at(v3f const& eye, v3f const& target, v3f const& up);
        void look_at(float distance, quatf const& orientation, v3f const& target, v3f const& up);
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
    // The default sensor aperture is as for 35mm DSLR.
    //
    class Sensor
    {
    public:
        struct Shift { lc_millimeters x, y; };

        // spatial characteristics
        float handedness = -1.f; // left handed
        lc_millimeters aperture_x = { 35.f };
        lc_millimeters aperture_y = { 24.5f };
        v2f enlarge = { 1, 1 };
        Shift shift = { lc_millimeters{0}, lc_millimeters{0} };

        // Utility function to derive a focal length for this sensor
        // corresponding to the sensor geometry.
        //
        lc_millimeters focal_length_from_vertical_FOV(lc_radians fov);
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

    h = A-B is half the sensor plane aperture.
    f = b-B is the focal length

    There is a point C at infinity, whose parallel rays through a, b, and c,
    converge on the edge of the sensor plane, at A.

    The field of view of the lens, at infinity, can therefore be approximated by

        fov = 2 atan((h/2) / f)

    Given a field of view, the assumption of a focus at infinity, and a sensor
    aperture, the focal length may be calculated accordingly:

        f = tan(fov/2) / (h/2)

    an anamorphic lens' horizontal field of view must take squeeze into account:

        fov = 2 atan((s * h/2) / f)

    The hyperfocal distance is useful for determining what ranges are in focus.

    If a camera is focused at infinity, objects from the hyperfocal distance to
    infinity are in focus.

    O----------------H----------------------------|
    Infinity       hyperfocal distance         Sensor plane

    If a camera is focussed at the hyperfocal distance, then objects from 1/2 H to
    infinity are in focus.

    O----------------H----------H/2---------------|
    Infinity       hyperfocal distance         Sensor plane

    Given a focus distance, and the hyperfocal distance, the in focus range is
    given by

        Dn = hd / (h + (d - f))
        Df = hd / (h - (d - f))

    Given a circle of confusion, such as 0.002mm as might be typical for 35mm film,
    the hyperfocal distance can be determined.

        h = f^2/(fstop * CoC)

    @TODO PBRT has complex lens model, see RealisticCamera. Consider it for adoption here,
    perhaps as a secondary structure in Optics, or as a subclass.

    https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/cameras.h

    PBRT defines Orthographic, Perspective, Spherical, and Realistic cameras.

    @TODO introduce at least Orthographic, in addtion to the present model which wuold
    be Perspective.

    */

    class Optics
    {
    public:
        float fStop = 8.f;

        // effective focal_length, matching a lens' field of view.
        lc_millimeters focal_length = { 50.f };

        // focus_distance is measured from film/sensor plane. If focused at the
        // hyperfocal distance, the value may be the hyperfocal distance, or postivie infinity.
        lc_meters focus_distance = { 3.f };

        float zfar = 1e5f;
        float znear = 0.1f;

        // squeeze describes anamorphic pixels, for example, Cinemascope squeeze would be 2.
        float squeeze = 1.f;

        lc_meters hyperfocal_distance(lc_millimeters CoC);
        v2f    focus_range(lc_millimeters hyperfocal_distance); // return value in mm
    };

    /*-------------------------------------------------------------------------
      Aperture

      A slight simplification; the aperture describes both the pupil of the
      aperture and also the shutter.

      Currently the shape of the aperture is modeled as a straight sided polygon.
      In the future, an arbitrary mask could be specified to use in convolution
      or raytracing.
     */

    struct Aperture
    {
        float shutter_open = 0.f;
        float shutter_duration = 0.f;
        unsigned int shutter_blades = 7;

        // this iris will result in an f-stop of 8 for the default 50mm lens.
        lc_millimeters iris{ 6.25f };
    };


    /*-------------------------------------------------------------------------
     Camera

     A Camera is comprised of a Mount, a Sensor, and Optics.
     The camera implements interaction and constrain solving.

     The camera has some utility functions, and some interaction functions.
     The interaction methods must be called between a begin_interaction/end_interaction
     pair.

     Explicit constraints are the position of the camera, the world_up vector, and
     the focus_point. These constraints may be modified by the interaction methods;
     otherwise, they may be modified freely outside of a begin-end interaction block.

     Implicit constraints are specific to the interaction mode. For example, the
     TurnTableOrbit implicitly constrains the camera position to a point on a sphere
     whose radius corresponds to the distance from the camera to the focus point when
     begin_interaction() is invoked.

     */

    struct HitResult
    {
        bool hit;
        v3f point;
    };

    class Camera
    {
    public:
        Mount    mount;
        Optics   optics;
        Aperture aperture;
        Sensor   sensor;

        Camera();
        ~Camera();

        // perspective and field of view are computed from the sensor
        // and optics of the camera.
        //
        m44f        perspective(float aspect = 1.f) const;
        m44f        inv_perspective(float aspect = 1.f) const;
        lc_radians     vertical_FOV() const;
        lc_radians     horizontal_FOV() const;

        // focal length of the lens, divided by the diameter of the iris opening
        float f_stop() const { return optics.focal_length.mm / aperture.iris.mm; }

        // move the camera along the view vector such that both bounds are visible
        void frame(v3f const& bound1, v3f const& bound2);

        void set_clipping_planes_within_bounds(float min_near, float max_far, v3f const& bound1, v3f const& bound2);

        float distance_to_plane(v3f const& planePoint, v3f const& planeNormal) const;

        // Returns a world-space ray through the given pixel, originating at the camera
        lc_ray get_ray_from_pixel(v2f const& pixel, v2f const& viewport_origin, v2f const& viewport_size) const;

        HitResult hit_test(const v2f& mouse, const v2f& viewport, const v3f& plane_point, const v3f& plane_normal) const;

        v2f project_to_viewport(v2f const& viewport_origin, v2f const& viewport_size, const v3f& point) const;

        m44f view_projection(float aspect = 1.f) const;
        m44f inv_view_projection(float aspect = 1.f) const;
    };


    typedef uint64_t InteractionToken;

    enum class InteractionMode
    {
        Static = 0, Dolly, Crane, TurnTableOrbit, PanTilt, Arcball
    };

    enum class InteractionPhase
    {
        None = 0, Restart, Start, Continue, Finish
    };

    class PanTiltController
    {
        uint64_t _epoch = 0;

        // constraints
        v3f _world_up{ 0, 1, 0 };
        v3f _orbit_center{ 0, 0, 0 };

        // local settings
        float _orbit_speed = 0.5f;
        float _pan_tilt_speed = 0.25f;

        // working state
        v2f _viewport_size = { 0, 0 };
        v3f _initial_focus_point = { 0, 0, 0 };
        float _initial_focus_distance = 5.f;
        v2f _init_mouse{ 0,0 };
        v2f _prev_mouse{ 0,0 };
        m44f _initial_inv_projection = { 1,0,0,0, 0,1,0,0, 0,0,1,0.2f, 0,0,0,1 };

        void _dolly(Camera& camera, const v3f& delta);
        void _turntable(Camera& camera, const v2f& delta);
        void _pantilt(Camera& camera, const v2f& delta);

    public:
        PanTiltController() = default;

        v3f world_up_constraint() const;
        v3f orbit_center_constraint() const;
        void set_orbit_center_constraint(v3f const& pos);
        void set_world_up_constraint(v3f const& up);
        void set_speed(float orbit, float pan_tilt);

        // begin_interaction
        //
        // returns an InteractionToken. In the future this will be in aid of
        // multitouch, multidevice interactions on the same camera
        //
        InteractionToken begin_interaction(v2f const& viewport_size);
        void end_interaction(InteractionToken);

        // Synchronize constraints and epoch to the most recent of this and controller.
        void sync_constraints(PanTiltController& controller);

        void set_roll(Camera& camera, InteractionToken, lc_radians roll);

        // delta is the 2d motion of a mouse or gesture in the screen plane, or
        // the absolute value of an analog joystick position.
        //
        // This interaction mode is intended for joystick like behaviors that have an
        // explicit neutral zero point. For example, delta could be computed as
        // delta = mousePos - mouseClickPos;
        //
        void single_stick_interaction(
            Camera& camera,
            InteractionToken,
            InteractionMode mode,
            v2f const& delta_in,
            lc_radians roll_hint,
            float dt);

        void dual_stick_interaction(
            Camera& camera,
            InteractionToken,
            InteractionMode mode,
            v3f const& pos_delta_in,
            v3f const& rotation_delta_in,
            lc_radians roll_hint,
            float dt);

        // This mode is intended for through the lens screen space manipulation.
        // Dolly: the camera will be moved in the view plane to keep initial under current
        // in the horizontal direction, and forward and backward motion will be under a
        // heuristic
        //
        // Crane: the camera will be moved in the view plane to keep initial under current
        //
        // TurnTableOrbit: roughly screen relative tumble motions
        //
        // Gimbal: The camera will be panned and tilted to keep initial under current
        //
        void ttl_interaction(
            Camera& camera,
            InteractionToken tok,
            InteractionPhase phase,
            InteractionMode mode,
            v2f const& current,
            lc_radians roll_hint,
            float dt);

        // through the lens, with a point in world space to keep under the mouse.
        // dolly: hit_point will be constrained to stay under the mouse
        // crane: same
        // gimbal: same
        // turntable orbit, roughly screen relative tumble motions
        //
        void constrained_ttl_interaction(
            Camera& camera,
            InteractionToken tok,
            InteractionPhase phase,
            InteractionMode mode,
            v2f const& current,
            v3f const& initial_hit_point,
            lc_radians roll_hint,
            float dt);
    };


}
} // lab::camera
#endif // _cplusplus

#endif
