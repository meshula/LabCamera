
/*------------------------------------------------------------------------------
    Copyright (c) 2013 Nick Porcino, All rights reserved.
    License is MIT: http://opensource.org/licenses/MIT

    Version 1.0

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

    lc_optics - a mathematical description of a simple lens
    lc_aperture - the lens' exit pupil geometry
    lc_sensor - geometry of the imaging plane

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
 */

#ifndef LAB_CAMERA_H
#define LAB_CAMERA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
    #define SRET(a) a
#else
    #define SRET(a) (a)
#endif

/*------------------------------------------------------------------------------
   LabCamera interfaces uses these math types. These types are compatible
   with almost any other library via static casting or copying. Note that the
   quaternion follows the gamedev convention of (im, re).
 */

typedef struct { float x, y; }       lc_v2f;
typedef struct { float x, y, z; }    lc_v3f;
typedef struct { float x, y, z, w; } lc_v4f;
typedef struct { float x, y, z, w; } lc_quatf;
typedef struct { lc_v4f x; lc_v4f y; lc_v4f z; lc_v4f w; } lc_m44f;
typedef struct { lc_v3f pos; lc_v3f dir; } lc_ray;

/*------------------------------------------------------------------------------
   Wrapped units and conversions. These clarify interface values that would be
   ambiguous if indicated by a floating point value.
 */

typedef struct { float m; }  lc_meters;
typedef struct { float mm; } lc_millimeters;
inline lc_millimeters m_as_mm(lc_meters m) {
    return SRET(lc_millimeters){ m.m * 1.e3f }; }
inline lc_meters mm_as_m(lc_millimeters m) {
    return SRET(lc_meters){ m.mm * 1.e-3f }; }

typedef struct { float rad; } lc_radians;
typedef struct { float deg; } lc_degrees;
inline lc_radians radians_from_degrees(lc_degrees d) {
    return SRET(lc_radians){ d.deg * 0.01745329251f }; }
inline lc_degrees degrees_from_radians(lc_radians r) {
    return SRET(lc_degrees){ r.rad * 57.2957795131f }; }

/*------------------------------------------------------------------------------
    lc_rigid_transform
  ------------------------------------------------------------------------------
    Basic utility functions include computation of basis
    vectors, and point and vector transformations.
 */

typedef struct {
    lc_quatf orientation;
    lc_v3f   position;
    lc_v3f   scale;
} lc_rigid_transform;

void lc_rt_set_identity(lc_rigid_transform*);

void lc_rt_set_ops(lc_rigid_transform*,
        lc_quatf orientation, lc_v3f position, lc_v3f scale);
void lc_rt_set_op_uniform_scale(lc_rigid_transform*,
        lc_quatf orientation, lc_v3f position, float scale);
void lc_rt_set_op(lc_rigid_transform*, lc_quatf orientation, lc_v3f position);

lc_v3f  lc_rt_right(const lc_rigid_transform*);
lc_v3f  lc_rt_up(const lc_rigid_transform*);
lc_v3f  lc_rt_forward(const lc_rigid_transform*);

lc_m44f lc_rt_matrix(const lc_rigid_transform*);
lc_v3f  lc_rt_transform_vector(const lc_rigid_transform*, lc_v3f vec);
lc_v3f  lc_rt_transform_point(const lc_rigid_transform*, lc_v3f p);
lc_v3f  lc_rt_detransform_point(const lc_rigid_transform*, lc_v3f p);
lc_v3f  lc_rt_detransform_vector(const lc_rigid_transform*, lc_v3f vec);

inline bool lc_rt_has_uniform_scale(const lc_rigid_transform* rt) {
    return rt->scale.x == rt->scale.y && rt->scale.x == rt->scale.z; }

/*------------------------------------------------------------------------------
    lc_mount
  ------------------------------------------------------------------------------
    A nodal mount, centered on the lens' entrance pupil, a point typically very
    near the intersection of the lens' optical axis and the sensor or film.

    The mount's transform is left handed, y is up, -z is forward

    Utilities are provided to calculate various useful matrices, and to
    extract yaw, pitch, and roll.
 */

typedef struct {
    lc_rigid_transform transform;
} lc_mount;

void lc_mount_set_default(lc_mount*);

lc_m44f lc_mount_rotation_transform(const lc_mount*);
lc_m44f lc_mount_inv_rotation_transform(const lc_mount*);
lc_v3f  lc_mount_ypr(const lc_mount*);

// derived values suitable for shaders
lc_m44f lc_mount_gl_view_transform(const lc_mount*);
lc_m44f lc_mount_gl_view_transform_inv(const lc_mount*);
lc_m44f lc_mount_model_view_transform_f16(const lc_mount*,
            float const*const view_matrix);
lc_m44f lc_mount_model_view_transform_m44f(const lc_mount*,
            lc_m44f const*const view_matrix);

// multiply mm by this value to get world values
/// @TODO add a setter, work this value through all calculations
inline float lc_mount_mm_to_world() { return 1000.f; }

// mutation
void lc_mount_set_view(lc_mount*,
            float distance, lc_quatf orientation, lc_v3f target, lc_v3f up);
void lc_mount_set_view_transform_m44f(lc_mount*, lc_m44f const*const);
void lc_mount_set_view_transform_f16(lc_mount*, float const* const);
void lc_mount_set_view_transform_quat_pos(lc_mount*, lc_quatf q, lc_v3f eye);
void lc_mount_set_view_transform_ypr_eye(lc_mount*, lc_v3f ypr, lc_v3f eye);
void lc_mount_look_at(lc_mount*, lc_v3f eye, lc_v3f target, lc_v3f up);

/*------------------------------------------------------------------------------
   lc_sensor
  ------------------------------------------------------------------------------

    describes the geometry of the plane where an image is to be resolved.

    The sensor's coordinate system has its center at (0, 0) and
    its bounds are -1 to 1.
    enlarge is a multiplicative value; and shift is an additive value
    in the sensor's coordinate system.

    Shift and enlarge can be used to create projection matrices for
    subregions of an image. For example, if the default lc_sensor is to be
    rendered as four tiles, a matrix for rendering the upper left quadrant
    can be computed by setting enlarge to { 2, 2 }, and
    shift to millimeters{-17.5f}, millimeters{-12.25f}.

    The default sensor aperture is as for 35mm DSLR.

    A handedness of -1 indicates a left handed camera

    lc_sensor does not describe the electrical or photochemical characteristics
    of a sensor.
 */

typedef struct {
    struct { lc_millimeters x, y; } shift;
    struct { float x, y; }          enlarge;
    struct { lc_millimeters x, y; } aperture;
    float                           handedness;
} lc_sensor;

void lc_sensor_set_default(lc_sensor* s);

// Derive focal length for the supplied sensor geometry and fov
lc_millimeters lc_sensor_focal_length_from_vertical_FOV(lc_sensor*, lc_radians);

/*------------------------------------------------------------------------------
    lc_optics
  ------------------------------------------------------------------------------

    A and B are on the sensor plane.

    There is a point at infinity, O, on the axis of the lens, whose parallel
    rays converge on B.
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

    If a camera is focussed at the hyperfocal distance, then objects
    from 1/2 H to infinity are in focus.

    O----------------H----------H/2---------------|
    Infinity       hyperfocal distance         Sensor plane

    Given a focus distance, and the hyperfocal distance, the in focus range is
    given by

        Dn = hd / (h + (d - f))
        Df = hd / (h - (d - f))

    Given a circle of confusion, such as 0.002mm as might be typical for
    35mm film, the hyperfocal distance can be determined.

        h = f^2/(fstop * CoC)

    @TODO PBRT has complex lens model, see RealisticCamera. Consider it for
    adoption here, perhaps as a secondary structure in Optics, or as a subclass.

    https://github.com/mmp/pbrt-v4/blob/master/src/pbrt/cameras.h

    PBRT defines Orthographic, Perspective, Spherical, and Realistic cameras.

    @TODO introduce at least Orthographic, in addtion to the present model
    which would be Perspective.

    Note that this struct does not model lens imperfections such as focus
    breathing, vignetting, distortion, flaring, spherical aberration or 
    chromatic aberration. Nor does it model field curvature or diffraction.
 */

typedef struct {
    //  fstop is focal length / diameter of effective aperture (entrance pupil)
    //   of the lens. The effective aperture is the appearance of the aperture
    //   as viewed from the front of the lens, the aperture "seen" by the 
    //   photons entering the front lens element.
    float fStop;

    // the amount of light loss that occurs as light travels through the optics
    // tStop is computed as fStop / sqrt(transmittance)
    float transmittance;

    // effective focal_length, matching a lens' field of view.
    lc_millimeters focal_length;

    // focus_distance is measured from film/sensor plane. If focused at the
    // hyperfocal distance, the value may be the hyperfocal distance,
    // or postive infinity.
    lc_meters focus_distance;

    float zfar;
    float znear;

    // squeeze describes anamorphic pixels, for example, Cinemascope squeeze
    // would be 2.
    float squeeze;
} lc_optics;

void      lc_optics_set_default(lc_optics*);
lc_meters lc_optics_hyperfocal_distance(lc_optics*, lc_millimeters CoC);

// return value is (width mm, height mm)
lc_v2f    lc_optics_focus_range(lc_optics*, lc_millimeters hyperfocal_distance);

/*------------------------------------------------------------------------------
    lc_aperture
  ------------------------------------------------------------------------------

    A slight simplification; the aperture describes both the pupil of the
    aperture and also the shutter.

    Currently the shape of the aperture is modeled as a straight sided polygon.
    In the future, an arbitrary mask could be specified to use in convolution
    or raytracing.

    shutter angle is the shutter duration * frame rate * 360 degrees, and not
    computed here.
 */

typedef struct {
    float          shutter_open;     // offset in seconds from start of exposure
    float          shutter_duration; // duration of exposure, in seconds
    unsigned int   shutter_blades;   // default 6
    lc_millimeters iris;             // the default aperture is 6.25mm,
                                     // corresponding to f8 for a 50mm lens
} lc_aperture;

void lc_aperture_set_default(lc_aperture* a);

/*------------------------------------------------------------------------------
    lc_camera
  ------------------------------------------------------------------------------

    A Camera is comprised of a Mount, a Sensor, and Optics.
    The camera implements interaction and constrain solving.

    The camera has some utility functions, and some interaction functions.
    The interaction methods must be called between a
    begin_interaction/end_interaction pair.

    Explicit constraints are the position of the camera, the world_up vector,
    and the focus_point. These constraints may be modified by the interaction
    methods; otherwise, they may be modified freely outside of a begin-end
    interaction block.

    Implicit constraints are specific to the interaction mode. For example, the
    TurnTableOrbit implicitly constrains the camera position to a point on a
    sphere whose radius corresponds to the distance from the camera to the focus
    point when begin_interaction() is invoked.
  */

typedef struct {
    bool hit;
    lc_v3f point;
} lc_hit_result;

typedef struct {
    lc_mount    mount;
    lc_optics   optics;
    lc_aperture aperture;
    lc_sensor   sensor;
} lc_camera;

void lc_camera_set_defaults(lc_camera*);

// perspective and field of view are computed from the sensor
// and optics of the camera.
//
lc_m44f    lc_camera_perspective(const lc_camera* cam, float aspect);
lc_m44f    lc_camera_inv_perspective(const lc_camera* cam, float aspect);
lc_radians lc_camera_vertical_FOV(const lc_camera* cam);
lc_radians lc_camera_horizontal_FOV(const lc_camera* cam);

// focal length of the lens, divided by the diameter of the iris opening
inline float lc_camera_f_stop(lc_camera* cam) {
    return cam->optics.focal_length.mm / cam->aperture.iris.mm; }

// move the camera along the view vector such that both bounds are visible
void lc_camera_frame(lc_camera* cam, lc_v3f bound1, lc_v3f bound2);

void lc_camera_set_clipping_planes_within_bounds(lc_camera* cam,
        float min_near, float max_far, lc_v3f bound1, lc_v3f bound2);

float lc_camera_distance_to_plane(const lc_camera* cam,
        lc_v3f planePoint, lc_v3f planeNormal);

// Returns a world-space ray through the given pixel, originating at the camera
lc_ray lc_camera_get_ray_from_pixel(const lc_camera*,
        lc_v2f pixel, lc_v2f viewport_origin, lc_v2f viewport_size);

lc_hit_result lc_camera_hit_test(const lc_camera*,
        lc_v2f mouse, lc_v2f viewport, lc_v3f plane_point, lc_v3f plane_normal);

lc_v2f lc_camera_project_to_viewport(const lc_camera*,
        lc_v2f viewport_origin, lc_v2f viewport_size, lc_v3f point);

lc_m44f lc_camera_view_projection(const lc_camera*, float aspect);
lc_m44f lc_camera_inv_view_projection(const lc_camera*, float aspect);

/// @TODO add a calculation for the entrance pupil, returned as an offset from
/// the sensor plane. Also add some words about why the entrance pupil and
/// focal length and distance to the sensor plane, and why the entrance pupil
/// and sensor plane are not at the same distance necessarily.

/*------------------------------------------------------------------------------
   lc_interaction
  ------------------------------------------------------------------------------
 */

typedef uint64_t InteractionToken;

typedef enum {
    lc_i_ModeStatic = 0,
    lc_i_ModeDolly,
    lc_i_ModeCrane,
    lc_i_ModeTurnTableOrbit,
    lc_i_ModePanTilt,
    lc_i_ModeArcball
} lc_i_Mode;

typedef enum {
    lc_i_PhaseNone = 0,
    lc_i_PhaseRestart,
    lc_i_PhaseStart,
    lc_i_PhaseContinue,
    lc_i_PhaseFinish
} lc_i_Phase;

typedef struct lc_interaction lc_interaction;

/* If the system being interfaced has a mouse motion, button cilcked/unclicked
   style interface, lc_update_phase simplifies Phase management.
 */
lc_i_Phase lc_update_phase(lc_i_Phase current_phase, bool button_click);;

lc_interaction* lc_i_create_interactive_controller();
void            lc_i_free_interactive_controller(lc_interaction* i);

lc_v3f lc_i_world_up_constraint(const lc_interaction*);
lc_v3f lc_i_orbit_center_constraint(const lc_interaction*);
void   lc_i_set_orbit_center_constraint(lc_interaction*, lc_v3f pos);
void   lc_i_set_world_up_constraint(lc_interaction*, lc_v3f up);
void   lc_i_set_speed(lc_interaction*, float orbit, float pan_tilt);

/* begin_interaction

   returns an InteractionToken. In the future this will be in aid of
   multitouch, multidevice interactions on the same camera
 */
InteractionToken lc_i_begin_interaction(lc_interaction*, lc_v2f viewport_size);
void             lc_i_end_interaction(lc_interaction*, InteractionToken);

/* Synchronize constraints and epoch to the most recent of the supplied
   controllers
 */
void lc_i_sync_constraints(lc_interaction*, lc_interaction*);

void lc_i_set_roll(lc_interaction*,
        lc_camera* camera, InteractionToken, lc_radians roll);

/* delta is the 2d motion of a mouse or gesture in the screen plane, or
   the absolute value of an analog joystick position.

   This interaction mode is intended for joystick like behaviors that have an
   explicit neutral zero point. For example, delta could be computed as
   delta = mousePos - mouseClickPos;
*/
void lc_i_single_stick_interaction(
    lc_interaction*,
    lc_camera*,
    InteractionToken,
    lc_i_Mode,
    lc_v2f delta_in,
    lc_radians roll_hint,
    float dt);

void lc_i_dual_stick_interaction(
    lc_interaction*,
    lc_camera*,
    InteractionToken,
    lc_i_Mode,
    lc_v3f pos_delta_in,
    lc_v3f rotation_delta_in,
    lc_radians roll_hint,
    float dt);

/* The through the lens interaction mode is intended for through as much
   through the lens interaction as is possible without a ray cast into
   the 3d world.
  
   Dolly: the camera will be moved in the view plane to keep initial under
          current in the horizontal direction, and forward and backward motion
          will be subject to a heuristic.

   Crane: the camera will be moved in the view plane to keep the initial 
          click location under the pointer.

   TurnTableOrbit: roughly screen relative tumble motions

   Gimbal: The camera will be panned and tilted to keep initial under current
 */
void lc_i_ttl_interaction(
    lc_interaction*,
    lc_camera*,
    InteractionToken,
    lc_i_Phase,
    lc_i_Mode,
    lc_v2f mouse_pos,
    lc_radians roll_hint,
    float dt);

/* Through the lens, with a point in world space to keep under the mouse.
   dolly:  hit_point will be constrained to stay under the mouse
   crane:  same
   gimbal: same
   turntable orbit, roughly screen relative tumble motions
 */
void lc_i_constrained_ttl_interaction(
    lc_interaction*,
    lc_camera*,
    InteractionToken,
    lc_i_Phase,
    lc_i_Mode,
    lc_v2f current,
    lc_v3f initial_hit_point,
    lc_radians roll_hint,
    float dt);

#ifdef __cplusplus
}       // extern "C"
#endif
#undef SRET

#endif  // LABCAMERA_H

