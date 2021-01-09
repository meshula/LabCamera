//
//  LabCamera.swift
//  
//
//  Created by Domenico Porcino on 1/4/21.
//

import liblabcamera
import simd

func simd_float4x4_from_lc_m44f(m: lc_m44f) -> simd_float4x4 {
    return simd_matrix(
        simd_float4(x:m.x.x, y:m.x.y, z:m.x.z, w:m.x.w),
        simd_float4(x:m.y.x, y:m.y.y, z:m.y.z, w:m.x.w),
        simd_float4(x:m.z.x, y:m.z.y, z:m.z.z, w:m.z.w),
        simd_float4(x:m.w.x, y:m.w.y, z:m.w.z, w:m.w.w))
}

func simd_float3_from_lc_v3f(v: lc_v3f) -> simd_float3 {
    return simd_float3(x: v.x, y: v.y, z: v.z)
}

func lc_v3f_from_simd_float3(v: simd_float3) -> lc_v3f {
    return lc_v3f(x:v.x, y:v.y, z:v.z)
}

func simd_float2_from_lc_v2f(v: lc_v2f) -> simd_float2 {
    return simd_float2(x: v.x, y: v.y)
}

func lc_v2f_from_simd_float2(v: simd_float2) -> lc_v2f {
    return lc_v2f(x:v.x, y:v.y)
}

class RigidTransform {
    internal var rt: lc_rigid_transform = lc_rigid_transform()
    var orientation: simd_quatf {
        get {
            return simd_quatf(angle:rt.orientation.w, axis:simd_float3(rt.orientation.x, rt.orientation.y, rt.orientation.z))
        }
        set (v) {
            rt.orientation.w = v.angle
            rt.orientation.x = v.axis.x
            rt.orientation.y = v.axis.y
            rt.orientation.z = v.axis.z
        }
    }
    var position: simd_float3 {
        get {
            return simd_float3(x: rt.position.x, y: rt.position.y, z: rt.position.z)
        }
        set (v) {
            rt.position.x = v.x
            rt.position.y = v.y
            rt.position.z = v.z
        }
    }
    var scale: simd_float3 {
        get {
            return simd_float3(x: rt.scale.x, y: rt.scale.y, z: rt.scale.z)
        }
        set (v) {
            rt.scale.x = v.x
            rt.scale.y = v.y
            rt.scale.z = v.z
        }
    }
    
    var right: simd_float3 {
        get {
            let v = lc_rt_right(&rt)
            return simd_float3(x: v.x, y: v.y, z: v.z)
        }
    }

    var up: simd_float3 {
        get {
            let v = lc_rt_up(&rt)
            return simd_float3(x: v.x, y: v.y, z: v.z)
        }
    }

    var forward: simd_float3 {
        get {
            let v = lc_rt_forward(&rt)
            return simd_float3(x: v.x, y: v.y, z: v.z)
        }
    }

    var matrix: simd_float4x4 {
        get { return simd_float4x4_from_lc_m44f(m: lc_rt_matrix(&rt)) }
    }
    
    init() {
        lc_rt_set_identity(&rt)
    }
    
    func setIdentity() {
        lc_rt_set_identity(&rt)
    }
    
    func transformVector(p: simd_float3) -> simd_float3 {
        let vec = lc_v3f(x: p.x, y: p.y, z: p.z)
        let v = lc_rt_transform_vector(&rt, vec)
        return simd_float3(x: v.x, y: v.y, z: v.z)
    }
    
    func transformPoint(p: simd_float3) -> simd_float3 {
        let vec = lc_v3f(x: p.x, y: p.y, z: p.z)
        let v = lc_rt_transform_point(&rt, vec)
        return simd_float3(x: v.x, y: v.y, z: v.z)
    }

    func detransformVector(p: simd_float3) -> simd_float3 {
        let vec = lc_v3f(x: p.x, y: p.y, z: p.z)
        let v = lc_rt_detransform_vector(&rt, vec)
        return simd_float3(x: v.x, y: v.y, z: v.z)
    }
    
    func detransformPoint(p: simd_float3) -> simd_float3 {
        let vec = lc_v3f(x: p.x, y: p.y, z: p.z)
        let v = lc_rt_detransform_point(&rt, vec)
        return simd_float3(x: v.x, y: v.y, z: v.z)
    }
}

class Mount {
    internal var m : lc_mount = lc_mount()
    
    init() {
        lc_mount_set_default(&m)
    }
    
    var mmToWorld: Float {
        get {
            return 1000.0 //return lc_mount_mm_to_world()
        }
    }
    
    var rotation: simd_float4x4 {
        get { return simd_float4x4_from_lc_m44f(m: lc_mount_rotation_transform(&m)) }
    }

    var inverseRotation: simd_float4x4 {
        get { return simd_float4x4_from_lc_m44f(m: lc_mount_inv_rotation_transform(&m)) }
    }
    
    var ypr: simd_float3 {
        get {
            let v = lc_mount_ypr(&m)
            return simd_float3(x:v.x, y:v.y, z:v.z)
        }
    }
    
    var GLView: simd_float4x4 {
        get { return simd_float4x4_from_lc_m44f(m:lc_mount_gl_view_transform(&m)) }
    }
    
    var inverseGLView: simd_float4x4 {
        get { return simd_float4x4_from_lc_m44f(m: lc_mount_gl_view_transform_inv(&m)) }
    }
    /*
    [ rx ux ux 0 ]
    [ ry uy uz 0 ]
    [ rz uz fz 0 ]
    [ tx ty tz 1 ]
    */
    func modelView(view: simd_float4x4) -> simd_float4x4 {
        var v = lc_m44f(
            x: lc_v4f(x:view.columns.0.x, y:view.columns.0.y, z:view.columns.0.z, w:view.columns.0.w),
            y: lc_v4f(x:view.columns.1.x, y:view.columns.1.y, z:view.columns.1.z, w:view.columns.1.w),
            z: lc_v4f(x:view.columns.2.x, y:view.columns.2.y, z:view.columns.2.z, w:view.columns.2.w),
            w: lc_v4f(x:view.columns.3.x, y:view.columns.3.y, z:view.columns.3.z, w:view.columns.3.w))
        return simd_float4x4_from_lc_m44f(m: lc_mount_model_view_transform_m44f(&m, &v))
    }
    
    func setView(distance: Float, orientation: simd_quatf, target: simd_float3, up: simd_float3) {
        let o = lc_quatf(x:orientation.axis.x, y:orientation.axis.y, z:orientation.axis.z, w:orientation.angle)
        let t = lc_v3f(x:target.x, y:target.y, z:target.z)
        let u = lc_v3f(x:up.x, y:up.y, z:up.y)
        lc_mount_set_view(&m, distance, o, t, u)
    }
    
    func setView(matrix: simd_float4x4) {
        var v = lc_m44f(
            x: lc_v4f(x:matrix.columns.0.x, y:matrix.columns.0.y, z:matrix.columns.0.z, w:matrix.columns.0.w),
            y: lc_v4f(x:matrix.columns.1.x, y:matrix.columns.1.y, z:matrix.columns.1.z, w:matrix.columns.1.w),
            z: lc_v4f(x:matrix.columns.2.x, y:matrix.columns.2.y, z:matrix.columns.2.z, w:matrix.columns.2.w),
            w: lc_v4f(x:matrix.columns.3.x, y:matrix.columns.3.y, z:matrix.columns.3.z, w:matrix.columns.3.w))
        lc_mount_set_view_transform_m44f(&m, &v)
    }

    func setView(orientation: simd_quatf, eye: simd_float3) {
        let o = lc_quatf(x:orientation.axis.x, y:orientation.axis.y, z:orientation.axis.z, w:orientation.angle)
        let t = lc_v3f(x:eye.x, y:eye.y, z:eye.z)
        lc_mount_set_view_transform_quat_pos(&m, o, t);
    }

    func setView(ypr: simd_float3, eye: simd_float3) {
        let o = lc_v3f(x:ypr.x, y:ypr.y, z:ypr.z)
        let t = lc_v3f(x:eye.x, y:eye.y, z:eye.z)
        lc_mount_set_view_transform_ypr_eye(&m, o, t);
    }

    func setView(eye: simd_float3, target: simd_float3, up: simd_float3) {
        let e = lc_v3f(x:eye.x, y:eye.y, z:eye.z)
        let t = lc_v3f(x:target.x, y:target.y, z:target.z)
        let u = lc_v3f(x:up.x, y:up.y, z:up.z)
        lc_mount_look_at(&m, e, t, u);
    }
    
    func lookAt(eye: simd_float3, target: simd_float3, up: simd_float3) {
        setView(eye: eye, target:target, up: up)
    }

}

struct Square
{
    var x: lc_millimeters
    var y: lc_millimeters
    
    init() {
        x = lc_millimeters()
        y = lc_millimeters()
        x.mm = 0
        y.mm = 0
    }
}

class Sensor
{
    internal var s: lc_sensor
    
    var shift: Square {
        get {
            var res = Square()
            res.x = s.shift.x
            res.y = s.shift.y
            return res
        }
        set(newSquare) {
            s.shift.x = newSquare.x
            s.shift.y = newSquare.y
        }
    }

    var enlarge: simd_float2 {
        get {
            return simd_float2(x: s.enlarge.x, y: s.enlarge.y)
        }
        set(newEnlarge) {
            s.enlarge.x = newEnlarge.x
            s.enlarge.y = newEnlarge.y
        }
    }
    
    var aperture: Square {
        get {
            var res = Square()
            res.x = s.aperture.x
            res.y = s.aperture.y
            return res
        }
        set(newSquare) {
            s.aperture.x = newSquare.x
            s.aperture.y = newSquare.y
        }
    }
    
    var handedness: Float {
        get {
            return s.handedness
        }
        set(h) {
            s.handedness = h
        }
    }
    
    init() {
        s = lc_sensor()
        lc_sensor_set_default(&s)
    }
    
    func FocalLengthFromVerticalFOV(radians: Float) -> lc_millimeters {
        var r: lc_radians = lc_radians()
        r.rad = radians
        return lc_sensor_focal_length_from_vertical_FOV(&s, r)
    }
}

class Optics
{
    internal var o: lc_optics

    var fStop: Float {
        get { return o.fStop }
        set(newF) { o.fStop = newF }
    }

    var focalLength: lc_millimeters {
        get { return o.focal_length }
        set(newF) { o.focal_length = newF }
    }
    
    var focusDistance: lc_meters {
        get { return o.focus_distance }
        set(newF) { o.focus_distance = newF }
    }

    var zFar: Float {
        get { return o.zfar }
        set(v) { o.zfar = v }
    }

    var zNear: Float {
        get { return o.znear }
        set(v) { o.znear = v }
    }

    var squeeze: Float {
        get { return o.squeeze }
        set(v) { o.squeeze = v }
    }

    init() {
        o = lc_optics()
        lc_optics_set_default(&o)
    }

    func HyperfocalDistance(CoC: lc_millimeters) -> lc_meters {
        return lc_optics_hyperfocal_distance(&o, CoC)
    }

    func FocusRange(hyperfocalDistance: lc_millimeters) -> Square {
        let r = lc_optics_focus_range(&o, hyperfocalDistance)
        var s: Square = Square()
        s.x.mm = r.x
        s.y.mm = r.y
        return s
    }
}

class Aperture {
    internal var a: lc_aperture

    var shutterOpen: Float {
        get { return a.shutter_open }
        set(v) { a.shutter_open = v }
    }

    var shutterDuration: Float {
        get { return a.shutter_duration }
        set(v) { a.shutter_duration = v }
    }

    var shutterBlades: UInt32 {
        get { return a.shutter_blades }
        set(v) { a.shutter_blades = v }
    }

    var iris: lc_millimeters {
        get { return a.iris }
        set(v) { a.iris = v }
    }

    init() {
        a = lc_aperture()
        lc_aperture_set_default(&a)
    }
}

class Camera {
    internal var c: lc_camera
    
    init() {
        c = lc_camera()
        lc_camera_set_defaults(&c)
    }

    func perspective(aspect:Float) -> simd_float4x4 {
        return simd_float4x4_from_lc_m44f(m: lc_camera_perspective(&c, aspect))
    }

    func inversePerspective(aspect:Float) -> simd_float4x4 {
        return simd_float4x4_from_lc_m44f(m: lc_camera_inv_perspective(&c, aspect))
    }
    
    func viewProjection(aspect:Float) -> simd_float4x4 {
        return simd_float4x4_from_lc_m44f(m: lc_camera_view_projection(&c, aspect))
    }
    
    func inverseViewProjection(aspect:Float) -> simd_float4x4 {
        return simd_float4x4_from_lc_m44f(m: lc_camera_inv_view_projection(&c, aspect))
    }

    var verticalFOV: lc_radians {
        get { return lc_camera_vertical_FOV(&c) }
    }
    
    var horizontalFOV: lc_radians {
        get { return lc_camera_horizontal_FOV(&c) }
    }
    
    var fStop: Float {
        return c.optics.focal_length.mm / c.aperture.iris.mm
    }
    
    func frame(spatialBound1: simd_float3, spatialBound2: simd_float3) {
        lc_camera_frame(&c, lc_v3f_from_simd_float3(v: spatialBound1), lc_v3f_from_simd_float3(v: spatialBound2))
    }
    
    func setClippingPlanes(minNear:Float, maxFar:Float, spatialBound1: simd_float3, spatialBound2: simd_float3) {
        lc_camera_set_clipping_planes_within_bounds(&c,
            minNear, maxFar, lc_v3f_from_simd_float3(v: spatialBound1), lc_v3f_from_simd_float3(v: spatialBound2))
    }

    func distanceToPlane(planePoint:simd_float3, planeNormal:simd_float3) -> Float {
        return lc_camera_distance_to_plane(&c, lc_v3f_from_simd_float3(v: planePoint), lc_v3f_from_simd_float3(v: planeNormal))
    }

    func rayFromPixel(pixel:simd_float2, viewportOrigin:simd_float2, viewportSize:simd_float2) -> lc_ray {
        return lc_camera_get_ray_from_pixel(&c, lc_v2f_from_simd_float2(v: pixel), lc_v2f_from_simd_float2(v: viewportOrigin), lc_v2f_from_simd_float2( v:viewportSize))
    }

    func hitTest(mouse:simd_float2, viewport:simd_float2, planePoint:simd_float3, planeNormal:simd_float3) -> lc_hit_result {
        return lc_camera_hit_test(&c, lc_v2f_from_simd_float2(v:mouse), lc_v2f_from_simd_float2(v: viewport), lc_v3f_from_simd_float3(v: planePoint), lc_v3f_from_simd_float3(v: planeNormal))
    }

    func projectToViewport(viewportOrigin:simd_float2, viewportSize:simd_float2, point:simd_float3) -> simd_float2 {
        return simd_float2_from_lc_v2f(v: lc_camera_project_to_viewport(&c, lc_v2f_from_simd_float2(v: viewportOrigin), lc_v2f_from_simd_float2(v: viewportSize), lc_v3f_from_simd_float3(v: point)))
    }

}

enum InteractionMode : Int {
    case Static = 0  // lc_i_ModeStatic
    case Dolly = 1   // lc_i_ModeDolly
    case Crane = 2   // lc_i_ModeTurntableOrbit
    case PanTilt = 3 // lc_i_ModePanTilt
    case Arcball = 4 // lc_i_ModeArcBall
}

enum InteractionPhase : Int {
    case None = 0 // lc_i_PhaseNone
    case Restart = 1 // lc_i_PhaseRestart
    case Start = 2 // lc_i_PhaseStart
    case Continue = 3 // lc_i_PhaseContinue
    case Finish = 4 // lc_i_PhaseFinish
}

class Interaction {
    internal var i: OpaquePointer?
    
    init() {
        i = lc_i_create_interactive_controller()
    }
    
    deinit {
        lc_i_free_interactive_controller(i)
        i = nil
    }

    var worldUpConstraint: simd_float3 {
        get { return simd_float3_from_lc_v3f(v: lc_i_world_up_constraint(i)) }
        set(up) { lc_i_set_world_up_constraint(i, lc_v3f_from_simd_float3(v: up)) }
    }

    var orbitCenterConstraint: simd_float3 {
        get { return simd_float3_from_lc_v3f(v: lc_i_orbit_center_constraint(i)) }
        set(center) { lc_i_set_orbit_center_constraint(i, lc_v3f_from_simd_float3(v: center)) }
    }

    func setSpeed(orbitSpeed: Float, pantiltSpeed: Float) {
        lc_i_set_speed(i, orbitSpeed, pantiltSpeed)
    }

    func beginInteraction(viewportSize: simd_float2) -> InteractionToken {
        return lc_i_begin_interaction(i, lc_v2f_from_simd_float2(v: viewportSize))
    }

    func endInteraction(interactionToken: InteractionToken) {
        lc_i_end_interaction(i, interactionToken)
    }

    static func syncConstraints(a: Interaction, b: Interaction) {
        lc_i_sync_constraints(a.i, b.i)
    }

    func setRoll(camera: Camera, interactionToken: InteractionToken, roll: lc_radians) {
        lc_i_set_roll(i, &camera.c, interactionToken, roll)
    }

    func singleStickInteraction(camera: Camera, interactionToken: InteractionToken, mode: InteractionMode, deltaIn: simd_float2, rollHint:lc_radians, dt: Float) {
        lc_i_single_stick_interaction(i, &camera.c,
                                      interactionToken, lc_i_Mode(rawValue: UInt32(mode.rawValue)),
                                      lc_v2f_from_simd_float2(v: deltaIn),
                                      rollHint, dt)
    }

    func dualStickInteraction(camera: Camera, interactionToken: InteractionToken, mode: InteractionMode, posDeltaIn: simd_float3, rotationDeltaIn: simd_float3, rollHint:lc_radians, dt: Float) {
        lc_i_dual_stick_interaction(i, &camera.c, interactionToken, lc_i_Mode(rawValue: UInt32(mode.rawValue)), lc_v3f_from_simd_float3(v: posDeltaIn), lc_v3f_from_simd_float3(v: rotationDeltaIn), rollHint, dt)
    }
    
    func TTLInteraction(camera: Camera, interactionToken: InteractionToken, phase: InteractionPhase, mode: InteractionMode, mouseDeltaIn: simd_float2, rollHint:lc_radians, dt: Float) {
        lc_i_ttl_interaction(i, &camera.c, interactionToken, lc_i_Phase(rawValue: UInt32(phase.rawValue)), lc_i_Mode(rawValue: UInt32(mode.rawValue)), lc_v2f_from_simd_float2(v: mouseDeltaIn), rollHint, dt)
    }

    func constraintedTTLInteraction(camera: Camera, interactionToken: InteractionToken, phase: InteractionPhase, mode: InteractionMode, currentMouse: simd_float2, initialHitPoint: simd_float3, rollHint:lc_radians, dt: Float) {
        lc_i_constrained_ttl_interaction(i, &camera.c, interactionToken, lc_i_Phase(rawValue: UInt32(phase.rawValue)), lc_i_Mode(rawValue: UInt32(mode.rawValue)), lc_v2f_from_simd_float2(v: currentMouse), lc_v3f_from_simd_float3(v: initialHitPoint), rollHint, dt)
    }
}
