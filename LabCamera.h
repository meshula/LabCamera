
/*
 Copyright (c) 2013 Nick Porcino, All rights reserved.
 License is MIT: http://opensource.org/licenses/MIT

    This is a single file library; define LAB_CAMERA_DRY in one and only
    one source file to instantiate the library.
*/

#ifndef LAB_CAMERA_H
#define LAB_CAMERA_H

namespace lab {
namespace camera {

    // These trivial types are provided to give lab::camera opaque math
    // types compatible with just about any other math library via
    // simple casting or copying.
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


    // MM and Radians are provided as simple reminders as to what the
    // intended units are. There is no template trickery to provide
    // magic conversions to other measures.
    //
    typedef float MM;      // millimeters
    typedef float Radians;

    // Mount is a nodal mount, centered on the camera's sensor
    // The mount's transform is left handed, y is up, -z is forward

    class Mount
    {
        m44f _viewTransform;

    public:
        Mount();

        void setViewTransform(m44f const& t) { _viewTransform = t; }
        void setViewTransform(quatf const& q, float r);
        const m44f& viewTransform() const { return _viewTransform; }
        m44f rotationTransform() const { m44f j = _viewTransform; j[3] = { 0,0,0,1 }; return j; }

        v3f right() const;
        v3f up() const;
        v3f forward() const;
        v3f position() const;

        void lookat(v3f eye, v3f target, v3f up);
    };

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
        MM aperture_x = 35.f;
        MM aperture_y = 24.5f;
        v2f enlarge = { 1, 1 };
        v2f shift = { 0, 0 };

        // sensing characteristics
        v3f lift = { 0, 0, 0 };
        v3f gain = { 1, 1, 1 };
        v3f knee = { 1, 1, 1 };
        v3f gamma = { 2.2f, 2.2f, 2.2f };
    };

    /*
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
        MM focalLength = 50.f;
        float zfar = 1e5f;
        float znear = 0.1f;
        float squeeze = 1.f; // w/h
    };

    m44f    perspective(const Sensor& sensor, const Optics& optics);
    Radians verticalFOV(const Sensor& sensor, const Optics& optics);
    MM focal_length_from_FOV(MM sensor_aperture, Radians fov);

    m44f perspective(const Sensor& sensor, const Optics& optics);

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
    };

    enum class CameraRigMode
    {
        Dolly, Crane, TurnTableOrbit, Fly
    };

    // delta is the 2d motion of a mouse or gesture in the screen plane,
// typically computed as scale * (currMousePos - prevMousePos);
//
    void cameraRig_interact(Camera& camera, CameraRigMode mode, v2f delta);

}
} // lab::camera

#endif

#ifdef LAB_CAMERA_DRY

#include <algorithm>
#include <cmath>

namespace lab {
    namespace camera {

        m44f m44f_identity = { 1.f,0.f,0.f,0.f, 0.f,1.f,0.f,0.f, 0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f };

        // Support for 3D spatial rotations using quaternions, via qmul(qmul(q, v), qconj(q))
        constexpr v3f qxdir(const quatf& q) { return { q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, (q.x * q.y + q.z * q.w) * 2, (q.z * q.x - q.y * q.w) * 2 }; }
        constexpr v3f qydir(const quatf& q) { return { (q.x * q.y - q.z * q.w) * 2, q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, (q.y * q.z + q.x * q.w) * 2 }; }
        constexpr v3f qzdir(const quatf& q) { return { (q.z * q.x + q.y * q.w) * 2, (q.y * q.z - q.x * q.w) * 2, q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z }; }

        constexpr v3f operator + (const v3f& a, const v3f& b) { return v3f{ a.x + b.x, a.y + b.y, a.z + b.z }; }
        constexpr v3f operator - (const v3f& a, const v3f& b) { return v3f{ a.x - b.x, a.y - b.y, a.z - b.z }; }
        constexpr v3f operator * (const v3f& a, float b) { return v3f{ a.x * b, a.y * b, a.z * b }; }
        constexpr v4f operator + (const v4f& a, const v4f& b) { return v4f{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
        constexpr v4f operator * (const v4f& a, float b) { return v4f{ a.x * b, a.y * b, a.z * b, a.w * b }; }
        constexpr v4f mul(const m44f& a, const v4f& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
        constexpr m44f mul(const m44f& a, const m44f& b) { return { mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w) }; }
        constexpr v3f cross(const v3f& a, const v3f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
        constexpr float dot(const v3f& a, const v3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z ; }
        float length(const v3f& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
        v3f normalize(const v3f& a) { return a * (1.f / length(a)); }
        constexpr v3f& operator += (v3f& a, const v3f& b) { return a = a + b; }

        inline quatf quat_fromAxisAngle(v3f v, float a)
        {
            quatf Result;
            float s = std::sin(a * 0.5f);
            Result.w = std::cos(a * 0.5f);
            Result.x = v.x * s;
            Result.y = v.y * s;
            Result.z = v.z * s;
            return Result;
        }

        inline v3f quat_rotateVector(quatf q, const v3f& v)
        {
            // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
            v3f u{ q.x, q.y, q.z };
            float s = q.w;

            return  u * 2.f * dot(u, v)
                + v * (s * s - dot(u, u))
                + cross(u, v) * 2.f * s;
        }

        inline m44f make_lookat_transform(const v3f& eye, const v3f& target, const v3f& up)
        {
            v3f zaxis = normalize(eye - target);
            v3f xaxis = normalize(cross(up, zaxis));
            v3f yaxis = cross(zaxis, xaxis);
            return { { xaxis.x, yaxis.x, zaxis.x, 0.f },
                     { xaxis.y, yaxis.y, zaxis.y, 0.f },
                     { xaxis.z, yaxis.z, zaxis.z, 0.f },
                     { -dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1.f } };
        }

        Mount::Mount()
            : _viewTransform(m44f_identity) {}

        void Mount::setViewTransform(quatf const& q, float r)
        {
            v3f qx = qxdir(q);
            v3f qy = qydir(q);
            v3f qz = qzdir(q);

            // nb m44f constructor takes rows, not columns
            // -r will land in the 14th spot as required.
            m44f rm{ qx.x, qy.x, qz.x, 0.f,
                      qx.y, qy.y, qz.y, 0.f,
                      qx.z, qy.z, qz.z, -r,
                      0.f,  0.f,  0.f, 1.f };
            setViewTransform(rm);
        }


        v3f Mount::right() const {
            return normalize(
                v3f{ _viewTransform[0].x,
                     _viewTransform[1].x,
                     _viewTransform[2].x });
        }
        v3f Mount::up() const {
            return normalize(
                v3f{ _viewTransform[0].y,
                     _viewTransform[1].y,
                     _viewTransform[2].y });
        }
        v3f Mount::forward() const {
            return normalize(
                v3f{ _viewTransform[0].z,
                     _viewTransform[1].z,
                     _viewTransform[2].z });
        }
        v3f Mount::position() const {
            return v3f{ _viewTransform[0].w,
                        _viewTransform[1].w,
                        _viewTransform[2].w };
        }


        void Mount::lookat(v3f eye, v3f target, v3f up)
        {
            _viewTransform = make_lookat_transform(eye, target, up);
        }


        m44f perspective(const Sensor& sensor, const Optics& optics, float aspect)
        {
            if (fabs(aspect) < std::numeric_limits<float>::epsilon())
                return m44f_identity;

            const float handedness = sensor.handedness; // -1 for left hand coordinates
            float left = -1.f, right = 1.f, bottom = -1.f, top = 1.f;
            const float halfFovy = verticalFOV(sensor, optics) * 0.5f;
            const float y = 1.f / tanf(halfFovy);
            const float x = y / aspect / optics.squeeze;
            const float scalex = 2.f * sensor.enlarge.x;
            const float scaley = 2.f * sensor.enlarge.y;
            const float dx = sensor.shift.x * 2.f * aspect / sensor.aperture_y; // 0.f is sensor shift x
            const float dy = sensor.shift.y * 2.f / sensor.aperture_y;          // 0.f is sensor shift y

            const float znear = optics.znear;
            const float zfar = optics.zfar;

            m44f result;
            memset(&result, 0, sizeof(m44f));
            result[0].x = scalex * x / (right - left);
            result[1].y = scaley * y / (top - bottom);
            result[2].x = (right + left + dx) / (right - left);
            result[2].y = (top + bottom + dy) / (top - bottom);
            result[2].z = handedness * (zfar + znear) / (zfar - znear);
            result[2].w = handedness;
            result[3].z = handedness * 2.f * zfar * znear / (zfar - znear);
            return result;
        }

        Radians verticalFOV(const Sensor& sensor, const Optics& optics) {
            return 2.f * std::atanf(sensor.aperture_y / (2.f * optics.focalLength) / optics.squeeze);
        }
        MM focal_length_from_FOV(MM sensor_aperture, Radians fov) {
            return tanf(fov * 0.5f) / (sensor_aperture * 0.5f);
        }

        m44f perspective(const Sensor& sensor, const Optics& optics)
        {
            return perspective(sensor, optics, 1.f);
        }

        void Camera::frame(v3f bound1, v3f bound2)
        {
            float r = 0.5f * length(bound2 - bound1);
            float g = (1.1f * r) / sinf(verticalFOV(sensor, optics) * 0.5f);
            focusPoint = (bound2 + bound1) * 0.5f;
            position = normalize(position - focusPoint) * g;
            updateViewTransform();
        }

        void Camera::autoSetClippingPlanes(v3f bound1, v3f bound2)
        {
            float clipNear = FLT_MAX;
            float clipFar = FLT_MIN;

            v4f points[8] = {
                {bound1.x, bound1.y, bound1.z, 1.f},
                {bound1.x, bound1.y, bound2.z, 1.f},
                {bound1.x, bound2.y, bound1.z, 1.f},
                {bound1.x, bound2.y, bound2.z, 1.f},
                {bound2.x, bound1.y, bound1.z, 1.f},
                {bound2.x, bound1.y, bound2.z, 1.f},
                {bound2.x, bound2.y, bound1.z, 1.f},
                {bound2.x, bound2.y, bound2.z, 1.f} };

            for (int p = 0; p < 8; ++p) {
                v4f dp = mul(mount.viewTransform(), points[p]);
                clipNear = std::min(dp.z, clipNear);
                clipFar = std::max(dp.z, clipFar);
            }

            clipNear -= 0.5f;
            clipFar += 0.5f;
            clipNear = std::max(0.1f, std::min(clipNear, 100000.f));
            clipFar = std::max(clipNear, std::min(clipNear, 100000.f));

            if (clipFar <= clipNear)
                clipFar = clipNear + 0.1f;

            optics.znear = clipNear;
            optics.zfar = clipFar;
        }
    
        float Camera::planeIntersect(v3f planePoint, v3f planeNormal)
        {
            float denom = dot(planeNormal, mount.forward());
            if (denom > 1.e-6f) {
                v3f p0 = planePoint - mount.position();
                return dot(p0, planeNormal) / denom;
            }
            return FLT_MAX; // ray and plane are parallel
        }

        // delta is the 2d motion of a mouse or gesture in the screen plane,
        // typically computed as scale * (currMousePos - prevMousePos);
        //
        void cameraRig_interact(Camera& camera, CameraRigMode mode, v2f delta)
        {
            v3f cameraToFocus = camera.position - camera.focusPoint;
            float distanceToFocus = length(cameraToFocus);
            const float feel = 0.02f;
            float scale = std::max(0.01f, logf(distanceToFocus) * feel);

            switch (mode)
            {
            case CameraRigMode::Dolly:
            {
                v3f camFwd = camera.mount.forward();
                v3f camRight = camera.mount.right();
                v3f deltaX = camRight * delta.x * scale;
                v3f dP = camFwd * delta.y * scale - deltaX;
                camera.position += dP;
                camera.focusPoint += dP;
                break;
            }
            case CameraRigMode::Crane:
            {
                v3f camUp = camera.mount.up();
                v3f camRight = camera.mount.right();
                v3f dP = camUp * -delta.y * scale - camRight * delta.x * scale;
                camera.position += dP;
                camera.focusPoint += dP;
                break;
            }
            case CameraRigMode::TurnTableOrbit:
            {
                v3f camFwd = camera.mount.forward();
                v3f worldUp = camera.worldUp;
                v3f mUv = normalize(cross(worldUp, camFwd));
                quatf yaw = quat_fromAxisAngle(v3f{ 0.f, 1.f, 0.f }, -feel * delta.x);
                quatf pitch = quat_fromAxisAngle(mUv, feel * 0.25f * delta.y);
                v3f rotatedVec = quat_rotateVector(yaw, quat_rotateVector(pitch, cameraToFocus));
                v3f test = normalize(rotatedVec);
                // disallow going over the poles
                if (std::abs(dot(worldUp, test)) < 0.99f)
                    camera.position = camera.focusPoint + rotatedVec;
                break;
            }
            case CameraRigMode::Fly:
                /// @TODO
                break;
            }
            camera.updateViewTransform();
        }

    }
} // lab::camera

#endif
