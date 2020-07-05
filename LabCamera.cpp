#include "LabCamera.h"

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
        constexpr v4f mul(const v4f& a, float b) { return { a.x * b, a.y * b, a.z * b, a.w * b }; }
        constexpr v4f mul(const m44f& a, const v4f& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
        constexpr m44f mul(const m44f& a, const m44f& b) { return { mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w) }; }
        constexpr v3f cross(const v3f& a, const v3f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
        constexpr float dot(const v3f& a, const v3f& b) { return { a.x * b.x + a.y * b.y + a.z * b.z }; }
        float length(const v3f& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
        constexpr v3f normalize(const v3f& a) { return a * (1.f / length(a)); }
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

        quatf quat_from_matrix(const m44f& mat)
        {
            float s;
            float q[4];
            int i, j, k;
            quatf quat;

            int nxt[3] = { 1, 2, 0 };
            float tr = mat[0].x + mat[1].y + mat[2].z;

            // check the diagonal
            if (tr > 0.0) {
                s = sqrtf(tr + 1.f);
                quat.w = s / 2.f;
                s = 0.5f / s;

                quat.x = (mat[1].z - mat[2].y) * s;
                quat.y = (mat[2].x - mat[0].z) * s;
                quat.z = (mat[0].y - mat[1].x) * s;
            }
            else {
                // diagonal is negative
                i = 0;
                if (mat[1].y > mat[0].x)
                    i = 1;

                v4f const* const mat_i = &mat[i];
                float const* const f_i = reinterpret_cast<float const* const>(mat_i);
                if (mat[2].z > f_i[i])
                    i = 2;

                j = nxt[i];
                k = nxt[j];

                v4f const* const mat_j = &mat[j];
                float const* const f_j = reinterpret_cast<float const* const>(mat_j);
                v4f const* const mat_k = &mat[k];
                float const* const f_k = reinterpret_cast<float const* const>(mat_k);

                s = sqrtf((f_i[i] - (f_j[j] + f_k[k])) + 1.f);

                q[i] = s * 0.5f;
                if (s != 0.f)
                    s = 0.5f / s;

                q[3] = (f_j[k] - f_k[j]) * s;
                q[j] = (f_i[j] + f_j[i]) * s;
                q[k] = (f_i[k] + f_k[i]) * s;

                quat.x = q[0];
                quat.y = q[1];
                quat.z = q[2];
                quat.w = q[3];
            }

            return quat;
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

        // from linalg
        m44f adjugate(const m44f& a)
        {
            return{ { a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w - a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
                a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w + a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w - a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
                a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w + a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w - a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
                a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w + a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w - a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w },
                { a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x,
                a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x + a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x - a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
                a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x + a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x - a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
                a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x + a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x - a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x },
                { a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y,
                a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y + a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y - a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
                a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y + a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y - a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
                a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y + a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y - a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y },
                { a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z,
                a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z + a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z - a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
                a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z + a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z - a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
                a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z + a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z - a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z } };
        }

        float determinant(const m44f& a)
        {
            return a.x.x * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w - a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w)
                + a.x.y * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x)
                + a.x.z * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y)
                + a.x.w * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z);
        }

        m44f invert(const m44f& mat)
        {
            m44f m = adjugate(mat);
            float oo_det = 1.f / determinant(mat);
            float* ptr = &m.x.x;
            for (int i = 0; i < 16; i++)
                ptr[i] *= oo_det;
            return m;
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


        constexpr v3f Mount::right() const {
            return normalize(
                v3f{ _viewTransform[0].x,
                     _viewTransform[1].x,
                     _viewTransform[2].x });
        }
        constexpr v3f Mount::up() const {
            return normalize(
                v3f{ _viewTransform[0].y,
                     _viewTransform[1].y,
                     _viewTransform[2].y });
        }
        constexpr v3f Mount::forward() const {
            return normalize(
                v3f{ _viewTransform[0].z,
                     _viewTransform[1].z,
                     _viewTransform[2].z });
        }
        constexpr v3f Mount::position() const {
            return v3f{ _viewTransform[3].x,
                        _viewTransform[3].y,
                        _viewTransform[3].z };
        }


        void Mount::lookat(v3f eye, v3f target, v3f up)
        {
            _viewTransform = make_lookat_transform(eye, target, up);
        }

        quatf Mount::rotation() const
        {
            return quat_from_matrix(_viewTransform);
        }

        m44f perspective(const Sensor& sensor, const Optics& optics, float aspect)
        {
            if (fabs(aspect) < std::numeric_limits<float>::epsilon())
                return m44f_identity;

            const float handedness = sensor.handedness; // -1 for left hand coordinates
            float left = -1.f, right = 1.f, bottom = -1.f, top = 1.f;
            const float halfFovy = verticalFOV(sensor, optics).value * 0.5f;
            const float y = 1.f / tanf(halfFovy);
            const float x = y / aspect / optics.squeeze;
            const float scalex = 2.f * sensor.enlarge.x;
            const float scaley = 2.f * sensor.enlarge.y;
            const float dx = sensor.shift.x * 2.f * aspect / sensor.aperture_y.value; // 0.f is sensor shift x
            const float dy = sensor.shift.y * 2.f / sensor.aperture_y.value;          // 0.f is sensor shift y

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

        m44f inv_perspective(const Sensor& sensor, const Optics& optics, float aspect)
        {
            return invert(perspective(sensor, optics, aspect));
        }


        radians verticalFOV(const Sensor& sensor, const Optics& optics) {
            return { 2.f * std::atanf(sensor.aperture_y.value / (2.f * optics.focalLength.value) / optics.squeeze) };
        }

        millimeters focal_length_from_FOV(millimeters sensor_aperture, radians fov) {
            return { tanf(fov.value * 0.5f) / (sensor_aperture.value * 0.5f) };
        }

        void Camera::frame(v3f bound1, v3f bound2)
        {
            float r = 0.5f * length(bound2 - bound1);
            float g = (1.1f * r) / sinf(verticalFOV(sensor, optics).value * 0.5f);
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
                v3f right = normalize(cross(worldUp, camFwd));
                quatf yaw = quat_fromAxisAngle(v3f{ 0.f, 1.f, 0.f }, -feel * delta.x);
                quatf pitch = quat_fromAxisAngle(right, feel * 0.25f * delta.y);
                v3f rotatedVec = quat_rotateVector(yaw, quat_rotateVector(pitch, cameraToFocus));
                v3f test = normalize(rotatedVec);

                // disallow going over the poles
                if (std::abs(dot(worldUp, test)) < 0.99f)
                    camera.position = camera.focusPoint + rotatedVec;
                break;
            }
            case CameraRigMode::Gimbal:
            {
                v3f camFwd = camera.mount.forward();
                v3f worldUp = camera.worldUp;
                v3f right = normalize(cross(worldUp, camFwd));

                v3f rel = camera.focusPoint - camera.position;
                quatf yaw = quat_fromAxisAngle(v3f{ 0.f, 1.f, 0.f }, feel * delta.x);
                quatf pitch = quat_fromAxisAngle(right, feel * -0.25f * delta.y);
                v3f rotatedVec = quat_rotateVector(yaw, quat_rotateVector(pitch, rel));
                camera.focusPoint = camera.position + rotatedVec;
                break;
            }
            }
            camera.updateViewTransform();
        }

        m44f Camera::viewProj(float aspect) const
        {
            m44f proj = perspective(sensor, optics, aspect);
            m44f view = mount.viewTransform();
            return mul(proj, view);
        }

        m44f Camera::inv_viewProj(float aspect) const
        {
            m44f proj = perspective(sensor, optics, aspect);
            m44f view = mount.viewTransform();
            return invert(mul(proj, view));
        }

        Ray Camera::get_ray_from_pixel(v2f pixel, v2f viewport_origin, v2f viewport_size) const
        {
            const float x = 2 * (pixel.x - viewport_origin.x) / viewport_size.x - 1;
            const float y = 1 - 2 * (pixel.y - viewport_origin.y) / viewport_size.y;
            float aspect = viewport_size.x / viewport_size.y;
            m44f inv_projection = inv_viewProj(aspect);

            v4f p0 = mul(inv_projection, v4f{ x, y, -1, 1 });
            v4f p1 = mul(inv_projection, v4f{ x, y, +1, 1 });

            p1 = mul(p1, 1.f / p1.w);
            p0 = mul(p0, 1.f / p0.w);
            return{ position, normalize({ p1.x - p0.x, p1.y - p0.y, p1.z - p0.z }) };
        }

    } // camera
} // lab

