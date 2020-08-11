#include "LabCamera.h"

#include <algorithm>
#include <cmath>

namespace lab {
    namespace camera {

        m44f m44f_identity = { 1.f,0.f,0.f,0.f, 0.f,1.f,0.f,0.f, 0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f };

        const float pi = 3.14159265359f;

        // Support for 3D spatial rotations using quaternions, via qmul(qmul(q, v), qconj(q))
        constexpr v3f qxdir(const quatf& q) { return { q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, (q.x * q.y + q.z * q.w) * 2, (q.z * q.x - q.y * q.w) * 2 }; }
        constexpr v3f qydir(const quatf& q) { return { (q.x * q.y - q.z * q.w) * 2, q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, (q.y * q.z + q.x * q.w) * 2 }; }
        constexpr v3f qzdir(const quatf& q) { return { (q.z * q.x + q.y * q.w) * 2, (q.y * q.z - q.x * q.w) * 2, q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z }; }
        constexpr v3f xyz(const v4f& a) { return { a.x, a.y, a.z }; }
        constexpr v3f cross(const v3f& a, const v3f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
        constexpr float dot(const v3f& a, const v3f& b) { return { a.x * b.x + a.y * b.y + a.z * b.z }; }
        constexpr v2f operator + (const v2f& a, const v2f& b) { return v2f{ a.x + b.x, a.y + b.y }; }
        constexpr v2f operator - (const v2f& a, const v2f& b) { return v2f{ a.x - b.x, a.y - b.y }; }
        constexpr v3f operator + (const v3f& a, const v3f& b) { return v3f{ a.x + b.x, a.y + b.y, a.z + b.z }; }
        constexpr v3f operator - (const v3f& a, const v3f& b) { return v3f{ a.x - b.x, a.y - b.y, a.z - b.z }; }
        constexpr v3f operator * (const v3f& a, float b) { return v3f{ a.x * b, a.y * b, a.z * b }; }
        constexpr v3f operator / (const v3f& a, float b) { return v3f{ a * (1.f / b) }; }
        constexpr v4f operator + (const v4f& a, const v4f& b) { return v4f{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
        constexpr v4f operator * (const v4f& a, float b) { return v4f{ a.x * b, a.y * b, a.z * b, a.w * b }; }
        constexpr v2f mul(const v2f& a, float b) { return { a.x * b, a.y * b }; }
        constexpr v3f mul(const v3f& a, float b) { return { a.x * b, a.y * b, a.z * b }; }
        constexpr v4f mul(const v4f& a, float b) { return { a.x * b, a.y * b, a.z * b, a.w * b }; }
        constexpr v3f mul(const m44f& a, const v3f& b) { return xyz(a.x) * b.x + xyz(a.y) * b.y + xyz(a.z) * b.z; }
        constexpr v4f mul(const m44f& a, const v4f& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
        constexpr quatf mul(const quatf& a, const quatf& b) { return { a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y, a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z, a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z }; }
        constexpr m44f mul(const m44f& a, const m44f& b) { return { mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w) }; }
        float length(const v3f& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
        constexpr v3f normalize(const v3f& a) { return a * (1.f / length(a)); }
        constexpr v3f& operator += (v3f& a, const v3f& b) { return a = a + b; }

        inline quatf quat_from_axis_angle(v3f v, float a)
        {
            quatf Result;
            float s = std::sin(a * 0.5f);
            Result.w = std::cos(a * 0.5f);
            Result.x = v.x * s;
            Result.y = v.y * s;
            Result.z = v.z * s;
            return Result;
        }

        inline v3f euler_from_quat(quatf q)
        {
            v3f rpy;
            const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
            rpy.x = float(atan2(2. * q2 * q3 + 2. * q0 * q1, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0));
            rpy.y = float(-asin(2. * q1 * q3 - 2. * q0 * q2));
            rpy.z = float(atan2(2. * q1 * q2 + 2. * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2));
            return rpy;
        }
        
        // adapted from IMath


        inline quatf quat_set_rotation_internal(v3f const& f0, v3f const& t0)
        {
            v3f h0 = normalize(f0 + t0);
            v3f v = cross(f0, h0);
            return { v.x, v.y, v.z, dot(f0, h0) };
        }


        inline quatf quat_from_vector_to_vector(v3f const& from, v3f const& to)
        {
            //
            // Create a quaternion that rotates vector from into vector to,
            // such that the rotation is around an axis that is the cross
            // product of from and to.
            //
            // This function calls function setRotationInternal(), which is
            // numerically accurate only for rotation angles that are not much
            // greater than pi/2.  In order to achieve good accuracy for angles
            // greater than pi/2, we split large angles in half, and rotate in
            // two steps.
            //

            //
            // Normalize from and to, yielding f0 and t0.
            //
            v3f f0 = normalize(from);
            v3f t0 = normalize(to);
            float d = dot(f0, t0);
            if (d >= 1)
            {
                // vectors are the same, return an identity quaternion
                return { 0,0,0,1 };
            }
            else if (d <= -1)
            {
                // f0 and t0 point in exactly opposite directions.
                // Pick an arbitrary axis that is orthogonal to f0,
                // and rotate by pi.

                v3f f02 = { f0.x * f0.x, f0.y * f0.y, f0.z * f0.z };
                v3f v;

                if (f02.x <= f02.y && f02.x <= f02.z)
                    v = normalize(cross(f0, { 1,0,0 }));
                else if (f02.y <= f02.z)
                    v = normalize(cross(f0, { 0,1,0 }));
                else
                    v = normalize(cross(f0, { 0,0,1 }));

                return { v.x, v.y, v.z, 0 };
            }

            if (d >= 0)
            {
                // The rotation angle is less than or equal to pi/2.
                return quat_set_rotation_internal(f0, t0);
            }

            //
            // The angle is greater than pi/2.  After computing h0,
            // which is halfway between f0 and t0, we rotate first
            // from f0 to h0, then from h0 to t0.
            //

            v3f h0 = normalize(f0 + t0);
            quatf q = quat_set_rotation_internal(f0, h0);
            return mul(q, quat_set_rotation_internal(h0, t0));
        }


        inline v3f quat_rotate_vector(quatf q, const v3f& v)
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
            return m44f{ v4f{ xaxis.x, yaxis.x, zaxis.x, 0.f },
                         v4f{ xaxis.y, yaxis.y, zaxis.y, 0.f },
                         v4f{ xaxis.z, yaxis.z, zaxis.z, 0.f },
                         v4f{ -dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1.f } };
        }

        // from linalg
        m44f adjugate(const m44f& a)
        {
            return m44f{ 
                v4f{ a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w - a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
                     a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w + a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w - a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
                     a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w + a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w - a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
                     a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w + a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w - a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w },
                v4f{ a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x,
                     a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x + a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x - a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
                     a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x + a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x - a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
                     a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x + a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x - a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x },
                v4f{ a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y,
                     a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y + a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y - a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
                     a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y + a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y - a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
                     a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y + a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y - a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y },
                v4f{ a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z,
                     a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z + a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z - a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
                     a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z + a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z - a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
                     a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z + a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z - a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z } };
        }

        float determinant(m44f const& a)
        {
            return a.x.x * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w + a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w - a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w)
                + a.x.y * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x)
                + a.x.z * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y)
                + a.x.w * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z);
        }

        m44f transpose(m44f const& a)
        {
            return {
                a.x.x, a.y.x, a.z.x, a.w.x,
                a.x.y, a.y.y, a.z.y, a.w.y,
                a.x.z, a.y.z, a.z.z, a.w.z,
                a.x.w, a.y.w, a.z.w, a.w.w
            };
        }

        m44f invert(m44f const& mat)
        {
            m44f m = adjugate(mat);
            float oo_det = 1.f / determinant(mat);
            float* ptr = &m.x.x;
            for (int i = 0; i < 16; i++)
                ptr[i] *= oo_det;
            return m;
        }

        m44f rotation(v3f e)
        {
            float cos_yaw = cosf(e.x);
            float cos_pitch = cosf(e.y);
            float cos_roll = cosf(e.z);

            float sin_yaw = sinf(e.x);
            float sin_pitch = sinf(e.y);
            float sin_roll = sinf(e.z);

            m44f m;
            m[0].x = cos_roll * cos_yaw;
            m[0].y = sin_roll * cos_yaw;
            m[0].z = -sin_yaw;
            m[0].w = 0;

            m[1].x = -sin_roll * cos_pitch + cos_roll * sin_yaw * sin_pitch;
            m[1].y = cos_roll * cos_pitch + sin_roll * sin_yaw * sin_pitch;
            m[1].z = cos_yaw * sin_pitch;
            m[1].w = 0;

            m[2].x = sin_roll * sin_pitch + cos_roll * sin_yaw * cos_pitch;
            m[2].y = -cos_roll * sin_pitch + sin_roll * sin_yaw * cos_pitch;
            m[2].z = cos_yaw * cos_pitch;
            m[2].w = 0;

            m[3] = { 0,0,0,1 };
            return m;
        }

        static bool intersect_ray_plane(const lab::camera::Ray& ray, const lab::camera::v3f& point, const lab::camera::v3f& normal,
            lab::camera::v3f* intersection = nullptr, float* outT = nullptr)
        {
            const float PLANE_EPSILON = 0.001f;
            const float d = ray.dir.x * normal.x + ray.dir.y * normal.y + ray.dir.z * normal.z;

            // Make sure we're not parallel to the plane
            if (std::abs(d) > PLANE_EPSILON)
            {
                float w = normal.x * point.x + normal.y * point.y + normal.z * point.z;
                w = -w;

                float distance = ray.pos.x * normal.x + ray.pos.y * normal.y + ray.pos.z * normal.z + w;
                float t = -distance / d;

                if (t >= PLANE_EPSILON)
                {
                    if (outT) *outT = t;
                    if (intersection)
                    {
                        lab::camera::v3f result = ray.pos;
                        result.x += t * ray.dir.x;
                        result.y += t * ray.dir.y;
                        result.z += t * ray.dir.z;
                        *intersection = result;
                    }
                    return true;
                }
            }
            if (outT) *outT = std::numeric_limits<float>::max();
            return false;
        }


//-----------------------------------------------------------------------------

        Mount::Mount()
            : _view_transform(m44f_identity) {}

        m44f Mount::inv_view_transform() const
        {
            return invert(_view_transform);
        }

        m44f Mount::inv_rotation_transform() const
        {
            return transpose(rotation_transform());
        }

        void Mount::set_view_transform(quatf const& v, v3f const& eye)
        {
            v3f xaxis = {
                1 - 2 * (v.y * v.y + v.z * v.z),
                2 * (v.x * v.y + v.z * v.w),
                2 * (v.z * v.x - v.y * v.w),
            };
            v3f yaxis = {
                2 * (v.x * v.y - v.z * v.w),
                1 - 2 * (v.z * v.z + v.x * v.x),
                2 * (v.y * v.z + v.x * v.w),
            };
            v3f zaxis = {
                2 * (v.z * v.x + v.y * v.w),
                2 * (v.y * v.z - v.x * v.w),
                1 - 2 * (v.y * v.y + v.x * v.x),
            };

            _view_transform = { 
                v4f{ xaxis.x, yaxis.x, zaxis.x, 0.f },
                v4f{ xaxis.y, yaxis.y, zaxis.y, 0.f },
                v4f{ xaxis.z, yaxis.z, zaxis.z, 0.f },
                v4f{ -dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1.f } };
        }

        void Mount::set_view_transform(v3f const& ypr, v3f const& eye)
        {
            _view_transform = transpose(lab::camera::rotation(ypr));
            m44f const& m = _view_transform;
            _view_transform.w.x = -dot({ m.x.x, m.y.x, m.z.x }, eye);
            _view_transform.w.y = -dot({ m.x.y, m.y.y, m.z.y }, eye);
            _view_transform.w.z = -dot({ m.x.z, m.y.z, m.z.z }, eye);
        }

        constexpr v3f Mount::right() const {
            return normalize(
                v3f{ _view_transform[0].x,
                     _view_transform[1].x,
                     _view_transform[2].x });
        }
        constexpr v3f Mount::up() const {
            return normalize(
                v3f{ _view_transform[0].y,
                     _view_transform[1].y,
                     _view_transform[2].y });
        }
        constexpr v3f Mount::forward() const {
            return normalize(
                v3f{ _view_transform[0].z,
                     _view_transform[1].z,
                     _view_transform[2].z });
        }
        constexpr v3f Mount::position() const {
            return v3f{ _view_transform[3].x,
                        _view_transform[3].y,
                        _view_transform[3].z };
        }


        void Mount::look_at(v3f const& eye, v3f const& target, v3f const& up)
        {
            _view_transform = make_lookat_transform(eye, target, up);
        }

        quatf Mount::rotation() const
        {
            return quat_from_matrix(_view_transform);
        }

        m44f perspective(const Sensor& sensor, const Optics& optics, float aspect)
        {
            if (fabs(aspect) < std::numeric_limits<float>::epsilon())
                return m44f_identity;

            const float handedness = sensor.handedness; // -1 for left hand coordinates
            float left = -1.f, right = 1.f, bottom = -1.f, top = 1.f;
            const float halfFovy = vertical_FOV(sensor, optics).value * 0.5f;
            const float y = 1.f / tanf(halfFovy);
            const float x = y / aspect / optics.squeeze;
            const float scalex = 2.f * sensor.enlarge.x;
            const float scaley = 2.f * sensor.enlarge.y;
            const float dx = sensor.shift.x.value * 2.f * aspect / sensor.aperture_y.value;
            const float dy = sensor.shift.y.value * 2.f / sensor.aperture_y.value;

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

        radians vertical_FOV(const Sensor& sensor, const Optics& optics) 
        {
            float cropped_f = optics.focal_length.value * sensor.enlarge.y;
            return { 2.f * std::atanf(sensor.aperture_y.value / (2.f * cropped_f)) };
        }

        millimeters Sensor::focal_length_from_FOV(radians fov)
        {
            if (fov.value < 0 || fov.value > 3.141592653589793238f)
                return millimeters{ 0.f };

            float f = (aperture_y.value / (2 * tanf(0.5f * fov.value)));
            return { f / enlarge.y };
        }

        Camera::Camera() 
        {
            mount.look_at(position, focus_point, world_up);
        }

        void Camera::frame(v3f const& bound1, v3f const& bound2)
        {
            float r = 0.5f * length(bound2 - bound1);
            float g = (1.1f * r) / sinf(vertical_FOV(sensor, optics).value * 0.5f);
            focus_point = (bound2 + bound1) * 0.5f;
            position = normalize(position - focus_point) * g;
            mount.look_at(position, focus_point, world_up);
        }

        void Camera::set_clipping_planes_within_bounds(v3f const& bound1, v3f const& bound2)
        {
            float clip_near = FLT_MAX;
            float clip_far = FLT_MIN;

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
                v4f dp = mul(mount.view_transform(), points[p]);
                clip_near = std::min(dp.z, clip_near);
                clip_far = std::max(dp.z, clip_far);
            }

            clip_near -= 0.5f;
            clip_far += 0.5f;
            clip_near = std::max(0.1f, std::min(clip_near, 100000.f));
            clip_far = std::max(clip_near, std::min(clip_near, 100000.f));

            if (clip_far <= clip_near)
                clip_far = clip_near + 0.1f;

            optics.znear = clip_near;
            optics.zfar = clip_far;
        }

        float Camera::distance_to_plane(v3f const& plane_point, v3f const& plane_normal)
        {
            float denom = dot(plane_normal, mount.forward());
            if (denom > 1.e-6f) {
                v3f p0 = plane_point - mount.position();
                return dot(p0, plane_normal) / denom;
            }
            return FLT_MAX; // ray and plane are parallel
        }

        m44f Camera::view_projection(float aspect) const
        {
            m44f proj = perspective(sensor, optics, aspect);
            m44f view = mount.view_transform();
            return mul(proj, view);
        }

        m44f Camera::inv_view_projection(float aspect) const
        {
            m44f proj = perspective(sensor, optics, aspect);
            m44f view = mount.view_transform();
            return invert(mul(proj, view));
        }

        Ray Camera::get_ray_from_pixel(v2f const& pixel, v2f const& viewport_origin, v2f const& viewport_size) const
        {
            // 3d normalized device coordinates
            const float x = 2 * (pixel.x - viewport_origin.x) / viewport_size.x - 1;
            const float y = 1 - 2 * (pixel.y - viewport_origin.y) / viewport_size.y;

            // eye coordinates
            m44f inv_projection = inv_view_projection(1.f);
            v4f p0 = mul(inv_projection, v4f{ x, y, -1, 1 });
            v4f p1 = mul(inv_projection, v4f{ x, y, +1, 1 });

            p1 = mul(p1, 1.f / p1.w);
            p0 = mul(p0, 1.f / p0.w);
            return{ position, normalize({ p1.x - p0.x, p1.y - p0.y, p1.z - p0.z }) };
        }

        HitResult Camera::hit_test(const v2f& mouse, const v2f& viewport, const v3f& plane_point, const v3f& plane_normal)
        {
            lab::camera::Ray ray = get_ray_from_pixel(mouse, { 0, 0 }, viewport);
            HitResult r;
            r.hit = intersect_ray_plane(ray, plane_point, plane_normal, &r.point);
            return r;
        }


        bool Camera::check_constraints(CameraRigMode mode)
        {
            m44f t = mount.rotation_transform();
            float declination = atan2(t[2].y, t[2].z);
            float azimuth = _azimuth;
            if (fabsf(declination) < pi * 0.5f)
            {
                v3f forward = mount.forward();
                forward.y = 0;
                forward = normalize(forward);
                float azimuth = atan2f(forward.x, forward.z);
                if (azimuth < 0)
                    azimuth += 2.f * pi;
            }
            return declination == _declination && azimuth == _azimuth;
        }

        void Camera::update_constraints(CameraRigMode mode)
        {
            m44f t = mount.rotation_transform();
            _declination = atan2(t[2].y, t[2].z);

            if (fabsf(_declination) < pi * 0.5f)
            {
                v3f forward = mount.forward();
                forward.y = 0;
                forward = normalize(forward);
                _azimuth = atan2f(forward.x, forward.z);
                if (_azimuth < 0)
                    _azimuth += 2.f * pi;
            }
        }

        // delta is the 2d motion of a mouse or gesture in the screen plane,
        // typically computed as scale * (currMousePos - prevMousePos);
        //
        void Camera::rig_interact(CameraRigMode mode, v2f const& delta_in)
        {
            _previous_rig_mode = mode;

            v2f delta = delta_in;

            const float buffer = 4.f;

            if (fabsf(delta.x) < buffer)
            {
                float dx = fabsf(delta.x) / buffer;
                dx *= dx;
                delta.x = buffer * copysign(dx, delta.x);
            }
            if (fabsf(delta.y) < buffer)
            {
                float dy = fabsf(delta.y) / buffer;
                dy *= dy;
                delta.y = buffer * copysign(dy, delta.y);
            }

            v3f camera_to_focus = position - focus_point;
            float distance_to_focus = length(camera_to_focus);
            const float feel = 0.02f;
            float scale = std::max(0.01f, logf(distance_to_focus) * feel);

            switch (mode)
            {
            case CameraRigMode::Dolly:
            {
                v3f camFwd = mount.forward();
                v3f camRight = mount.right();
                v3f deltaX = camRight * delta.x * scale;
                v3f dP = camFwd * delta.y * scale - deltaX;
                position += dP;
                focus_point += dP;
                mount.look_at(position, focus_point, world_up);
                break;
            }
            case CameraRigMode::Crane:
            {
                v3f camera_up = mount.up();
                v3f camera_right = mount.right();
                v3f dP = camera_up * -delta.y * scale - camera_right * delta.x * scale;
                position += dP;
                focus_point += dP;
                mount.look_at(position, focus_point, world_up);
                break;
            }
            case CameraRigMode::Gimbal:
            case CameraRigMode::TurnTableOrbit:
            {
                if (mode == CameraRigMode::Gimbal)
                    delta.y *= -1.f;   // to feel like the camera is moving in the gesture direction
                else
                    delta.x *= -1.f;   // to feel like the object is moving in the gesture direction

                _azimuth += 0.01f * delta.x;
                while (_azimuth > 2.f * pi)
                    _azimuth -= 2.f * pi;
                while (_azimuth < 0)
                    _azimuth += 2.f * pi;

                _declination += 0.002f * delta.y;
                while (_declination > pi * 0.5f)
                    _declination = pi * 0.5f;
                while (_declination < -pi * 0.5f)
                    _declination = -pi * 0.5f;

                v3f ypr{ _azimuth, _declination, 0 };
                m44f rot = lab::camera::rotation(ypr);
                if (mode == CameraRigMode::TurnTableOrbit)
                {
                    position = mul(rot, v3f{ 0, 0, distance_to_focus }) + focus_point;
                }
                else
                {
                    focus_point = position - mul(rot, v3f{ 0, 0, distance_to_focus });
                }
                mount.set_view_transform(ypr, position);
                break;
            }
            default:
            {
                // a tumble that moves the focus_point
                v3f camera_forward = mount.forward();
                v3f right = normalize(cross(world_up, camera_forward));

                v3f rel = mul(camera_to_focus, -1.f);
                quatf yaw = quat_from_axis_angle(v3f{ 0.f, 1.f, 0.f }, feel * 0.5f * delta.x);
                quatf pitch = quat_from_axis_angle(right, feel * -0.125f * delta.y);
                v3f rotatedVec = quat_rotate_vector(yaw, quat_rotate_vector(pitch, rel));
                focus_point = position + rotatedVec;
                mount.look_at(position, focus_point, world_up);
                update_constraints(mode);
                break;
            }
            }
        }

        // Initial is the screen position of the beginning of the interaction, current is the
        // current position
        //
        void Camera::rig_interact(CameraRigMode mode,
            v2f const& viewport_size,
            Camera const& initial_camera,
            v2f const& initial, v2f const& current)
        {
            _previous_rig_mode = mode;

            switch (mode)
            {
            case CameraRigMode::Crane:
            case CameraRigMode::Dolly:
            case CameraRigMode::TurnTableOrbit:
            {
                v2f dp = current - initial;
                dp.x *= -10.f / viewport_size.x;
                dp.y *= -10.f / viewport_size.y;
                rig_interact(mode, dp);
                break;
            }
            case CameraRigMode::Gimbal:
            {
                Ray original_ray = initial_camera.get_ray_from_pixel(initial, { 0, 0 }, viewport_size);
                Ray new_ray = initial_camera.get_ray_from_pixel(current, { 0, 0 }, viewport_size);
                quatf rotation = quat_from_vector_to_vector(new_ray.dir, original_ray.dir); // rotate in opposite direction
                v3f rel = initial_camera.focus_point - initial_camera.position;
                rel = quat_rotate_vector(rotation, rel);
                focus_point = position + rel;
                mount.look_at(position, focus_point, world_up);
                update_constraints(mode);
                break;
            }
            }
        }

        void Camera::rig_interact(CameraRigMode mode,
            v2f const& viewport_size,
            Camera const& initial_camera,
            v2f const& initial, v2f const& current,
            v3f const& initial_hit_point)
        {
            _previous_rig_mode = mode;

            switch (mode)
            {
            case CameraRigMode::Crane:
            {
                // calculate intersect at one unit distance view plane
                lab::camera::Ray mouse_dir = get_ray_from_pixel(current, { 0, 0 }, viewport_size);
                lab::camera::Ray forward = get_ray_from_pixel({ viewport_size.x * 0.5f, viewport_size.y * 0.5f }, { 0, 0 }, viewport_size);
                lab::camera::v3f center_of_image_plane = mount.position();
                lab::camera::v4f c2 = mount.rotation_transform().z;
                center_of_image_plane.x += forward.dir.x;
                center_of_image_plane.y += forward.dir.y;
                center_of_image_plane.z += forward.dir.z;
                lab::camera::v3f mouse_on_image_plane;
                intersect_ray_plane(mouse_dir, center_of_image_plane, forward.dir, &mouse_on_image_plane);

                v3f hit_to_mouse = normalize(initial_hit_point - mouse_on_image_plane);
                v3f new_camera_pos;

                if (intersect_ray_plane(Ray{ initial_hit_point, hit_to_mouse }, position, mount.forward(), &new_camera_pos))
                {
                    v3f delta = new_camera_pos - position;
                    focus_point += delta;
                    position = new_camera_pos;

                    printf("%f %f %f/%f %f %f\n", position.x, position.y, position.z, new_camera_pos.x, new_camera_pos.y, new_camera_pos.z);

                    mount.look_at(position, focus_point, world_up);
                    update_constraints(mode);
                }
                else
                {
                    printf("miss %f\n", c2.x);
                }
                break;
            }

            default:
                rig_interact(mode, viewport_size, initial_camera, initial, current);
                break;
            }
        }


    } // camera
} // lab

