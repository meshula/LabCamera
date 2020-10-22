#include "LabCamera.h"

#include <algorithm>
#include <cmath>
#include <cfloat>

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
        constexpr float dot(const v3f& a, const v3f& b) { return  a.x * b.x + a.y * b.y + a.z * b.z; }
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
        v3f normalize(const v3f& a) { return a * (1.f / length(a)); }
        float length(const quatf& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w); }
        quatf normalize(const quatf& a) { float l = 1.f / length(a); return { a.x * l, a.y * l, a.z * l, a.w * l }; }
        constexpr v3f& operator += (v3f& a, const v3f& b) { return a = a + b; }

        m44f rotz(float r)
        {
            float c = cosf(r);
            float s = sinf(r);
            return { c,-s, 0, 0,
                      s, c, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1 };
        }

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

        inline v3f euler_from_quat(const quatf& q)
        {
            v3f ypr;
            const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
            ypr.x = float(asin(2. * q1 * q3 - 2. * q0 * q2));
            ypr.y = float(atan2(2. * q2 * q3 + 2. * q0 * q1, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0));
            ypr.z = float(atan2(2. * q1 * q2 + 2. * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2));
            return ypr;
        }

        // swing twist decomposition.
        // cf. https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis/4341489
        // assumes dir is normailzed
        inline float angle_about_dir(const quatf& q, const v3f& dir)
        {
            v3f axis = { q.x, q.y, q.z };
            float dot_product = dot(dir, axis);
            v3f projection = mul(dir, dot_product);
            quatf twist = normalize(quatf{ projection.x, projection.y, projection.z, q.w });

            if (dot_product < 0.0) {
                // Ensure `twist` points towards `direction`
                // (if calculating the new axis, then all of twist needs negating)
                twist.w = -twist.w;
                // Rotation angle `twist.angle()` is now reliable
            }
            return 2.f * acosf(twist.w); // if axis and angle is being computed, more work is necessary: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
        }

        inline quatf rotation_about_dir(const quatf& q, const v3f& dir)
        {
            v3f axis = { q.x, q.y, q.z };
            float dot_product = dot(dir, axis);
            v3f projection = mul(dir, dot_product);
            quatf twist = normalize(quatf{ projection.x, projection.y, projection.z, q.w });

            if (dot_product < 0.0) {
                // Ensure `twist` points towards `direction`
                twist = mul(twist, -1.f);
                // Rotation angle `twist.angle()` is now reliable
            }
            return twist;
        }

        // returns a ypr vector consistent with the ypr being used throughout this API
        // ypr is distinct from an Euler angle in that it is the specific pair of angles
        // of azimuth and declination, as opposed to an arbitrary but otherwise correct
        // Euler angle tuple.
        inline v3f ypr_from_quat(const quatf& q)
        {
            quatf yaw_rot = rotation_about_dir(q, v3f{ 0, 1, 0 });
            quatf pitch_rot = rotation_about_dir(q, v3f{ 1, 0, 0 });

            // get local roll about the composed yaw an pitch
            //quatf yaw_pitch = mul(yaw_rot, pitch_rot);
            //float n = 1.f / sqrtf(1.f - yaw_pitch.w * yaw_pitch.w);
            //quatf roll_rot = rotation_about_dir(q, v3f{ yaw_pitch.x * n, yaw_pitch.y * n, yaw_pitch.z * n });
            v3f ypr = {
                2.f * acosf(yaw_rot.w),
                2.f * acosf(pitch_rot.w),
                0 //2.f * acosf(roll_rot.w)
            };

            ypr.x = 2.f * pi - ypr.x;

            if (ypr.y > pi)
                ypr.y = 2.f * pi - ypr.y;
            else
                ypr.y *= -1.f;
            return ypr;
        }

        float distance_point_to_plane(v3f const& a, v3f const& point, v3f const& normal)
        {
            v3f d = point - a;
            return dot(d, normal);
        }
        
        // adapted from Imath


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

        m44f rotation_quat(quatf const& v)
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

            return {
                v4f{ xaxis.x, xaxis.y, xaxis.z, 0.f },
                v4f{ yaxis.x, yaxis.y, yaxis.z, 0.f },
                v4f{ zaxis.x, zaxis.y, zaxis.z, 0.f },
                v4f{ 0.f, 0.f, 0.f, 1.f } };
        }


#if 0

        m44f rotation_xyz(v3f e)
        {
            float cx = cosf(-e.y);
            float cy = cosf(-e.x);
            float cz = 1.f; // cosf(e.z);

            float sx = sinf(-e.y);
            float sy = sinf(-e.x);
            float sz = 0.f; //sinf(e.z);

            m44f m;
            
            // zyx
            m[0].x =  cy * cz;
            m[0].y = -cy * sz;
            m[0].z =  sy;
            m[0].w = 0;

            m[1].x =  sx * sy * cz + cx * sz;
            m[1].y = -sx * sy * sz + cx * cz;
            m[1].z = -sx * cy;
            m[1].w = 0;

            m[2].x = -cx * sy * cz + sx * sz;
            m[2].y =  cx * sy * sz + sx * cz;
            m[2].z =  cx * cy;
            m[2].w = 0;
            m[3] = { 0,0,0,1 };
            return m;
        }

#endif

        m44f rotation_xyz(v3f e)
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

        Ray get_ray(m44f const& inv_projection, v3f const& camera_position, v2f const& pixel, v2f const& viewport_origin, v2f const& viewport_size)
        {
            // 3d normalized device coordinates
            const float x = 2 * (pixel.x - viewport_origin.x) / viewport_size.x - 1;
            const float y = 1 - 2 * (pixel.y - viewport_origin.y) / viewport_size.y;

            // eye coordinates
            v4f p0 = mul(inv_projection, v4f{ x, y, -1, 1 });
            v4f p1 = mul(inv_projection, v4f{ x, y, +1, 1 });

            p1 = mul(p1, 1.f / p1.w);
            p0 = mul(p0, 1.f / p0.w);
            return{ camera_position, normalize(v3f { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z }) };
        }



//-----------------------------------------------------------------------------
// PanTiltController
//
//-----------------------------------------------------------------------------


        InteractionToken PanTiltController::begin_interaction(v2f const& viewport_size)
        {
            // nb: in the future the InteractionToken will be used to manage
            // multitouch and multidevice interactions.

            _viewport_size = viewport_size;
            return 0;
        }

        void PanTiltController::end_interaction(InteractionToken)
        {
        }

        // delta is the 2d motion of a mouse or gesture in the screen plane,
        // typically computed as scale * (currMousePos - prevMousePos);
        //
        void PanTiltController::joystick_interaction(Camera& camera, InteractionToken, InteractionPhase phase, InteractionMode mode, v2f const& delta_in)
        {
            if (0)
            {
                m44f m = camera.mount.view_transform();
                quatf q = quat_from_matrix(m);
                v3f e2 = ypr_from_quat(q);
                printf("%f %f %f\n", e2.x, e2.y, e2.z);
            }

            if (phase == InteractionPhase::Start)
            {
                _initial_inv_projection = camera.inv_view_projection(1.f);
                _initial_position_constraint = _position;
                _initial_focus_point = _focus_point;
            }

            // joystick mode controls
            v2f delta = delta_in;

            const float buffer = 4.f;

            // make control less sensitive within the buffer 
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

            v3f camera_to_focus = _position - _focus_point;
            float distance_to_focus = length(camera_to_focus);
            const float feel = 0.02f;
            float scale = std::max(0.01f, logf(distance_to_focus) * feel);

            switch (mode)
            {
            case InteractionMode::Dolly:
            {
                v3f camFwd = camera.mount.forward();
                v3f camRight = camera.mount.right();
                v3f deltaX = camRight * delta.x * scale;
                v3f dP = camFwd * delta.y * scale - deltaX;
                _position += dP;
                _focus_point += dP;
                camera.mount.look_at(_position, _focus_point, _world_up);
                break;
            }
            case InteractionMode::Crane:
            {
                v3f camera_up = camera.mount.up();
                v3f camera_right = camera.mount.right();
                v3f dP = camera_up * -delta.y * scale - camera_right * delta.x * scale;
                _position += dP;
                _focus_point += dP;
                camera.mount.look_at(_position, _focus_point, _world_up);
                break;
            }
            case InteractionMode::Gimbal:
            case InteractionMode::TurnTableOrbit:
            {
                if (mode == InteractionMode::Gimbal)
                    delta.y *= -1.f;   // to feel like the camera is moving in the gesture direction
                else
                    delta.x *= -1.f;   // to feel like the object is moving in the gesture direction

                v3f start_ypr = camera.mount.ypr();
                start_ypr.z = 0;

                // azimuth
                start_ypr.x += 0.01f * delta.x;
                while (start_ypr.x > 2.f * pi)
                    start_ypr.x -= 2.f * pi;
                while (start_ypr.x < 0)
                    start_ypr.x += 2.f * pi;

                start_ypr.y += 0.002f * delta.y;
                if (start_ypr.y > pi * 0.5f)
                    start_ypr.y = pi * 0.5f;
                if (start_ypr.y < -pi * 0.5f)
                    start_ypr.y = -pi * 0.5f;

                v3f ypr{ start_ypr.x, start_ypr.y, start_ypr.z };

                m44f rot = lab::camera::rotation_xyz(ypr);
                if (mode == InteractionMode::TurnTableOrbit)
                {
                    _position = mul(rot, v3f{ 0, 0, distance_to_focus }) + _focus_point;
                }
                else
                {
                    _focus_point = _position - mul(rot, v3f{ 0, 0, distance_to_focus });
                }
                camera.mount.set_view_transform_ypr_eye(ypr, _position);
                break;
            }
            default:
            {
                // a tumble that moves the focus_point
                v3f camera_forward = camera.mount.forward();
                v3f right = normalize(cross(_world_up, camera_forward));

                v3f rel = mul(camera_to_focus, -1.f);
                quatf yaw = quat_from_axis_angle(v3f{ 0.f, 1.f, 0.f }, feel * 0.5f * delta.x);
                quatf pitch = quat_from_axis_angle(right, feel * -0.125f * delta.y);
                v3f rotatedVec = quat_rotate_vector(yaw, quat_rotate_vector(pitch, rel));
                _focus_point = _position + rotatedVec;
                camera.mount.look_at(_position, _focus_point, _world_up);
                break;
            }
            }
        }


        // Initial is the screen position of the beginning of the interaction, current is the
        // current position
        //
        void PanTiltController::ttl_interaction(Camera& camera, InteractionToken tok, InteractionPhase phase, InteractionMode mode, v2f const& current)
        {
            switch (mode)
            {
            case InteractionMode::Crane:
            case InteractionMode::Dolly:
            case InteractionMode::TurnTableOrbit:
            {
                if (phase == InteractionPhase::Start)
                {
                    _init_mouse = current;
                }

                // Joystick mode
                v2f dp = current - _init_mouse;
                v2f prev_dp = current - _prev_mouse;

                // reset the anchor if the interaction direction changes on either axis.
                // this is to increase the feeling of responsiveness
                if (dp.x * prev_dp.x < 0)
                    _init_mouse.x = current.x;
                if (dp.y * prev_dp.y < 0)
                    _init_mouse.y = current.y;
                dp = current - _init_mouse;

                const float speed_scale = 10.f;
                dp.x *= speed_scale / _viewport_size.x;
                dp.y *= -speed_scale / _viewport_size.y;
                joystick_interaction(camera, tok, phase, mode, dp);
                break;
            }
            case InteractionMode::Gimbal:
            {
                if (phase == InteractionPhase::Start)
                {
                    _init_mouse = current;
                    _initial_inv_projection = camera.inv_view_projection(1.f);
                    _initial_position_constraint = _position;
                    _initial_focus_point = _focus_point;
                }

                // Through the lens gimbal
                Ray original_ray = get_ray(_initial_inv_projection,
                    _initial_position_constraint, _init_mouse,
                    { 0, 0 }, _viewport_size);
                Ray new_ray = get_ray(_initial_inv_projection,
                    _position, current,
                    { 0, 0 }, _viewport_size);
                quatf rotation = quat_from_vector_to_vector(new_ray.dir, original_ray.dir); // rotate in opposite direction
                v3f rel = _initial_focus_point - _initial_position_constraint;
                rel = quat_rotate_vector(rotation, rel);
                _focus_point = _position + rel;
                camera.mount.look_at(_position, _focus_point, _world_up);
                break;
            }
            }

            _prev_mouse = current;
        }

        void PanTiltController::constrained_ttl_interaction(Camera& camera, InteractionToken tok,
            InteractionPhase phase, InteractionMode mode,
            v2f const& current,
            v3f const& initial_hit_point)
        {
            switch (mode)
            {
            case InteractionMode::Crane:
            {
                if (phase == InteractionPhase::Start)
                {
                    _initial_focus_point = initial_hit_point;
                }

                // Through the lens crane
                v2f target_xy = camera.project_to_viewport(v2f{ 0,0 }, _viewport_size, _initial_focus_point) - current;
                target_xy = mul(target_xy, 1.f / _viewport_size.x);
                v3f delta = mul(camera.mount.right(), target_xy.x * 1.f);
                delta += mul(camera.mount.up(), target_xy.y * -1.f);
                _position += delta;
                _focus_point += delta;
                camera.mount.set_view_transform_ypr_eye(camera.mount.ypr(), _position);
                break;
            }
            case InteractionMode::Dolly:
            {
                if (phase == InteractionPhase::Start)
                {
                    _initial_focus_point = initial_hit_point;
                }

                // Through the lens crane
                v2f target_xy = camera.project_to_viewport(v2f{ 0,0 }, _viewport_size, _initial_focus_point) - current;
                target_xy = mul(target_xy, 1.f / _viewport_size.x);
                v3f delta = mul(camera.mount.right(), target_xy.x * 1.f);
                v3f delta_fw = mul(camera.mount.forward(), target_xy.y * -1.f);

                // would moving forward by delta_fw push past the plane (focus_point, mount.forward())?
                v3f test_pos = _position + delta_fw;
                float dist = distance_point_to_plane(test_pos, _initial_focus_point, camera.mount.forward());
                if (dist < 0)
                {
                    _position += delta + delta_fw;
                    _focus_point += delta + delta_fw;
                    camera.mount.set_view_transform_ypr_eye(camera.mount.ypr(), _position);
                }
                break;
            }

            default:
                ttl_interaction(camera, tok, phase, mode, current);
                break;
            }
        }

        v3f PanTiltController::world_up_constraint() const
        {
            return _world_up;
        }

        v3f PanTiltController::position_constraint() const
        {
            return _position;
        }

        void PanTiltController::set_position_constraint(v3f const& pos)
        {
            _position = pos;
        }

        v3f PanTiltController::focus_constraint() const
        {
            return _focus_point;
        }

        void PanTiltController::set_focus_constraint(v3f const& pos)
        {
            _focus_point = pos;
        }

        void PanTiltController::set_world_up_constraint(v3f const& up)
        {
            _world_up = up;
        }



//-----------------------------------------------------------------------------
// Mount
//
//-----------------------------------------------------------------------------


        Mount::Mount()
        : _view_transform(m44f_identity)
        {
            look_at({ 0, 0.2f, 5 }, { 0,0,0 }, { 0,1,0 });
        }

        Mount::~Mount()
        {
        }

        m44f Mount::view_transform_inv() const
        {
            return invert(view_transform());
        }

        m44f Mount::model_view_transform(float const* const view_matrix) const
        {
            return mul(view_transform(), *(m44f*)view_matrix);
        }

        m44f Mount::model_view_transform(m44f const& view_matrix) const
        {
            return mul(view_transform(), view_matrix);
        }

        m44f Mount::inv_rotation_transform() const
        {
            return transpose(rotation_transform());
        }

        void Mount::set_view_transform_ypr_eye(quatf const& v, v3f const& eye)
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

        void Mount::set_view_transform_ypr_eye(v3f const& ypr, v3f const& eye)
        {
            v3f rot = { ypr.x, ypr.y, ypr.z };
            _view_transform = transpose(lab::camera::rotation_xyz(rot));
            m44f const& m = _view_transform;
            _view_transform.w.x = -dot({ m.x.x, m.y.x, m.z.x }, eye);
            _view_transform.w.y = -dot({ m.x.y, m.y.y, m.z.y }, eye);
            _view_transform.w.z = -dot({ m.x.z, m.y.z, m.z.z }, eye);
        }

        void Mount::set_view_transform_ypr_pos(v3f const& ypr, v3f const& pos)
        {
            v3f rot = { ypr.x, ypr.y, ypr.z };
            _view_transform = transpose(lab::camera::rotation_xyz(rot));
            _view_transform.w.x = pos.x;
            _view_transform.w.y = pos.y;
            _view_transform.w.z = pos.z;
        }

        v3f Mount::right() const {
            m44f m = view_transform();
            return normalize(
                v3f{ m[0].x,
                     m[1].x,
                     m[2].x });
        }
        v3f Mount::up() const {
            m44f m = view_transform();
            return normalize(
                v3f{ m[0].y,
                     m[1].y,
                     m[2].y });
        }
        v3f Mount::forward() const {
            m44f m = view_transform();
            return normalize(
                v3f{ m[0].z,
                     m[1].z,
                     m[2].z });
        }
        v3f Mount::position() const {
            m44f m = view_transform();
            v3f p = mul(xyz(m[3]), -1.f);
            m = inv_rotation_transform();
            p = mul(m, p);
            return p;
        }

        m44f Mount::view_transform() const 
        { 
            return _view_transform;
        }

        void Mount::look_at(v3f const& eye, v3f const& target, v3f const& up)
        {
            _view_transform = make_lookat_transform(eye, target, up);
        }

        quatf Mount::rotation() const
        {
            return quat_from_matrix(_view_transform);
        }

        v3f Mount::ypr() const
        {
            m44f t = rotation_transform();
            v3f ypr;

            v3f fwd = forward();
            fwd.y = 0;
            fwd = normalize(fwd);

            // yaw
            ypr.x = atan2f(fwd.x, fwd.z);
            while (ypr.x < 0)
                ypr.x += 2.f * pi;
            while (ypr.x > 2.f * pi)
                ypr.x -= 2.f * pi;

            // pitch
            ypr.y = atan2(t[2].y, t[2].z);
            if (ypr.y > 0.5f * pi)
            {
                ypr.y = -pi + ypr.y;
            }
            else if (ypr.y < -0.5f * pi)
            {
                ypr.y = ypr.y + pi;
            }

#if 0
            // roll
            // @TODO Still haven't deduced a way to compute roll
            // that gives an intuitive result
            quatf q = rotation();
            float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            ypr.z = atan2f(siny_cosp, cosy_cosp);
#else
            ypr.z = 0;
#endif
            return ypr;
        }



//-----------------------------------------------------------------------------
// Optics
//
//-----------------------------------------------------------------------------

        millimeters Optics::hyperfocal_distance(millimeters CoC)
        {
            return { focal_length.value * focal_length.value / (fStop * CoC.value) };
        }

        v2f Optics::focus_range(millimeters h)
        {
            v2f r;
            r.x = h.value * focus_distance.value / (h.value + (focus_distance.value - focal_length.value));
            r.y = h.value * focus_distance.value / (h.value - (focus_distance.value - focal_length.value));
            return r;
        }

//-----------------------------------------------------------------------------
// Sensor
//
//-----------------------------------------------------------------------------

        millimeters Sensor::focal_length_from_vertical_FOV(radians fov)
        {
            if (fov.value < 0 || fov.value > 3.141592653589793238f)
                return millimeters{ 0.f };

            float f = (aperture_y.value / (2 * tanf(0.5f * fov.value)));
            return { f / enlarge.y };
        }

//-----------------------------------------------------------------------------
// Camera
//
//-----------------------------------------------------------------------------

        Camera::Camera()
        {
        }

        Camera::~Camera()
        {
        }


        m44f Camera::perspective(float aspect) const
        {
            if (fabs(aspect) < std::numeric_limits<float>::epsilon())
                return m44f_identity;

            const float handedness = sensor.handedness; // -1 for left hand coordinates
            float left = -1.f, right = 1.f, bottom = -1.f, top = 1.f;
            const float halfFovy = vertical_FOV().value * 0.5f;
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

        m44f Camera::inv_perspective(float aspect) const
        {
            return invert(perspective(aspect));
        }

        radians Camera::vertical_FOV() const
        {
            float cropped_f = optics.focal_length.value * sensor.enlarge.y;
            return { 2.f * std::atanf(sensor.aperture_y.value / (2.f * cropped_f)) };
        }

        radians Camera::horizontal_FOV() const
        {
            float cropped_f = optics.focal_length.value * sensor.enlarge.y;
            return { 2.f * std::atanf(optics.squeeze * sensor.aperture_x.value / (2.f * cropped_f)) };
        }

        void Camera::frame(v3f const& bound1, v3f const& bound2)
        {
            float r = 0.5f * length(bound2 - bound1);
            float g = (1.1f * r) / sinf(vertical_FOV().value * 0.5f);
            v3f focus_point = (bound2 + bound1) * 0.5f;
            v3f position = normalize(mount.position() - focus_point) * g;
            mount.look_at(position, focus_point, mount.up());
        }

        void Camera::set_clipping_planes_within_bounds(float min_near, float max_far, v3f const& bound1, v3f const& bound2)
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

            for (int p = 0; p < 8; ++p)
            {
                v4f dp = mul(mount.view_transform(), points[p]);
                clip_near = std::min(dp.z, clip_near);
                clip_far = std::max(dp.z, clip_far);
            }

            clip_near = std::max(min_near, std::min(clip_near, max_far));
            clip_far = std::max(clip_near, std::min(clip_near, max_far));

            if (clip_far < clip_near)
            {
                float temp = clip_far;
                clip_far = clip_near;
                clip_near = clip_far;
            }

            optics.znear = clip_near;
            optics.zfar = clip_far;
        }

        float Camera::distance_to_plane(v3f const& plane_point, v3f const& plane_normal) const
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
            m44f proj = perspective(aspect);
            m44f view = mount.view_transform();
            return mul(proj, view);
        }

        m44f Camera::inv_view_projection(float aspect) const
        {
            m44f proj = perspective(aspect);
            m44f view = mount.view_transform();
            return invert(mul(proj, view));
        }

        Ray Camera::get_ray_from_pixel(v2f const& pixel, v2f const& viewport_origin, v2f const& viewport_size) const
        {
            m44f inv_projection = inv_view_projection(1.f);
            return get_ray(inv_projection, mount.position(), pixel, viewport_origin, viewport_size);
        }

        HitResult Camera::hit_test(const v2f& mouse, const v2f& viewport, const v3f& plane_point, const v3f& plane_normal) const
        {
            lab::camera::Ray ray = get_ray_from_pixel(mouse, { 0, 0 }, viewport);
            HitResult r;
            r.hit = intersect_ray_plane(ray, plane_point, plane_normal, &r.point);
            return r;
        }

        v2f Camera::project_to_viewport(v2f const& viewport_origin, v2f const& viewport_size, const v3f& point) const
        {
            m44f m = view_projection(1.f);

            v4f p = mul(m, v4f{ point.x, point.y, point.z, 1.f });
            v3f pnt = xyz(mul(p, 1.f / p.w));
            pnt.x = pnt.x * viewport_size.x * 0.5f + viewport_size.x * 0.5f;
            pnt.y = pnt.y * viewport_size.y * -0.5f + viewport_size.y * 0.5f;
            pnt.x -= viewport_origin.x;
            pnt.y -= viewport_origin.y;
            return { pnt.x, pnt.y };
        }

        v3f Camera::arcball_vector(v2f const& viewport_origin, v2f const& viewport_size, const v2f& point) const
        {
            /// @todo take origin into account

            v3f p{ 1.f * point.x / viewport_size.x * 2.f - 1.f,
                   1.f * point.y / viewport_size.y * 2.f - 1.f,
                   0.f };

            p.y = -p.y;

            float OP_squared = p.x * p.x + p.y * p.y;
            if (OP_squared <= 1)
                p.z = sqrtf(1 - OP_squared);  // Pythagoras
            else
                p = normalize(p);  // nearest point
            return p;
        }



    } // camera
} // lab

