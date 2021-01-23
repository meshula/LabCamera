#include "LabCamera.h"

#include <algorithm>
#include <cmath>
#include <cfloat>

// an anonymous namespace to prevent symbol exposure
namespace {

    lc_m44f m44f_identity = { 1.f,0.f,0.f,0.f, 0.f,1.f,0.f,0.f, 0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f };

    const float pi = 3.14159265359f;

    // Support for 3D spatial rotations using quaternions, via qmul(qmul(q, v), qconj(q))
    constexpr lc_v3f qxdir(const lc_quatf& q) { return { q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, (q.x * q.y + q.z * q.w) * 2, (q.z * q.x - q.y * q.w) * 2 }; }
    constexpr lc_v3f qydir(const lc_quatf& q) { return { (q.x * q.y - q.z * q.w) * 2, q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, (q.y * q.z + q.x * q.w) * 2 }; }
    constexpr lc_v3f qzdir(const lc_quatf& q) { return { (q.z * q.x + q.y * q.w) * 2, (q.y * q.z - q.x * q.w) * 2, q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z }; }
    constexpr lc_v3f xyz(const lc_v4f& a) { return { a.x, a.y, a.z }; }
    constexpr lc_v3f cross(const lc_v3f& a, const lc_v3f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
    constexpr float dot(const lc_v3f& a, const lc_v3f& b) { return  a.x * b.x + a.y * b.y + a.z * b.z; }
    constexpr float dot(const lc_v4f& a, const lc_v4f& b) { return  a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
    constexpr lc_v2f operator + (const lc_v2f& a, const lc_v2f& b) { return lc_v2f{ a.x + b.x, a.y + b.y }; }
    constexpr lc_v2f operator - (const lc_v2f& a, const lc_v2f& b) { return lc_v2f{ a.x - b.x, a.y - b.y }; }
    constexpr lc_v3f operator + (const lc_v3f& a, const lc_v3f& b) { return lc_v3f{ a.x + b.x, a.y + b.y, a.z + b.z }; }
    constexpr lc_v3f operator - (const lc_v3f& a, const lc_v3f& b) { return lc_v3f{ a.x - b.x, a.y - b.y, a.z - b.z }; }
    constexpr lc_v3f operator * (const lc_v3f& a, float b) { return lc_v3f{ a.x * b, a.y * b, a.z * b }; }
    constexpr lc_v3f operator / (const lc_v3f& a, float b) { return lc_v3f{ a * (1.f / b) }; }
    constexpr lc_v4f operator + (const lc_v4f& a, const lc_v4f& b) { return lc_v4f{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
    constexpr lc_v4f operator * (const lc_v4f& a, float b) { return lc_v4f{ a.x * b, a.y * b, a.z * b, a.w * b }; }
    constexpr lc_v3f& operator += (lc_v3f& a, const lc_v3f& b) { return a = a + b; }
    constexpr lc_v2f mul(const lc_v2f& a, float b) { return { a.x * b, a.y * b }; }
    constexpr lc_v3f mul(const lc_v3f& a, float b) { return { a.x * b, a.y * b, a.z * b }; }
    constexpr lc_v4f mul(const lc_v4f& a, float b) { return { a.x * b, a.y * b, a.z * b, a.w * b }; }
    constexpr lc_v3f mul(const lc_m44f& a, const lc_v3f& b) { return xyz(a.x) * b.x + xyz(a.y) * b.y + xyz(a.z) * b.z; }
    constexpr lc_v4f mul(const lc_m44f& a, const lc_v4f& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
    constexpr lc_quatf mul(const lc_quatf& a, float b) { return { a.x * b, a.y * b, a.z * b, a.w * b }; }
    constexpr lc_quatf mul(const lc_quatf& a, const lc_quatf& b) { return { a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y, a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z, a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z }; }
    constexpr lc_m44f mul(const lc_m44f& a, const lc_m44f& b) { return { mul(a,b.x), mul(a,b.y), mul(a,b.z), mul(a,b.w) }; }
    float length(const lc_v3f& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
    lc_v3f normalize(const lc_v3f& a) { return a * (1.f / length(a)); }
    float length(const lc_quatf& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w); }
    lc_quatf normalize(const lc_quatf& a)
    {
        float l = 1.f / length(a) * (a.w < 0 ? -1.f : 1.f); // after normalization, real part to be non-negative
        return { a.x * l, a.y * l, a.z * l, a.w * l };
    }

    //---- matrix ops

    // adjugate from linalg
    lc_m44f adjugate(const lc_m44f& a)
    {
        const lc_v4f& ax = a.x; const lc_v4f& ay = a.y; const lc_v4f& az = a.z; const lc_v4f& aw = a.w;
        return lc_m44f{
            lc_v4f{ ay.y * az.z * aw.w + aw.y * ay.z * az.w + az.y * aw.z * ay.w - ay.y * aw.z * az.w - az.y * ay.z * aw.w - aw.y * az.z * ay.w,
                 ax.y * aw.z * az.w + az.y * ax.z * aw.w + aw.y * az.z * ax.w - aw.y * ax.z * az.w - az.y * aw.z * ax.w - ax.y * az.z * aw.w,
                 ax.y * ay.z * aw.w + aw.y * ax.z * ay.w + ay.y * aw.z * ax.w - ax.y * aw.z * ay.w - ay.y * ax.z * aw.w - aw.y * ay.z * ax.w,
                 ax.y * az.z * ay.w + ay.y * ax.z * az.w + az.y * ay.z * ax.w - ax.y * ay.z * az.w - az.y * ax.z * ay.w - ay.y * az.z * ax.w },
            lc_v4f{ ay.z * aw.w * az.x + az.z * ay.w * aw.x + aw.z * az.w * ay.x - ay.z * az.w * aw.x - aw.z * ay.w * az.x - az.z * aw.w * ay.x,
                 ax.z * az.w * aw.x + aw.z * ax.w * az.x + az.z * aw.w * ax.x - ax.z * aw.w * az.x - az.z * ax.w * aw.x - aw.z * az.w * ax.x,
                 ax.z * aw.w * ay.x + ay.z * ax.w * aw.x + aw.z * ay.w * ax.x - ax.z * ay.w * aw.x - aw.z * ax.w * ay.x - ay.z * aw.w * ax.x,
                 ax.z * ay.w * az.x + az.z * ax.w * ay.x + ay.z * az.w * ax.x - ax.z * az.w * ay.x - ay.z * ax.w * az.x - az.z * ay.w * ax.x },
            lc_v4f{ ay.w * az.x * aw.y + aw.w * ay.x * az.y + az.w * aw.x * ay.y - ay.w * aw.x * az.y - az.w * ay.x * aw.y - aw.w * az.x * ay.y,
                 ax.w * aw.x * az.y + az.w * ax.x * aw.y + aw.w * az.x * ax.y - ax.w * az.x * aw.y - aw.w * ax.x * az.y - az.w * aw.x * ax.y,
                 ax.w * ay.x * aw.y + aw.w * ax.x * ay.y + ay.w * aw.x * ax.y - ax.w * aw.x * ay.y - ay.w * ax.x * aw.y - aw.w * ay.x * ax.y,
                 ax.w * az.x * ay.y + ay.w * ax.x * az.y + az.w * ay.x * ax.y - ax.w * ay.x * az.y - az.w * ax.x * ay.y - ay.w * az.x * ax.y },
            lc_v4f{ ay.x * aw.y * az.z + az.x * ay.y * aw.z + aw.x * az.y * ay.z - ay.x * az.y * aw.z - aw.x * ay.y * az.z - az.x * aw.y * ay.z,
                 ax.x * az.y * aw.z + aw.x * ax.y * az.z + az.x * aw.y * ax.z - ax.x * aw.y * az.z - az.x * ax.y * aw.z - aw.x * az.y * ax.z,
                 ax.x * aw.y * ay.z + ay.x * ax.y * aw.z + aw.x * ay.y * ax.z - ax.x * ay.y * aw.z - aw.x * ax.y * ay.z - ay.x * aw.y * ax.z,
                 ax.x * ay.y * az.z + az.x * ax.y * ay.z + ay.x * az.y * ax.z - ax.x * az.y * ay.z - ay.x * ax.y * az.z - az.x * ay.y * ax.z } };
    }

    float determinant(lc_m44f const& a)
    {
        const lc_v4f& ax = a.x; const lc_v4f& ay = a.y; const lc_v4f& az = a.z; const lc_v4f& aw = a.w;
        return ax.x * (ay.y * az.z * aw.w + aw.y * ay.z * az.w + az.y * aw.z * ay.w - ay.y * aw.z * az.w - az.y * ay.z * aw.w - aw.y * az.z * ay.w)
             + ax.y * (ay.z * aw.w * az.x + az.z * ay.w * aw.x + aw.z * az.w * ay.x - ay.z * az.w * aw.x - aw.z * ay.w * az.x - az.z * aw.w * ay.x)
             + ax.z * (ay.w * az.x * aw.y + aw.w * ay.x * az.y + az.w * aw.x * ay.y - ay.w * aw.x * az.y - az.w * ay.x * aw.y - aw.w * az.x * ay.y)
             + ax.w * (ay.x * aw.y * az.z + az.x * ay.y * aw.z + aw.x * az.y * ay.z - ay.x * az.y * aw.z - aw.x * ay.y * az.z - az.x * aw.y * ay.z);
    }

    lc_m44f transpose(lc_m44f const& a)
    {
        return {
            a.x.x, a.y.x, a.z.x, a.w.x,
            a.x.y, a.y.y, a.z.y, a.w.y,
            a.x.z, a.y.z, a.z.z, a.w.z,
            a.x.w, a.y.w, a.z.w, a.w.w
        };
    }

    lc_m44f invert(lc_m44f const& mat)
    {
        lc_m44f m = adjugate(mat);
        float oo_det = 1.f / determinant(mat);
        float* ptr = &m.x.x;
        for (int i = 0; i < 16; i++)
            ptr[i] *= oo_det;
        return m;
    }

    lc_m44f rotation_matrix_from_quat(lc_quatf const& v)
    {
        lc_v3f xaxis = {
            1 - 2 * (v.y * v.y + v.z * v.z),
            2 * (v.x * v.y + v.z * v.w),
            2 * (v.z * v.x - v.y * v.w),
        };
        lc_v3f yaxis = {
            2 * (v.x * v.y - v.z * v.w),
            1 - 2 * (v.z * v.z + v.x * v.x),
            2 * (v.y * v.z + v.x * v.w),
        };
        lc_v3f zaxis = {
            2 * (v.z * v.x + v.y * v.w),
            2 * (v.y * v.z - v.x * v.w),
            1 - 2 * (v.y * v.y + v.x * v.x),
        };

        return {
            lc_v4f{ xaxis.x, xaxis.y, xaxis.z, 0.f },
            lc_v4f{ yaxis.x, yaxis.y, yaxis.z, 0.f },
            lc_v4f{ zaxis.x, zaxis.y, zaxis.z, 0.f },
            lc_v4f{ 0.f, 0.f, 0.f, 1.f } };
    }

    lc_m44f rotation_xyz(lc_v3f e)
    {
        float cx = cosf(e.x);
        float cy = cosf(e.y);
        float cz = cosf(e.z);

        float sx = sinf(e.x);
        float sy = sinf(e.y);
        float sz = sinf(e.z);

        lc_m44f m;
        m.x.x = cx * cz;
        m.x.y = cx * -sz;
        m.x.z = -sx;
        m.x.w = 0;

        m.y.x = sx * sy * cz + cy * sz;
        m.y.y = sx * sy * sz + cy * cz;
        m.y.z = cx * sy;
        m.y.w = 0;

        m.z.x = sx * cy * cz - sy * sz;
        m.z.y = sx * cy * sz - sy * cz;
        m.z.z = cx * cy;
        m.z.w = 0;

        m.w = { 0,0,0,1 };
        return { m };
    }

    lc_m44f rotx(float r)
    {
        float c = cosf(r);
        float s = sinf(r);
        return { 1, 0, 0, 0,
                 0, c,-s, 0,
                 0, s, c, 0,
                 0, 0, 0, 1 };
    }

    lc_m44f roty(float r)
    {
        float c = cosf(r);
        float s = sinf(r);
        return { c, 0, s, 0,
                 0, 1, 0, 0,
                -s, 0, c, 0,
                 0, 0, 0, 1 };
    }

    lc_m44f rotz(float r)
    {
        float c = cosf(r);
        float s = sinf(r);
        return { c,-s, 0, 0,
                 s, c, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1 };
    }

    lc_m44f mat_from_axis_angle(const lc_v3f& axis, float angle)
    {
        float length2 = dot(axis, axis);
        if (length2 < FLT_EPSILON)
            return m44f_identity;

        lc_v3f n = normalize(axis);
        float s = sinf(angle);
        float c = cosf(angle);
        float k = 1.f - c;

        float xx = n.x * n.x * k + c;
        float yy = n.y * n.y * k + c;
        float zz = n.z * n.z * k + c;
        float xy = n.x * n.y * k;
        float yz = n.y * n.z * k;
        float zx = n.z * n.x * k;
        float xs = n.x * s;
        float ys = n.y * s;
        float zs = n.z * s;

        lc_m44f m;
        m.x.x = xx;
        m.x.y = xy + zs;
        m.x.z = zx - ys;
        m.x.w = 0.f;
        m.y.x = xy - zs;
        m.y.y = yy;
        m.y.z = yz + xs;
        m.y.w = 0.f;
        m.z.x = zx + ys;
        m.z.y = yz - xs;
        m.z.z = zz;
        m.z.w = 0.f;
        m.w.x = 0.f;
        m.w.y = 0.f;
        m.w.z = 0.f;
        m.w.w = 1.f;
        return m;
    }

    lc_m44f make_lookat_transform(const lc_v3f& eye, const lc_v3f& target, const lc_v3f& up)
    {
        lc_v3f zaxis = normalize(eye - target);
        lc_v3f xaxis = normalize(cross(up, zaxis));
        lc_v3f yaxis = cross(zaxis, xaxis);
        return lc_m44f{ lc_v4f{ xaxis.x, yaxis.x, zaxis.x, 0.f },
                     lc_v4f{ xaxis.y, yaxis.y, zaxis.y, 0.f },
                     lc_v4f{ xaxis.z, yaxis.z, zaxis.z, 0.f },
                     lc_v4f{ -dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1.f } };
    }

    //---- quat ops

    inline lc_quatf quat_inverse(const lc_quatf& q)
    {
        lc_v4f q1 = { -q.x,-q.y,-q.z,q.w };
        float d = 1.f / dot(q1, q1);
        return { q1.x * d, q1.y * d, q1.z * d, q1.w * d };
    }


    // swing twist decomposition.
    // cf. https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis/4341489
    // assumes dir is normailzed
    inline float angle_about_dir(const lc_quatf& q, const lc_v3f& dir)
    {
        lc_v3f axis = { q.x, q.y, q.z };
        float dot_product = dot(dir, axis);
        lc_v3f projection = mul(dir, dot_product);
        lc_quatf twist = normalize(lc_quatf{ projection.x, projection.y, projection.z, q.w });

        if (dot_product < 0.0) {
            // Ensure `twist` points towards `direction`
            // (if calculating the new axis, then all of twist needs negating)
            twist.w = -twist.w;
            // Rotation angle `twist.angle()` is now reliable
        }
        return 2.f * acosf(twist.w); // if axis and angle is being computed, more work is necessary: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
    }

    inline lc_quatf rotation_about_dir(const lc_quatf& q, const lc_v3f& dir)
    {
        lc_v3f axis = { q.x, q.y, q.z };
        float dot_product = dot(dir, axis);
        lc_v3f projection = mul(dir, dot_product);
        lc_quatf twist = normalize(lc_quatf{ projection.x, projection.y, projection.z, q.w });

        if (dot_product < 0.0) {
            // Ensure `twist` points towards `direction`
            twist = mul(twist, -1.f);
            // Rotation angle `twist.angle()` is now reliable
        }
        return twist;
    }

    inline lc_quatf quat_set_rotation_internal(lc_v3f const& f0, lc_v3f const& t0)
    {
        lc_v3f h0 = normalize(f0 + t0);
        lc_v3f v = cross(f0, h0);
        return { v.x, v.y, v.z, dot(f0, h0) };
    }

    inline lc_quatf quat_from_vector_to_vector(lc_v3f const& from, lc_v3f const& to)
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
        lc_v3f f0 = normalize(from);
        lc_v3f t0 = normalize(to);
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

            lc_v3f f02 = { f0.x * f0.x, f0.y * f0.y, f0.z * f0.z };
            lc_v3f v;

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

        lc_v3f h0 = normalize(f0 + t0);
        lc_quatf q = quat_set_rotation_internal(f0, h0);
        return mul(q, quat_set_rotation_internal(h0, t0));
    }

    inline lc_quatf quat_from_axis_angle(lc_v3f v, float a)
    {
        lc_quatf Result;
        float s = std::sin(a * 0.5f);
        Result.w = std::cos(a * 0.5f);
        Result.x = v.x * s;
        Result.y = v.y * s;
        Result.z = v.z * s;
        return Result;
    }

    inline lc_v3f euler_from_quat(const lc_quatf& q)
    {
        lc_v3f ypr;
        const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        ypr.x = float(atan2(2. * q2 * q3 + 2. * q0 * q1, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0));
        ypr.y = float(asin(2. * q1 * q3 - 2. * q0 * q2));
        ypr.z = float(atan2(2. * q1 * q2 + 2. * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2));
        return ypr;
    }

    inline lc_quatf quat_from_euler(const lc_v3f& e)
    {
        lc_quatf x = quat_from_axis_angle({ 1, 0, 0 }, e.x);
        lc_quatf y = quat_from_axis_angle({ 0, 1, 0 }, e.y);
        lc_quatf z = quat_from_axis_angle({ 0, 0, 1 }, e.z);
        lc_quatf result = normalize(mul(mul(x, y), z));
        return result;
    }

    inline lc_quatf quat_from_ypr(const lc_v3f& ypr)
    {
        lc_v3f e = { ypr.y, ypr.x, ypr.z };
        return quat_from_euler(e);
    }

    // returns a ypr vector consistent with the ypr being used throughout this API
    // ypr is distinct from an Euler angle in that it is the specific pair of angles
    // of azimuth and declination, as opposed to an arbitrary but otherwise correct
    // Euler angle tuple.

    inline lc_v3f ypr_from_quat(const lc_quatf& q)
    {
        lc_v3f qz = qzdir(q);
        lc_v3f ypr = { atan2f(qz.x, qz.z),
                    atan2f(qz.y, sqrtf(qz.x * qz.x + qz.z * qz.z)),
                    0 };

        if (ypr.y > pi)
            ypr.y = 2.f * pi - ypr.y;
        else
            ypr.y *= -1.f;

        if (fabsf(ypr.y) < 0.9995f)
        {
            // if not pitched straight up, calculate roll
            // compute normal to plane of zdir and up, which is parallel to the xz plane
            lc_v3f world_up = { 0.f, 1.f, 0.f };
            lc_v3f world_right = normalize(cross(world_up, qz));

            // the area of bivector formed world_right and right is the cosine of the angle between them
            lc_v3f camera_right = normalize(qxdir(q));
            float cos_r = dot(world_right, camera_right);
            float sin_r = length(cross(world_right, camera_right));
            float roll = std::atan2f(sin_r, cos_r);

            if (std::isfinite(roll))
            {
                ypr.z = roll;
            }
        }

        return ypr;
    }

    inline lc_v3f quat_rotate_vector(lc_quatf q, const lc_v3f& v)
    {
        // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
        lc_v3f u{ q.x, q.y, q.z };
        float s = q.w;

        return    u * 2.f * dot(u, v)
            + v * (s * s - dot(u, u))
            + cross(u, v) * 2.f * s;
    }

    lc_quatf quat_from_matrix(const lc_m44f& mat)
    {
        float s;
        float q[4];
        int i, j, k;
        lc_quatf quat;

        int nxt[3] = { 1, 2, 0 };
        float tr = mat.x.x + mat.y.y + mat.z.z;

        // check the diagonal
        if (tr > 0.0) {
            s = sqrtf(tr + 1.f);
            quat.w = s / 2.f;
            s = 0.5f / s;

            quat.x = (mat.y.z - mat.z.y) * s;
            quat.y = (mat.z.x - mat.x.z) * s;
            quat.z = (mat.x.y - mat.y.x) * s;
        }
        else {
            // diagonal is negative
            i = 0;

            const lc_v4f* m[3] = { &mat.x, &mat.y, &mat.z };

            if (m[1]->y > m[0]->x)
                i = 1;

            lc_v4f const* const mat_i = m[i];
            float const* const f_i = reinterpret_cast<float const* const>(mat_i);
            if (m[2]->z > f_i[i])
                i = 2;

            j = nxt[i];
            k = nxt[j];

            lc_v4f const* const mat_j = m[j];
            float const* const f_j = reinterpret_cast<float const* const>(mat_j);
            lc_v4f const* const mat_k = m[k];
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

    lc_quatf invert(const lc_quatf& q)
    {
        float len = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        lc_quatf result = { -q.x, -q.w, -q.z, q.w };
        return mul(result, len);
    }

    //---- rays

    static bool intersect_ray_plane(const lc_ray& ray, const lc_v3f& point, const lc_v3f& normal,
        lc_v3f* intersection = nullptr, float* outT = nullptr)
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
                    lc_v3f result = ray.pos;
                    result.x += t * ray.dir.x;
                    result.y += t * ray.dir.y;
                    result.z += t * ray.dir.z;
                    *intersection = result;
                }
                return true;
            }
        }
        if (outT)
            *outT = std::numeric_limits<float>::max();
        return false;
    }

    lc_ray get_ray(lc_m44f const& inv_projection, lc_v3f const& camera_position, lc_v2f const& pixel, lc_v2f const& viewport_origin, lc_v2f const& viewport_size)
    {
        // 3d normalized device coordinates
        const float x = 2 * (pixel.x - viewport_origin.x) / viewport_size.x - 1;
        const float y = 1 - 2 * (pixel.y - viewport_origin.y) / viewport_size.y;

        // eye coordinates
        lc_v4f p0 = mul(inv_projection, lc_v4f{ x, y, -1, 1 });
        lc_v4f p1 = mul(inv_projection, lc_v4f{ x, y, +1, 1 });

        p1 = mul(p1, 1.f / p1.w);
        p0 = mul(p0, 1.f / p0.w);
        return { camera_position, normalize(lc_v3f { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z }) };
    }

    float distance_point_to_plane(lc_v3f const& a, lc_v3f const& point, lc_v3f const& normal)
    {
        lc_v3f d = point - a;
        return dot(d, normal);
    }

} // anonymous namespace

void lc_rt_set_identity(lc_rigid_transform* rt)
{
    if (!rt)
        return;

    memset(static_cast<void*>(rt), 0, sizeof(lc_rigid_transform));
    rt->orientation.w = 1.f;
    rt->scale = { 1.f, 1.f, 1.f };
}

void lc_rt_set_ops(lc_rigid_transform* rt, lc_quatf orientation, lc_v3f position, lc_v3f scale)
{
    rt->orientation = orientation;
    rt->position = position;
    rt->scale = scale;
}

void lc_rt_set_op_uniform_scale(lc_rigid_transform* rt, lc_quatf orientation, lc_v3f position, float scale)
{
    rt->orientation = orientation;
    rt->position = position;
    rt->scale = lc_v3f{ scale, scale, scale };
}

void lc_rt_set_op(lc_rigid_transform* rt, lc_quatf orientation, lc_v3f position)
{
    rt->orientation = orientation;
    rt->position = position;
}

lc_m44f lc_rt_matrix(const lc_rigid_transform* rt)
{
    lc_v3f x = mul(qxdir(rt->orientation), rt->scale.x);
    lc_v3f y = mul(qxdir(rt->orientation), rt->scale.y);
    lc_v3f z = mul(qxdir(rt->orientation), rt->scale.z);
    lc_m44f result = { x.x, x.y, x.z, 0.f,
                    y.x, y.y, y.z, 0.f,
                    z.x, z.y, z.z, 0.f,
                    rt->position.x, rt->position.y, rt->position.z, 1.f };
    return result;
}

lc_v3f lc_rt_transform_vector(const lc_rigid_transform* rt, lc_v3f vec)
{
    lc_v3f v = { vec.x * rt->scale.x, vec.y * rt->scale.y, vec.z * rt->scale.z };
    return quat_rotate_vector(rt->orientation, v);
}

lc_v3f lc_rt_detransform_vector(const lc_rigid_transform* rt, lc_v3f vec)
{
    lc_quatf o = quat_inverse(rt->orientation);
    lc_v3f r = quat_rotate_vector(o, vec);
    return { r.x / rt->scale.x, r.y / rt->scale.y, r.z / rt->scale.z };
}

lc_v3f lc_rt_transform_point(const lc_rigid_transform* rt, lc_v3f p)
{
    return rt->position + lc_rt_transform_vector(rt, p);
}

lc_v3f lc_rt_detransform_point(const lc_rigid_transform* rt, lc_v3f p)
{
    return lc_rt_detransform_vector(rt, p - rt->position);
}

lc_v3f lc_rt_right(const lc_rigid_transform* rt)
{
    return qxdir(rt->orientation);
}

lc_v3f lc_rt_up(const lc_rigid_transform* rt)
{
    return qydir(rt->orientation);
}

lc_v3f lc_rt_forward(const lc_rigid_transform* rt)
{
    return qzdir(rt->orientation);
}

struct lc_interaction
{
    uint64_t _epoch = 0;

    // constraints
    lc_v3f _world_up{ 0, 1, 0 };
    lc_v3f _orbit_center{ 0, 0, 0 };

    // local settings
    float _orbit_speed = 0.5f;
    float _pan_tilt_speed = 0.25f;

    // working state
    lc_v2f _viewport_size = { 0, 0 };
    lc_v3f _initial_focus_point = { 0, 0, 0 };
    float _initial_focus_distance = 5.f;
    lc_v2f _init_mouse{ 0,0 };
    lc_v2f _prev_mouse{ 0,0 };
    lc_m44f _initial_inv_projection = { 1,0,0,0, 0,1,0,0, 0,0,1,0.2f, 0,0,0,1 };

    void _dolly(lc_camera& camera, const lc_v3f& delta);
    void _turntable(lc_camera& camera, const lc_v2f& delta);
    void _pantilt(lc_camera& camera, const lc_v2f& delta);
};

lc_interaction* lc_i_create_interactive_controller()
{
    return new lc_interaction();
}

void lc_i_free_interactive_controller(lc_interaction* i)
{
    if (i)
        delete i;
}

lc_i_Phase lc_update_phase(lc_i_Phase current_phase, bool button_click)
{
    if (button_click)
    {
        switch (current_phase) {
        case lc_i_PhaseNone:     return lc_i_PhaseStart;
        case lc_i_PhaseRestart:  return lc_i_PhaseRestart;
        case lc_i_PhaseStart:    return lc_i_PhaseContinue;
        case lc_i_PhaseContinue: return lc_i_PhaseContinue;
        case lc_i_PhaseFinish:   return lc_i_PhaseRestart;
        }
    }
    else
    {
        switch (current_phase) {
        case lc_i_PhaseNone:     return lc_i_PhaseNone;
        case lc_i_PhaseRestart:  return lc_i_PhaseRestart;
        case lc_i_PhaseStart:    return lc_i_PhaseFinish;
        case lc_i_PhaseContinue: return lc_i_PhaseFinish;
        case lc_i_PhaseFinish: return lc_i_PhaseNone;
        }
    }
}



//-----------------------------------------------------------------------------
// lc_interaction
//
//-----------------------------------------------------------------------------

InteractionToken lc_i_begin_interaction(lc_interaction* i, lc_v2f viewport_size)
{
    i->_viewport_size = viewport_size;
    ++i->_epoch;
    return i->_epoch;
}

void lc_i_sync_constraints(lc_interaction* i, lc_interaction* ptc)
{
    if (ptc->_epoch == i->_epoch)
        return;

    if (ptc->_epoch > i->_epoch)
    {
        // update constarints from incoming controller
        i->_world_up = ptc->_world_up;
        i->_orbit_center = ptc->_orbit_center;
        i->_epoch = ptc->_epoch;
    }
    else
    {
        // update constarints from incoming controller
        ptc->_world_up = i->_world_up;
        ptc->_orbit_center = i->_orbit_center;
        ptc->_epoch = i->_epoch;
    }
}

void lc_i_end_interaction(lc_interaction*, InteractionToken)
{
}

/// @TODO - this belongs under lc_mount
void lc_i_set_roll(lc_interaction* i, lc_camera* camera, InteractionToken, lc_radians r)
{
    lc_v3f dir = lc_rt_forward(&camera->mount.transform);
    lc_quatf q = quat_from_axis_angle(dir, r.rad);
    q = mul(q, camera->mount.transform.orientation);
    lc_mount_set_view_transform_quat_pos(&camera->mount, q, camera->mount.transform.position);
}

void lc_interaction::_dolly(lc_camera& camera, const lc_v3f& delta)
{
    const lc_rigid_transform* cmt = &camera.mount.transform;
    lc_v3f pos = cmt->position;
    lc_v3f camera_to_focus = pos - _orbit_center;
    float distance_to_focus = length(camera_to_focus);
    const float feel = 0.02f;
    float scale = std::max(0.01f, logf(distance_to_focus) * feel);
    lc_v3f deltaX = lc_rt_right(cmt) * -delta.x * scale;
    lc_v3f dP = lc_rt_forward(cmt) * -delta.z * scale - deltaX - lc_rt_up(cmt) * -delta.y * scale;
    _orbit_center += dP;
    lc_mount_set_view_transform_quat_pos(&camera.mount, cmt->orientation, cmt->position + dP);
};

void lc_interaction::_turntable(lc_camera& camera, const lc_v2f& delta)
{
    const lc_rigid_transform* cmt = &camera.mount.transform;
    lc_v3f up = { 0,1,0 };   // turntable orbits about the world up axis
    lc_v3f fwd = lc_rt_forward(cmt);
    bool test_inversion = fabsf(dot(up, fwd)) > (1.f - 1.e-5);

    lc_v3f rt = normalize(cross(up, fwd));
    lc_quatf rx = quat_from_axis_angle(up, delta.x * _orbit_speed);
    lc_quatf ry = quat_from_axis_angle(rt, -delta.y * _orbit_speed * 0.25f);
    lc_quatf quat_step = mul(ry, rx);
    lc_quatf new_quat = mul(cmt->orientation, quat_step);

    lc_v3f pos_pre = cmt->position;
    lc_v3f pos = pos_pre - _orbit_center;
    pos = quat_rotate_vector(quat_step, pos) + _orbit_center;

    // because the orientation is synthesized from a motion in world space
    // and a rotation in camera space, recompose the camera orientation by
    // constraining the camera's direction to face the orbit center.
    lc_v3f local_up = { 0, 1, 0 };
    lc_mount_look_at(&camera.mount, pos, _orbit_center, local_up);

    lc_v3f rt_post = lc_rt_right(cmt);
    if (dot(rt_post, rt) < -0.5f)
    {
        // if the input rotation causes motion past the pole, reset it.
        lc_mount_look_at(&camera.mount, pos_pre, _orbit_center, local_up);
    }
}

void lc_interaction::_pantilt(lc_camera& camera, const lc_v2f& delta)
{
    const lc_rigid_transform* cmt = &camera.mount.transform;

    lc_quatf restore_quat = cmt->orientation;
    lc_v3f restore_pos = cmt->position;
    lc_v3f restore_orbit = _orbit_center;

    lc_v3f up = { 0,1,0 };   // turntable orbits about the world up axis
    lc_v3f rt = normalize(cross(up, lc_rt_forward(cmt)));
    lc_quatf rx = quat_from_axis_angle(up, -delta.x * _pan_tilt_speed);
    lc_quatf ry = quat_from_axis_angle(rt, delta.y * _pan_tilt_speed * 0.25f);
    lc_quatf quat_step = mul(rx, ry);
    lc_quatf new_quat = mul(cmt->orientation, quat_step);

    lc_v3f pos = restore_orbit - cmt->position;
    _orbit_center = quat_rotate_vector(quat_step, pos) + cmt->position;

    // because the orientation is synthesized from a motion in world space
    // and a rotation in camera space, recompose the camera orientation by
    // constraining the camera's direction to face the orbit center.
    lc_mount_look_at(&camera.mount, cmt->position, _orbit_center, lc_v3f{ 0,1,0 });

    lc_v3f rt_post = lc_rt_right(cmt);
    if (dot(rt_post, rt) < -0.5f)
    {
        // if the input rotation causes motion past the pole, reset it.
        lc_mount_set_view_transform_quat_pos(&camera.mount, restore_quat, restore_pos);
        _orbit_center = restore_orbit;
    }
};




// delta is the 2d motion of a mouse or gesture in the screen plane,
// typically computed as scale * (currMousePos - prevMousePos);
//
void lc_i_single_stick_interaction(lc_interaction* i,
    lc_camera* camera, InteractionToken tok,
    lc_i_Mode mode, lc_v2f delta_in, lc_radians roll_hint, float /*dt*/)
{
    //if (phase == lc_i_PhaseNone)
    //    return;

    const lc_rigid_transform* cmt = &camera->mount.transform;

    // joystick mode controls
    lc_v2f delta = delta_in;

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

    switch (mode)
    {
    case lc_i_ModeDolly:
        i->_dolly(*camera, { delta.x, 0, delta.y });
        break;

    case lc_i_ModeCrane:
        i->_dolly(*camera, { delta.x, delta.y, 0 });
        break;

    case lc_i_ModePanTilt:
        i->_pantilt(*camera, { delta.x, delta.y });
        lc_i_set_roll(i, camera, tok, roll_hint);
        break;

    case lc_i_ModeTurnTableOrbit:
        i->_turntable(*camera, delta);
        lc_i_set_roll(i,camera, tok, roll_hint);
        break;

    case lc_i_ModeStatic:
        // do nothing
        break;
    case lc_i_ModeArcball:
        //not supported
        break;
    }
}


void lc_i_dual_stick_interaction(lc_interaction* i, lc_camera* camera, InteractionToken tok,
    lc_i_Mode mode, lc_v3f pos_delta_in, lc_v3f rotation_delta_in,
    lc_radians roll_hint, float /*dt*/)
{
    //if (phase == lc_i_PhaseNone)
    //    return;

    lc_v3f pos_delta = pos_delta_in;
    lc_v3f rotation_delta = rotation_delta_in;
    auto curve = [&](float& x)
    {
        const float buffer = 4.f;
        if (fabsf(x) < buffer)
        {
            float dx = fabsf(x) / buffer;
            dx *= dx;
            x = buffer * copysign(dx, x);
        }
    };

    // make control less sensitive within the buffer
    curve(pos_delta.x);
    curve(pos_delta.y);
    curve(rotation_delta.x);
    curve(rotation_delta.y);

    switch (mode)
    {
    case lc_i_ModeDolly:
    {
        i->_dolly(*camera, { pos_delta.x, 0, pos_delta.z });
        i->_pantilt(*camera, { rotation_delta.x, rotation_delta.z });
        break;
    }
    case lc_i_ModeCrane:
    {
        i->_dolly(*camera, { pos_delta.x, pos_delta.z, 0 });
        i->_pantilt(*camera, { rotation_delta.x, rotation_delta.z });
        break;
    }

    case lc_i_ModePanTilt:
    {
        i->_dolly(*camera, { pos_delta.x, 0, pos_delta.z });
        i->_pantilt(*camera, { rotation_delta.x, rotation_delta.z });
        break;
    }

    case lc_i_ModeTurnTableOrbit:
    {
        i->_dolly(*camera, { pos_delta.x, 0, pos_delta.z });
        i->_turntable(*camera, { rotation_delta.x, rotation_delta.z });
        lc_i_set_roll(i, camera, tok, roll_hint);
        break;
    }

    case lc_i_ModeStatic:
        // do nothing
        break;
    case lc_i_ModeArcball:
        //not supported
        break;
    }
}


// Initial is the screen position of the beginning of the interaction, current is the
// current position
//
void lc_i_ttl_interaction(lc_interaction* i, lc_camera* camera, InteractionToken tok,
    lc_i_Phase phase, lc_i_Mode mode, lc_v2f current_mouse_, lc_radians roll_hint, float dt)
{
    if (phase == lc_i_PhaseNone)
        return;

    const lc_rigid_transform* cmt = &camera->mount.transform;
    switch (mode)
    {
    case lc_i_ModeArcball:
    {
        lc_v2f current_mouse = current_mouse_;

        if (phase == lc_i_PhaseStart)
        {
            i->_init_mouse = current_mouse;
        }
        else if (phase == lc_i_PhaseFinish)
        {
        }

        float w = i->_viewport_size.x * 0.5f;
        float h = i->_viewport_size.y * 0.5f;
        float min_dimension = 1.f / std::min(w, h);
        lc_v3f v0{ i->_init_mouse.x, i->_init_mouse.y, 0.f };
        lc_v3f v1{ current_mouse.x, current_mouse.y, 0.f };

        auto mouse_to_vec = [w, h, min_dimension](lc_v3f& v)
        {
            v.x = -(v.x - w) * min_dimension;
            v.y = (v.y - h) * min_dimension;
            float len_squared = v.x * v.x + v.y * v.y;
            if (len_squared > 1.f)
                len_squared = 1.f;
            v.z = sqrt(1.f - len_squared); // a point on the virtual sphere
        };
        mouse_to_vec(v0);
        mouse_to_vec(v1);

        if (dot(v0, v0) > 1e-4f && dot(v1, v1) > 1e-4f)
        {
            lc_quatf rot = quat_from_vector_to_vector(v0, v1);
            rot = mul(cmt->orientation, rot);
            lc_v3f fwd = { 0, 0, length(cmt->position - i->_orbit_center) };
            lc_v3f pos_pre = cmt->position;
            lc_v3f pos = quat_rotate_vector(normalize(rot), fwd) + i->_orbit_center;
            lc_v3f rt = lc_rt_right(cmt); // turntable tilts about the camera right axis

            // because the orientation is synthesized from a motion in world space
            // and a rotation in camera space, recompose the camera orientation by
            // constraining the camera's direction to face the orbit center.
            lc_v3f local_up = { 0, 1, 0 };
            lc_mount_look_at(&camera->mount, pos, i->_orbit_center, local_up);
            lc_i_set_roll(i, camera, tok, roll_hint);

            lc_v3f rt_post = lc_rt_right(cmt);
            if (dot(rt_post, rt) < -0.5f)
            {
                // if the input rotation causes motion past the pole, reset it.
                lc_mount_look_at(&camera->mount, pos_pre, i->_orbit_center, local_up);
            }

            i->_init_mouse = current_mouse;
        }
    }
    break;

    case lc_i_ModeCrane:
    case lc_i_ModeDolly:
    case lc_i_ModeTurnTableOrbit:
    {
        if (phase == lc_i_PhaseStart)
        {
            i->_init_mouse = current_mouse_;
        }

        // Joystick mode
        lc_v2f dp = current_mouse_ - i->_init_mouse;
        lc_v2f prev_dp = current_mouse_ - i->_prev_mouse;

        // reset the anchor if the interaction direction changes on either axis.
        // this is to increase the feeling of responsiveness
        if (dp.x * prev_dp.x < 0)
            i->_init_mouse.x = current_mouse_.x;
        if (dp.y * prev_dp.y < 0)
            i->_init_mouse.y = current_mouse_.y;
        dp = current_mouse_ - i->_init_mouse;

        dp.x /=  i->_viewport_size.x;
        dp.y /= -i->_viewport_size.y;
        lc_i_single_stick_interaction(i, camera, tok, mode, dp, roll_hint, dt);
        break;
    }

    case lc_i_ModePanTilt:
    {
        if (phase == lc_i_PhaseStart)
        {
            i->_init_mouse = current_mouse_;
            i->_initial_inv_projection = lc_camera_inv_view_projection(camera, 1.f);
            i->_initial_focus_point = i->_orbit_center;
        }

        lc_v3f pos = camera->mount.transform.position;

        // Through the lens gimbal
        lc_ray original_ray = get_ray(i->_initial_inv_projection,
            pos, i->_init_mouse,
            { 0, 0 }, i->_viewport_size);
        lc_ray new_ray = get_ray(i->_initial_inv_projection,
            pos, current_mouse_,
            { 0, 0 }, i->_viewport_size);

        lc_quatf rotation = quat_from_vector_to_vector(new_ray.dir, original_ray.dir); // rotate the orbit center in the opposite direction

        lc_v3f rel = i->_initial_focus_point - pos;
        rel = quat_rotate_vector(rotation, rel);
        i->_orbit_center = pos + rel;
        lc_mount_look_at(&camera->mount, pos, i->_orbit_center, lc_i_world_up_constraint(i));
        lc_i_set_roll(i, camera, tok, roll_hint);
        break;
    }

    case lc_i_ModeStatic:
        // do nothing
        break;
    } // switch

    i->_prev_mouse = current_mouse_;
}

void lc_i_constrained_ttl_interaction(lc_interaction* i,
    lc_camera* camera, InteractionToken tok,
    lc_i_Phase phase, lc_i_Mode mode,
    lc_v2f current,
    lc_v3f initial_hit_point,
    lc_radians roll_hint,
    float dt)
{
    if (phase == lc_i_PhaseNone)
        return;

    const lc_rigid_transform* cmt = &camera->mount.transform;
    switch (mode)
    {
    case lc_i_ModeCrane:
    {
        if (phase == lc_i_PhaseStart)
        {
            i->_initial_focus_point = initial_hit_point;
        }

        // Through the lens crane
        lc_v2f target_xy = lc_camera_project_to_viewport(camera, lc_v2f{ 0,0 }, i->_viewport_size, i->_initial_focus_point) - current;
        target_xy = mul(target_xy, 1.f / i->_viewport_size.x);
        lc_v3f delta = mul(lc_rt_right(cmt), target_xy.x * 1.f);
        delta += mul(lc_rt_up(cmt), target_xy.y * -1.f);
        i->_orbit_center += delta;
        lc_mount_set_view_transform_quat_pos(&camera->mount, camera->mount.transform.orientation, cmt->position + delta);
        break;
    }
    case lc_i_ModeDolly:
    {
        if (phase == lc_i_PhaseStart)
        {
            i->_initial_focus_point = initial_hit_point;
        }

        // Through the lens crane
        lc_v2f target_xy = lc_camera_project_to_viewport(camera, lc_v2f{ 0,0 }, i->_viewport_size, i->_initial_focus_point) - current;
        target_xy = mul(target_xy, 1.f / i->_viewport_size.x);
        lc_v3f delta = mul(lc_rt_right(cmt), target_xy.x * 1.f);
        lc_v3f delta_fw = mul(lc_rt_forward(cmt), target_xy.y * -1.f);

        lc_v3f test_pos = cmt->position + delta_fw;
        float dist = distance_point_to_plane(test_pos, i->_initial_focus_point, lc_rt_forward(cmt));
        if (dist < 0)
        {
            // moving forward would not push past the plane (focus_point, mount.forward())?
            i->_orbit_center += delta + delta_fw;
            lc_mount_set_view_transform_quat_pos(&camera->mount, camera->mount.transform.orientation, test_pos + delta);
        }
        break;
    }

    default:
        lc_i_ttl_interaction(i, camera, tok, phase, mode, current, roll_hint, dt);
        break;
    }
}

lc_v3f lc_i_world_up_constraint(const lc_interaction* i)
{
    return i->_world_up;
}

lc_v3f lc_i_orbit_center_constraint(const lc_interaction* i)
{
    return i->_orbit_center;
}

void lc_i_set_orbit_center_constraint(lc_interaction* i, lc_v3f pos)
{
    i->_orbit_center = pos;
}

void lc_i_set_world_up_constraint(lc_interaction* i, lc_v3f up)
{
    i->_world_up = up;
}

void lc_i_set_speed(lc_interaction* i, float o, float pt)
{
    i->_orbit_speed = o;
    i->_pan_tilt_speed = pt;
}

//-----------------------------------------------------------------------------
// Aperture
//
//-----------------------------------------------------------------------------

void lc_aperture_set_default(lc_aperture* a)
{
    if (!a)
        return;

    a->shutter_open = 0.f;
    a->shutter_duration = 0.f;
    a->shutter_blades = 6;
    a->iris = lc_millimeters{ 6.25f };
}

//-----------------------------------------------------------------------------
// lc_mount
//
//-----------------------------------------------------------------------------


void lc_mount_set_default(lc_mount* mnt)
{
    lc_rt_set_identity(&mnt->transform);
    lc_mount_look_at(mnt, lc_v3f{ 0, 1.f, 10.f }, lc_v3f{ 0,0,0 }, lc_v3f{ 0,1,0 });
}

lc_m44f lc_mount_gl_view_transform(const lc_mount* mnt)
{
    lc_m44f m = lc_mount_inv_rotation_transform(mnt);
    m.w.x = -dot(lc_v3f{ m.x.x, m.y.x, m.z.x }, mnt->transform.position);
    m.w.y = -dot(lc_v3f{ m.x.y, m.y.y, m.z.y }, mnt->transform.position);
    m.w.z = -dot(lc_v3f{ m.x.z, m.y.z, m.z.z }, mnt->transform.position);
    return m;
}

lc_m44f lc_mount_gl_view_transform_inv(const lc_mount* mnt)
{
    return invert(lc_mount_gl_view_transform(mnt));
}

lc_m44f lc_mount_model_view_transform_f16(const lc_mount* mnt, float const* const view_matrix)
{
    return mul(lc_mount_gl_view_transform(mnt), *(lc_m44f*)view_matrix);
}

lc_m44f lc_mount_model_view_transform_m44f(const lc_mount* mnt, lc_m44f const* const view_matrix)
{
    return mul(lc_mount_gl_view_transform(mnt), *view_matrix);
}

lc_m44f lc_mount_rotation_transform(const lc_mount* mnt)
{
    return rotation_matrix_from_quat(mnt->transform.orientation);
}

lc_m44f lc_mount_inv_rotation_transform(const lc_mount* mnt)
{
    return transpose(lc_mount_rotation_transform(mnt));
}

void lc_mount_set_view_transform_m44f(lc_mount* mnt, lc_m44f const* const m)
{
    lc_v3f p = mul(xyz(m->w), -1.f);
    lc_m44f m2 = *m;
    m2.w = { 0, 0, 0, 1 };
    m2 = transpose(m2);
    mnt->transform.position = mul(m2, p);
    mnt->transform.orientation = quat_from_matrix(*m);
}

void lc_mount_set_view_transform_f16(lc_mount* mnt, float const* const m)
{
    lc_m44f m2 = *(lc_m44f*)m;
    lc_v3f p = mul(xyz(m2.w), -1.f);
    m2.w = { 0, 0, 0, 1 };
    m2 = transpose(m2);
    mnt->transform.position = mul(m2, p);
    mnt->transform.orientation = quat_from_matrix(*(lc_m44f*) m);
}


void lc_mount_set_view_transform_quat_pos(lc_mount* mnt, lc_quatf q, lc_v3f eye)
{
    mnt->transform.position = eye;
    mnt->transform.orientation = normalize(q);
}

void lc_mount_set_view_transform_ypr_eye(lc_mount* mnt, lc_v3f ypr, lc_v3f eye)
{
    mnt->transform.orientation = quat_from_euler(ypr);
    mnt->transform.position = eye;
}


void lc_mount_look_at(lc_mount* mnt, lc_v3f eye, lc_v3f target, lc_v3f up)
{
    mnt->transform.position = eye;
    lc_m44f m = make_lookat_transform(eye, target, up);
    mnt->transform.orientation = quat_from_matrix(transpose(m));
}

void lc_mount_set_view(lc_mount* mnt, float distance, lc_quatf orientation, lc_v3f target, lc_v3f up)
{
    lc_v3f eye = { 0, 0, distance };
    eye = quat_rotate_vector(orientation, eye);
    lc_mount_look_at(mnt, eye, target, up);
}

lc_v3f lc_mount_ypr(const lc_mount* mnt)
{
    return ypr_from_quat(mnt->transform.orientation);
}


//-----------------------------------------------------------------------------
// Optics
//
//-----------------------------------------------------------------------------

void lc_optics_set_default(lc_optics* o)
{
    if (!o)
        return;

    o->fStop = 8.f;
    o->focal_length = lc_millimeters{ 50.f };
    o->zfar = 1e5f;
    o->znear = 0.1f;
    o->squeeze = 1.f;
}

lc_meters lc_optics_hyperfocal_distance(lc_optics* o, lc_millimeters CoC)
{
    return { mm_as_m(o->focal_length).m * mm_as_m(o->focal_length).m / (o->fStop * mm_as_m(CoC).m) };
}

lc_v2f lc_optics_focus_range(lc_optics* o, lc_millimeters h)
{
    lc_v2f r;
    float h_m = mm_as_m(h).m;
    r.x = h_m * o->focus_distance.m / (h_m + (o->focus_distance.m - mm_as_m(o->focal_length).m));
    r.y = h_m * o->focus_distance.m / (h_m - (o->focus_distance.m - mm_as_m(o->focal_length).m));
    return r;
}

//-----------------------------------------------------------------------------
// lc_sensor
//
//-----------------------------------------------------------------------------

void lc_sensor_set_default(lc_sensor* s)
{
    s->shift.x.mm = 0;
    s->shift.y.mm = 0;
    s->handedness = -1.f;       // left handed
    s->aperture.x.mm = 35.f;
    s->aperture.y.mm = 24.5f;
    s->enlarge.x = 1.f;
    s->enlarge.y = 1.f;
}

lc_millimeters lc_sensor_focal_length_from_vertical_FOV(lc_sensor* s, lc_radians fov)
{
    if (fov.rad < 0 || fov.rad > 3.141592653589793238f)
        return lc_millimeters{ 0.f };

    float f = (s->aperture.y.mm / (2 * tanf(0.5f * fov.rad)));
    return { f / s->enlarge.y };
}

//-----------------------------------------------------------------------------
// Camera
//
//-----------------------------------------------------------------------------

void lc_camera_set_defaults(lc_camera* cam)
{
    lc_optics_set_default(&cam->optics);
    lc_aperture_set_default(&cam->aperture);
    lc_sensor_set_default(&cam->sensor);
    lc_mount_set_default(&cam->mount);
}

lc_m44f lc_camera_perspective(const lc_camera* cam, float aspect)
{
    if (fabs(aspect) < std::numeric_limits<float>::epsilon())
        return m44f_identity;

    const float handedness = cam->sensor.handedness; // -1 for left hand coordinates
    float left = -1.f, right = 1.f, bottom = -1.f, top = 1.f;
    const float halfFovy = lc_camera_vertical_FOV(cam).rad * 0.5f;
    const float y = 1.f / tanf(halfFovy);
    const float x = y / aspect / cam->optics.squeeze;
    const float scalex = 2.f * cam->sensor.enlarge.x;
    const float scaley = 2.f * cam->sensor.enlarge.y;
    const float dx = mm_as_m(cam->sensor.shift.x).m * 2.f * aspect / mm_as_m(cam->sensor.aperture.y).m;
    const float dy = mm_as_m(cam->sensor.shift.y).m * 2.f / mm_as_m(cam->sensor.aperture.y).m;

    const float znear = cam->optics.znear;
    const float zfar = cam->optics.zfar;

    lc_m44f result;
    memset(&result, 0, sizeof(lc_m44f));
    result.x.x = scalex * x / (right - left);
    result.y.y = scaley * y / (top - bottom);
    result.z.x = (right + left + dx) / (right - left);
    result.z.y = (top + bottom + dy) / (top - bottom);
    result.z.z = handedness * (zfar + znear) / (zfar - znear);
    result.z.w = handedness;
    result.w.z = handedness * 2.f * zfar * znear / (zfar - znear);
    return { result };
}

lc_m44f lc_camera_inv_perspective(const lc_camera* cam, float aspect)
{
    return invert(lc_camera_perspective(cam, aspect));
}

lc_radians lc_camera_vertical_FOV(const lc_camera* cam)
{
    float cropped_f = cam->optics.focal_length.mm * cam->sensor.enlarge.y;
    return { 2.f * std::atanf(cam->sensor.aperture.y.mm / (2.f * cropped_f)) };
}

lc_radians lc_camera_horizontal_FOV(const lc_camera* cam)
{
    float cropped_f = cam->optics.focal_length.mm * cam->sensor.enlarge.y;
    return { 2.f * std::atanf(cam->optics.squeeze * cam->sensor.aperture.x.mm / (2.f * cropped_f)) };
}

// move the camera along the view vector such that both bounds are visible
void lc_camera_frame(lc_camera* cam, lc_v3f bound1, lc_v3f bound2)
{
    const lc_rigid_transform* cmt = &cam->mount.transform;
    float r = 0.5f * length(bound2 - bound1);
    float g = (1.1f * r) / sinf(lc_camera_vertical_FOV(cam).rad * 0.5f);
    lc_v3f focus_point = (bound2 + bound1) * 0.5f;
    lc_v3f position = normalize(cmt->position - focus_point) * g;
    lc_mount_look_at(&cam->mount, position, focus_point, lc_rt_up(cmt));
}

void lc_camera_set_clipping_planes_within_bounds(lc_camera* cam, float min_near, float max_far, lc_v3f bound1, lc_v3f bound2)
{
    float clip_near = FLT_MAX;
    float clip_far = FLT_MIN;

    lc_v4f points[8] = {
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
        lc_v4f dp = mul(lc_mount_gl_view_transform(&cam->mount), points[p]);
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

    cam->optics.znear = clip_near;
    cam->optics.zfar = clip_far;
}

float lc_camera_distance_to_plane(const lc_camera* cam, lc_v3f plane_point, lc_v3f plane_normal)
{
    const lc_rigid_transform* cmt = &cam->mount.transform;
    float denom = dot(plane_normal, lc_rt_forward(cmt));
    if (denom > 1.e-6f) {
        lc_v3f p0 = plane_point - cmt->position;
        return dot(p0, plane_normal) / denom;
    }
    return FLT_MAX; // ray and plane are parallel
}

lc_m44f lc_camera_view_projection(const lc_camera* cam, float aspect)
{
    lc_m44f proj = lc_camera_perspective(cam, aspect);
    lc_m44f view = lc_mount_gl_view_transform(&cam->mount);
    return mul(proj, view);
}

lc_m44f lc_camera_inv_view_projection(const lc_camera* cam, float aspect)
{
    lc_m44f proj = lc_camera_perspective(cam, aspect);
    lc_m44f view = lc_mount_gl_view_transform(&cam->mount);
    return invert(mul(proj, view));
}

lc_ray lc_camera_get_ray_from_pixel(const lc_camera* cam, lc_v2f pixel, lc_v2f viewport_origin, lc_v2f viewport_size)
{
    const lc_rigid_transform* cmt = &cam->mount.transform;
    lc_m44f inv_projection = lc_camera_inv_view_projection(cam, 1.f);
    return get_ray(inv_projection, cmt->position, pixel, viewport_origin, viewport_size);
}

lc_hit_result lc_camera_hit_test(const lc_camera* cam, lc_v2f mouse, lc_v2f viewport, lc_v3f plane_point, lc_v3f plane_normal)
{
    lc_ray ray = lc_camera_get_ray_from_pixel(cam, mouse, { 0, 0 }, viewport);
    lc_hit_result r;
    r.hit = intersect_ray_plane(ray, plane_point, plane_normal, &r.point);
    return r;
}

lc_v2f lc_camera_project_to_viewport(const lc_camera* cam, lc_v2f viewport_origin, lc_v2f viewport_size, lc_v3f point)
{
    lc_m44f m = lc_camera_view_projection(cam, 1.f);

    lc_v4f p = mul(m, lc_v4f{ point.x, point.y, point.z, 1.f });
    lc_v3f pnt = xyz(mul(p, 1.f / p.w));
    pnt.x = pnt.x * viewport_size.x * 0.5f + viewport_size.x * 0.5f;
    pnt.y = pnt.y * viewport_size.y * -0.5f + viewport_size.y * 0.5f;
    pnt.x -= viewport_origin.x;
    pnt.y -= viewport_origin.y;
    return { pnt.x, pnt.y };
}
