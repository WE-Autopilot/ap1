/**
 * Class for 2d and 3d vectos
 * Created: Oct. 11, 2025
 * Author(s): Logan Ouellette
 */

#ifndef AP1_VECTOR_HPP
#define AP1_VECTOR_HPP

#include <format>
#include <math.h>
#include <stdexcept>
#include <stdlib.h>

template <typename T>
float distance(const T &v1, const T &v2) {
    return std::sqrt(
        std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2)
    );
}

// 2D vector class
class vec2f
{
  public:
    float x, y;

    vec2f() : x(0.f), y(0.f) {}
    vec2f(float x, float y) : x(x), y(y) {};

    inline vec2f operator-() const
    {
        return vec2f(-x, -y);
    }

    inline float operator[](const int i) const
    {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else
        {
            throw std::out_of_range(std::format("Index %d out of bounds for vec3f", i));
        }
    }

    inline vec2f operator+(const vec2f& v) const
    {
        return vec2f(x + v.x, y + v.y);
    }

    inline vec2f operator-(const vec2f& v) const
    {
        return vec2f(x - v.x, y - v.y);
    }

    inline vec2f operator*(const float f) const
    {
        return vec2f(x * f, y * f);
    }

    inline vec2f operator/(const float f) const
    {
        return vec2f(x / f, y / f);
    }

    inline vec2f& operator+=(const vec2f& v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    inline vec2f& operator-=(const vec2f& v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    inline vec2f& operator*=(const float t)
    {
        x *= t;
        y *= t;
        return *this;
    }

    inline vec2f& operator/=(const float t)
    {
        x /= t;
        y /= t;
        return *this;
    }

    inline float length() const
    {
        return sqrt(x * x + y * y);
    }

    inline float squared_length() const
    {
        return x * x + y * y;
    }

    inline void make_unit_vector()
    {
        auto len = this->length();
        x /= len;
        y /= len;
    }

    inline vec2f unit_vector() const
    {
        auto len = this->length();
        return vec2f(x / len, y / len);
    }

    inline vec2f dot(const vec2f& v) const
    {
        return vec2f(x * v.x, y * v.y);
    }

    inline float cross(const vec2f& v) const
    {
        return x * v.y - y * v.x;
    }
};

// 3D vector class
class vec3f
{
  public:
    float x, y, z;

    vec3f() : x(0.f), y(0.f), z(0.f) {}
    vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
    vec3f(const vec2f& v): x(v.x), y(v.y), z(0) {}

    float operator[](int i) const
    {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else
        {
            throw std::out_of_range(std::format("Index %d out of bounds for vec3f", i));
        }
    }

    inline vec3f operator-() const
    {
        return vec3f(-x, -y, -z);
    }

    inline vec3f operator+(const vec3f& v) const
    {
        return vec3f(x + v.x, y + v.y, z + v.z);
    }

    inline vec3f operator-(const vec3f& v) const
    {
        return vec3f(x - v.x, y - v.y, z - v.z);
    }

    inline vec3f operator*(const float f) const
    {
        return vec3f(x * f, y * f, z * f);
    }

    inline vec3f operator/(const float f) const
    {
        return vec3f(x / f, y / f, z / f);
    }

    inline vec3f& operator+=(const vec3f& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    inline vec3f& operator-=(const vec3f& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    inline vec3f& operator*=(const float t)
    {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    inline vec3f& operator/=(const float t)
    {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }

    inline float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    inline float squared_length() const
    {
        return x * x + y * y + z * z;
    }

    inline void make_unit_vector()
    {
        auto len = this->length();
        x /= len;
        y /= len;
        z /= len;
    }

    inline vec3f unit_vector() const
    {
        auto len = this->length();
        return vec3f(x / len, y / len, z / len);
    }

    inline vec3f dot(const vec3f& v) const
    {
        return vec3f(x * v.x, y * v.y, z * v.z);
    }

    inline vec3f cross(const vec3f& v) const
    {
        return vec3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
};

// free function overloads
inline vec2f operator*(float s, const vec2f& v) {
    return v * s;
}

inline vec3f operator*(float s, const vec3f& v) {
    return v * s;
}

inline vec2f operator/(float s, const vec2f& v) {
    return v / s;
}

inline vec3f operator/(float s, const vec3f& v) {
    return v / s;
}

#endif
