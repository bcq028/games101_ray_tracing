#pragma once

#include <cmath>
#include <iostream>

class Vector3f {
public:
  Vector3f() : x(0), y(0), z(0) {}
  Vector3f(float xx) : x(xx), y(xx), z(xx) {}
  Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
  Vector3f operator*(const float &r) const {
    return Vector3f(x * r, y * r, z * r);
  }
  Vector3f operator/(const float &r) const {
    return Vector3f(x / r, y / r, z / r);
  }

  Vector3f operator*(const Vector3f &v) const {
    return Vector3f(x * v.x, y * v.y, z * v.z);
  }
  Vector3f operator-(const Vector3f &v) const {
    return Vector3f(x - v.x, y - v.y, z - v.z);
  }
  Vector3f operator+(const Vector3f &v) const {
    return Vector3f(x + v.x, y + v.y, z + v.z);
  }
  Vector3f operator-() const { return Vector3f(-x, -y, -z); }
  Vector3f &operator+=(const Vector3f &v) {
    x += v.x, y += v.y, z += v.z;
    return *this;
  }
  friend Vector3f operator*(const float &r, const Vector3f &v) {
    return Vector3f(v.x * r, v.y * r, v.z * r);
  }
  friend std::ostream &operator<<(std::ostream &os, const Vector3f &v) {
    return os << v.x << ", " << v.y << ", " << v.z;
  }
  float x, y, z;
};

inline double length(const Vector3f &a){
  return sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
}

class Vector2f {
public:
  Vector2f() : x(0), y(0) {}
  Vector2f(float xx) : x(xx), y(xx) {}
  Vector2f(float xx, float yy) : x(xx), y(yy) {}
  Vector2f operator*(const float &r) const { return Vector2f(x * r, y * r); }
  Vector2f operator+(const Vector2f &v) const {
    return Vector2f(x + v.x, y + v.y);
  }
  float x, y;
};

inline double length(const Vector2f &a){
  return sqrt(a.x*a.x+a.y*a.y);
}

inline Vector3f lerp(const Vector3f &a, const Vector3f &b, const float &t) {
  return a+(b-a)*t;
}

inline Vector3f normalize(const Vector3f &v) {
  return v/length(v);
}

inline float dotProduct(const Vector3f &a, const Vector3f &b) {
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

inline Vector3f crossProduct(const Vector3f &a, const Vector3f &b) {
    Vector3f ret;
    ret.x=a.y*b.z-a.z*b.y;
    ret.y=a.z*b.x-a.x*b.z;
    ret.z=a.x*b.y-a.y*b.x;
    return ret;
}

