#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

struct Vector3 {
    double x{};
    double y{};
    double z{};

    constexpr Vector3() = default;
    constexpr Vector3(double x_value, double y_value, double z_value)
        : x(x_value), y(y_value), z(z_value) {}

    [[nodiscard]] constexpr Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr Vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    [[nodiscard]] constexpr double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    [[nodiscard]] constexpr Vector3 cross(const Vector3& other) const {
        return {y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
    }

    [[nodiscard]] constexpr double length_squared() const { return dot(*this); }
    [[nodiscard]] double length() const { return std::sqrt(length_squared()); }

    [[nodiscard]] Vector3 normalized() const {
        const double magnitude = length();
        if (magnitude == 0.0) {
            throw std::domain_error("Cannot normalize a zero-length Vector3");
        }
        return *this / magnitude;
    }
};

// Keep the previous type name available for existing callers.
using Vec3 = Vector3;

[[nodiscard]] constexpr Vector3 operator*(double scalar, const Vector3& vector) {
    return vector * scalar;
}

struct Ray {
    Vector3 origin;
    Vector3 direction;

    [[nodiscard]] constexpr Vector3 at(double t) const {
        return {origin.x + t * direction.x,
                origin.y + t * direction.y,
                origin.z + t * direction.z};
    }
};

struct Sphere {
    Vector3 center;
    double radius{};
    Vector3 albedo{0.8, 0.8, 0.8};
    Vector3 emission{};
};

struct HitRecord {
    Vector3 point;
    Vector3 normal;
    Vector3 albedo;
    Vector3 emission;
    double t{};
    bool front_face{};

    void set_face_normal(const Ray& ray, const Vector3& outward_normal) {
        front_face = ray.direction.dot(outward_normal) < 0.0;
        normal = front_face ? outward_normal : outward_normal * -1.0;
    }
};

class RandomGenerator {
public:
    RandomGenerator() : engine_(std::random_device{}()) {}
    explicit RandomGenerator(std::uint64_t seed) : engine_(seed) {}

    // Generates a uniformly distributed value in [0, 1).
    [[nodiscard]] double uniform() {
        return std::generate_canonical<double, 53>(engine_);
    }

    [[nodiscard]] double uniform(double min_value, double max_value) {
        if (min_value > max_value) {
            throw std::invalid_argument("Random range minimum exceeds maximum");
        }
        return std::uniform_real_distribution<double>(min_value, max_value)(engine_);
    }

private:
    std::mt19937_64 engine_;
};

// Tests the nearest sphere root in [t_min, t_max] and fills the surface record.
[[nodiscard]] inline bool hit(const Ray& ray, const Sphere& sphere,
                              double t_min, double t_max, HitRecord& record) {
    const Vector3 offset = ray.origin - sphere.center;
    const double a = ray.direction.dot(ray.direction);
    const double half_b = offset.dot(ray.direction);
    const double c = offset.dot(offset) - sphere.radius * sphere.radius;
    const double discriminant = half_b * half_b - a * c;

    if (a == 0.0 || sphere.radius <= 0.0 || discriminant < 0.0) {
        return false;
    }

    const double root = std::sqrt(discriminant);
    double t = (-half_b - root) / a;
    if (t < t_min || t > t_max) {
        t = (-half_b + root) / a;
        if (t < t_min || t > t_max) {
            return false;
        }
    }

    record.t = t;
    record.point = ray.at(t);
    record.albedo = sphere.albedo;
    record.emission = sphere.emission;
    record.set_face_normal(ray, (record.point - sphere.center) / sphere.radius);
    return true;
}

[[nodiscard]] inline bool hit(const Ray& ray, const std::vector<Sphere>& spheres,
                              double t_min, double t_max, HitRecord& record) {
    HitRecord candidate;
    bool found = false;
    double closest = t_max;
    for (const Sphere& sphere : spheres) {
        if (hit(ray, sphere, t_min, closest, candidate)) {
            found = true;
            closest = candidate.t;
            record = candidate;
        }
    }
    return found;
}

// Retained for the Phase 1 smoke check.
[[nodiscard]] inline bool hit_sphere(const Ray& ray, const Sphere& sphere, double& t_hit) {
    HitRecord record;
    if (!hit(ray, sphere, 0.0, std::numeric_limits<double>::infinity(), record)) {
        return false;
    }
    t_hit = record.t;
    return true;
}
