#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
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

    [[nodiscard]] constexpr Vector3 operator-() const { return {-x, -y, -z}; }

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

[[nodiscard]] inline Vector3 reflect(const Vector3& direction, const Vector3& normal) {
    return direction - normal * (2.0 * direction.dot(normal));
}

[[nodiscard]] inline bool refract(const Vector3& direction, const Vector3& normal,
                                  double eta_ratio, Vector3& refracted) {
    const double cos_theta = std::min((-direction).dot(normal), 1.0);
    const Vector3 perpendicular = (direction + normal * cos_theta) * eta_ratio;
    const double parallel_squared = 1.0 - perpendicular.length_squared();
    if (parallel_squared < 0.0) {
        return false;
    }
    refracted = perpendicular - normal * std::sqrt(parallel_squared);
    return true;
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

struct HitRecord;
class RandomGenerator;

class Material {
public:
    virtual ~Material() = default;
    [[nodiscard]] virtual bool scatter(const Ray& incoming, const HitRecord& hit,
                                       RandomGenerator& rng, Vector3& attenuation,
                                       Ray& scattered) const = 0;
};

class Lambertian final : public Material {
public:
    explicit Lambertian(Vector3 albedo) : albedo_(albedo) {}
    [[nodiscard]] bool scatter(const Ray&, const HitRecord& hit, RandomGenerator& rng,
                               Vector3& attenuation, Ray& scattered) const override;
private:
    Vector3 albedo_;
};

class Metal final : public Material {
public:
    explicit Metal(Vector3 albedo, double fuzz = 0.0)
        : albedo_(albedo), fuzz_(std::clamp(fuzz, 0.0, 1.0)) {}
    [[nodiscard]] bool scatter(const Ray& incoming, const HitRecord& hit,
                               RandomGenerator& rng, Vector3& attenuation,
                               Ray& scattered) const override;
private:
    Vector3 albedo_;
    double fuzz_;
};

class Dielectric final : public Material {
public:
    explicit Dielectric(double index_of_refraction) : refraction_index_(index_of_refraction) {
        if (index_of_refraction <= 0.0) {
            throw std::invalid_argument("Index of refraction must be positive");
        }
    }
    [[nodiscard]] bool scatter(const Ray& incoming, const HitRecord& hit,
                               RandomGenerator& rng, Vector3& attenuation,
                               Ray& scattered) const override;
private:
    double refraction_index_;
};

struct Sphere {
    Vector3 center;
    double radius{};
    Vector3 albedo{0.8, 0.8, 0.8};
    Vector3 emission{};
    std::shared_ptr<const Material> material{};
};

struct HitRecord {
    Vector3 point;
    Vector3 normal;
    Vector3 albedo;
    Vector3 emission;
    std::shared_ptr<const Material> material;
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

inline Vector3 random_in_unit_sphere(RandomGenerator& rng) {
    for (;;) {
        const Vector3 point{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0)};
        if (point.length_squared() < 1.0) return point;
    }
}

inline Vector3 random_cosine_direction(RandomGenerator& rng, const Vector3& normal) {
    constexpr double pi = 3.14159265358979323846;
    const double r1 = rng.uniform();
    const double r2 = rng.uniform();
    const double phi = 2.0 * pi * r1;
    const double radius = std::sqrt(r2);
    const double x = radius * std::cos(phi);
    const double y = radius * std::sin(phi);
    const double z = std::sqrt(1.0 - r2);
    const Vector3 helper = std::abs(normal.z) < 0.999
        ? Vector3{0.0, 0.0, 1.0} : Vector3{0.0, 1.0, 0.0};
    const Vector3 tangent = helper.cross(normal).normalized();
    const Vector3 bitangent = normal.cross(tangent);
    return (tangent * x + bitangent * y + normal * z).normalized();
}

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
    record.material = sphere.material;
    record.set_face_normal(ray, (record.point - sphere.center) / sphere.radius);
    return true;
}

inline bool Lambertian::scatter(const Ray&, const HitRecord& hit, RandomGenerator& rng,
                                Vector3& attenuation, Ray& scattered) const {
    attenuation = albedo_;
    scattered = Ray{hit.point + hit.normal * 1e-4,
                    random_cosine_direction(rng, hit.normal)};
    return true;
}

inline bool Metal::scatter(const Ray& incoming, const HitRecord& hit, RandomGenerator& rng,
                           Vector3& attenuation, Ray& scattered) const {
    const Vector3 reflected = reflect(incoming.direction.normalized(), hit.normal);
    const Vector3 direction = reflected + random_in_unit_sphere(rng) * fuzz_;
    if (direction.dot(hit.normal) <= 0.0) {
        return false;
    }
    attenuation = albedo_;
    scattered = Ray{hit.point + hit.normal * 1e-4, direction.normalized()};
    return true;
}

inline bool Dielectric::scatter(const Ray& incoming, const HitRecord& hit, RandomGenerator& rng,
                                Vector3& attenuation, Ray& scattered) const {
    attenuation = {1.0, 1.0, 1.0};
    const double eta_ratio = hit.front_face ? 1.0 / refraction_index_ : refraction_index_;
    const Vector3 unit_direction = incoming.direction.normalized();
    const double cos_theta = std::min((-unit_direction).dot(hit.normal), 1.0);
    const double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
    Vector3 direction;
    const double r0_base = (1.0 - eta_ratio) / (1.0 + eta_ratio);
    const double r0 = r0_base * r0_base;
    const double reflectance = r0 + (1.0 - r0) * std::pow(1.0 - cos_theta, 5.0);
    if (eta_ratio * sin_theta > 1.0 || rng.uniform() < reflectance) {
        direction = reflect(unit_direction, hit.normal);
    } else if (!refract(unit_direction, hit.normal, eta_ratio, direction)) {
        direction = reflect(unit_direction, hit.normal);
    }
    scattered = Ray{hit.point + direction * 1e-4, direction.normalized()};
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
