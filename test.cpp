#include <cmath>
#include <iostream>
#include <stdexcept>

#include "ray_tracer.h"

int main() {
    const Vector3 x_axis{1.0, 0.0, 0.0};
    const Vector3 y_axis{0.0, 1.0, 0.0};
    const Vector3 cross = x_axis.cross(y_axis);
    const Vector3 normalized = Vector3{3.0, 0.0, 4.0}.normalized();
    const Ray ray{{0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};
    const Sphere sphere{{0.0, 0.0, -5.0}, 1.0};
    double t_hit = 0.0;
    RandomGenerator rng{12345};

    if (x_axis.dot(y_axis) != 0.0 || cross.x != 0.0 || cross.y != 0.0 ||
        cross.z != 1.0 || std::abs(normalized.length() - 1.0) > 1e-12 ||
        ray.at(2.0).z != -2.0 || !hit_sphere(ray, sphere, t_hit) ||
        std::abs(t_hit - 4.0) > 1e-12) {
        std::cerr << "Phase 1 smoke check failed\n";
        return 1;
    }

    const double sample = rng.uniform();
    if (sample < 0.0 || sample >= 1.0) {
        std::cerr << "Random sample is outside [0, 1)\n";
        return 1;
    }

    const double balance_a = balance_heuristic(0.25, 0.75);
    const double balance_b = balance_heuristic(0.75, 0.25);
    const double power_a = power_heuristic(0.25, 0.75);
    const double power_b = power_heuristic(0.75, 0.25);
    Vector3 refracted;
    if (std::abs(balance_a + balance_b - 1.0) > 1e-12 ||
        std::abs(power_a + power_b - 1.0) > 1e-12 ||
        std::abs(balance_a - 0.25) > 1e-12 ||
        std::abs(power_a - 0.1) > 1e-12 ||
        !refract(Vector3{0.0, -1.0, 0.0}, Vector3{0.0, 1.0, 0.0}, 1.0 / 1.5,
                 refracted) || std::abs(refracted.y + 1.0) > 1e-12) {
        std::cerr << "MIS heuristic or refraction check failed\n";
        return 1;
    }

    std::cout << "Phase 1 and Phase 4 smoke checks passed\n";
    return 0;
}
