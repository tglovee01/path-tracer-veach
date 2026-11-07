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

    std::cout << "Phase 1 smoke check passed\n";
    return 0;
}
