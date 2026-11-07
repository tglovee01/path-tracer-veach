#include <iostream>

#include "ray_tracer.h"

int main() {
    const Ray ray{{0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};
    const Sphere sphere{{0.0, 0.0, -5.0}, 1.0};

    double t_hit = 0.0;
    if (hit_sphere(ray, sphere, t_hit)) {
        std::cout << "Ray hit the sphere at t = " << t_hit << '\n';
    } else {
        std::cout << "Ray missed the sphere.\n";
    }

    return 0;
}
