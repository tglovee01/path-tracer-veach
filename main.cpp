#include "ray_tracer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr double kPi = 3.14159265358979323846;

Vector3 multiply(const Vector3& a, const Vector3& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

bool is_emissive(const Sphere& sphere) {
    return sphere.emission.x > 0.0 || sphere.emission.y > 0.0 || sphere.emission.z > 0.0;
}

bool has_emission(const Vector3& emission) {
    return emission.x > 0.0 || emission.y > 0.0 || emission.z > 0.0;
}

std::size_t light_count(const std::vector<Sphere>& scene) {
    std::size_t count = 0;
    for (const Sphere& sphere : scene) count += is_emissive(sphere) ? 1u : 0u;
    return count;
}

double light_pdf_for_hit(const Vector3& origin, const Ray& ray, const HitRecord& light_hit,
                         std::size_t number_of_lights) {
    if (number_of_lights == 0 || !light_hit.front_face || light_hit.sphere_area <= 0.0) {
        return 0.0;
    }
    const Vector3 direction = ray.direction.normalized();
    const double cosine_at_light = -direction.dot(light_hit.normal);
    if (cosine_at_light <= 0.0) return 0.0;
    const double distance_squared = (light_hit.point - origin).length_squared();
    return distance_squared /
           (cosine_at_light * light_hit.sphere_area * number_of_lights);
}

Vector3 estimate_direct_light(const HitRecord& surface, const std::vector<Sphere>& scene,
                              std::size_t number_of_lights, RandomGenerator& rng) {
    if (number_of_lights == 0) return {};

    std::size_t selected = static_cast<std::size_t>(rng.uniform(0.0, number_of_lights));
    const Sphere* light = nullptr;
    for (const Sphere& sphere : scene) {
        if (!is_emissive(sphere)) continue;
        if (selected-- == 0) {
            light = &sphere;
            break;
        }
    }
    if (!light) return {};

    const double z = rng.uniform(-1.0, 1.0);
    const double phi = 2.0 * kPi * rng.uniform();
    const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
    const Vector3 light_normal{radial * std::cos(phi), radial * std::sin(phi), z};
    const Vector3 light_point = light->center + light_normal * light->radius;
    const Vector3 offset = light_point - surface.point;
    const double distance_squared = offset.length_squared();
    if (distance_squared <= 1e-12) return {};

    const double distance = std::sqrt(distance_squared);
    const Vector3 direction = offset / distance;
    const double cosine_surface = std::max(0.0, surface.normal.dot(direction));
    const double cosine_light = std::max(0.0, light_normal.dot(-direction));
    if (cosine_surface <= 0.0 || cosine_light <= 0.0) return {};

    const double area = 4.0 * kPi * light->radius * light->radius;
    const double pdf_light = distance_squared /
                             (cosine_light * area * number_of_lights);
    const double pdf_bsdf = cosine_surface / kPi;
    if (pdf_light <= 0.0) return {};

    const Ray shadow_ray{surface.point + surface.normal * 1e-4, direction};
    HitRecord blocker;
    if (hit(shadow_ray, scene, 1e-4, distance - 2e-4, blocker)) return {};

    const double weight = power_heuristic(pdf_light, pdf_bsdf);
    const Vector3 diffuse_brdf = surface.albedo / kPi;
    return multiply(light->emission, diffuse_brdf) *
           (cosine_surface * weight / pdf_light);
}

Vector3 trace_path(Ray ray, const std::vector<Sphere>& scene, RandomGenerator& rng) {
    Vector3 radiance{};
    Vector3 throughput{1.0, 1.0, 1.0};
    const std::size_t number_of_lights = light_count(scene);
    bool previous_bounce_was_diffuse = false;
    Vector3 previous_point{};
    double previous_bsdf_pdf = 0.0;

    for (int depth = 0; depth < 64; ++depth) {
        HitRecord record;
        if (!hit(ray, scene, 1e-4, std::numeric_limits<double>::infinity(), record)) {
            const Vector3 direction = ray.direction.normalized();
            const double blend = 0.5 * (direction.y + 1.0);
            const Vector3 sky = Vector3{1.0, 1.0, 1.0} * (1.0 - blend) +
                                Vector3{0.5, 0.7, 1.0} * blend;
            radiance = radiance + multiply(throughput, sky);
            break;
        }

        if (has_emission(record.emission)) {
            double emission_weight = 1.0;
            if (depth > 0 && previous_bounce_was_diffuse) {
                const double pdf_light = light_pdf_for_hit(previous_point, ray, record,
                                                           number_of_lights);
                emission_weight = power_heuristic(previous_bsdf_pdf, pdf_light);
            }
            radiance = radiance + multiply(throughput, record.emission) * emission_weight;
            break;
        }

        if (std::dynamic_pointer_cast<const Lambertian>(record.material)) {
            radiance = radiance + multiply(
                throughput, estimate_direct_light(record, scene, number_of_lights, rng));
        }
        Vector3 attenuation;
        Ray scattered;
        if (record.material) {
            if (!record.material->scatter(ray, record, rng, attenuation, scattered)) break;
        } else {
            if (record.albedo.length_squared() == 0.0) break;
            attenuation = record.albedo;
            scattered = Ray{record.point + record.normal * 1e-4,
                            random_cosine_direction(rng, record.normal)};
        }
        throughput = multiply(throughput, attenuation);
        if (depth >= 3) {
            const double survival = std::clamp(
                std::max({throughput.x, throughput.y, throughput.z}), 0.05, 0.95);
            if (rng.uniform() >= survival) break;
            throughput = throughput / survival;
        }
        previous_bounce_was_diffuse =
            static_cast<bool>(std::dynamic_pointer_cast<const Lambertian>(record.material)) ||
            !record.material;
        previous_point = record.point;
        previous_bsdf_pdf = std::max(0.0, record.normal.dot(scattered.direction.normalized())) / kPi;
        ray = scattered;
    }
    return radiance;
}

int to_byte(double linear) {
    const double gamma_corrected = std::sqrt(std::max(0.0, linear));
    return static_cast<int>(256.0 * std::clamp(gamma_corrected, 0.0, 0.999));
}

void write_u16_le(std::ostream& out, std::uint16_t value) {
    out.put(static_cast<char>(value & 0xff));
    out.put(static_cast<char>((value >> 8) & 0xff));
}

void write_u32_le(std::ostream& out, std::uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
        out.put(static_cast<char>((value >> shift) & 0xff));
}

bool write_bmp(const std::string& path, const std::vector<Vector3>& pixels,
               int width, int height) {
    const std::uint32_t row_size = (static_cast<std::uint32_t>(width) * 3u + 3u) & ~3u;
    const std::uint32_t data_size = row_size * static_cast<std::uint32_t>(height);
    constexpr std::uint32_t header_size = 54;
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.put('B'); out.put('M');
    write_u32_le(out, header_size + data_size);
    write_u16_le(out, 0); write_u16_le(out, 0);
    write_u32_le(out, header_size);
    write_u32_le(out, 40);
    write_u32_le(out, static_cast<std::uint32_t>(width));
    write_u32_le(out, static_cast<std::uint32_t>(height));
    write_u16_le(out, 1); write_u16_le(out, 24);
    write_u32_le(out, 0); write_u32_le(out, data_size);
    write_u32_le(out, 2835); write_u32_le(out, 2835);
    write_u32_le(out, 0); write_u32_le(out, 0);
    const std::uint32_t padding = row_size - static_cast<std::uint32_t>(width) * 3u;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const auto& p = pixels[static_cast<std::size_t>(y) * width + x];
            out.put(static_cast<char>(to_byte(p.z)));
            out.put(static_cast<char>(to_byte(p.y)));
            out.put(static_cast<char>(to_byte(p.x)));
        }
        for (std::uint32_t i = 0; i < padding; ++i) out.put(0);
    }
    return static_cast<bool>(out);
}

bool verify_russian_roulette_energy() {
    constexpr int samples = 1'000'000;
    constexpr Vector3 throughput{0.7, 0.5, 0.25};
    constexpr double survival = 0.7;
    RandomGenerator rng{0x5EEDu};
    Vector3 total{};
    for (int i = 0; i < samples; ++i)
        if (rng.uniform() < survival) total = total + throughput / survival;
    const Vector3 mean = total / samples;
    constexpr double tolerance = 0.002;
    return std::abs(mean.x - throughput.x) < tolerance &&
           std::abs(mean.y - throughput.y) < tolerance &&
           std::abs(mean.z - throughput.z) < tolerance;
}
} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--verify-energy") {
        if (!verify_russian_roulette_energy()) {
            std::cerr << "Russian roulette energy check failed\n";
            return 1;
        }
        std::cout << "Russian roulette energy check passed (1,000,000 samples)\n";
        return 0;
    }
    const std::string output_path = argc > 1 ? argv[1] : "render.bmp";
    constexpr int image_width = 320;
    constexpr int image_height = 200;
    constexpr int samples_per_pixel = 64;

    // A diffuse ground and object with a visible spherical area emitter.
    const std::vector<Sphere> scene{
        {{0.0, -100.5, -4.0}, 100.0, {0.75, 0.75, 0.75}, {},
         std::make_shared<Lambertian>(Vector3{0.75, 0.75, 0.75})},
        {{-0.75, 0.0, -4.0}, 0.5, {0.82, 0.58, 0.24}, {},
         std::make_shared<Metal>(Vector3{0.82, 0.58, 0.24}, 0.08)},
        {{0.65, 0.0, -4.0}, 0.5, {1.0, 1.0, 1.0}, {},
         std::make_shared<Dielectric>(1.5)},
        {{-1.2, 2.2, -3.6}, 0.55, {}, {12.0, 10.0, 8.0}},
        {{1.35, 1.5, -4.5}, 0.28, {}, {32.0, 28.0, 22.0}}
    };

    const Vector3 camera_origin{0.0, 0.6, 1.5};
    const Vector3 camera_target{0.0, 0.2, -4.0};
    const Vector3 forward = (camera_target - camera_origin).normalized();
    const Vector3 right = forward.cross(Vector3{0.0, 1.0, 0.0}).normalized();
    const Vector3 up = right.cross(forward).normalized();
    const double aspect = static_cast<double>(image_width) / image_height;
    const double viewport_height = 2.0 * std::tan((45.0 * kPi / 180.0) / 2.0);
    const double viewport_width = aspect * viewport_height;

    unsigned int thread_count = std::thread::hardware_concurrency();
    if (thread_count == 0) thread_count = 1;
    if (argc > 2) {
        try { thread_count = static_cast<unsigned int>(std::stoul(argv[2])); }
        catch (const std::exception&) {
            std::cerr << "Thread count must be a positive integer\n";
            return 2;
        }
        if (thread_count == 0) {
            std::cerr << "Thread count must be a positive integer\n";
            return 2;
        }
    }

    const std::size_t pixel_count = static_cast<std::size_t>(image_width) * image_height;
    std::vector<Vector3> pixels(pixel_count);
    std::atomic<std::size_t> next_pixel{0};
    const auto start_time = std::chrono::steady_clock::now();
    auto render_worker = [&]() {
        for (;;) {
            const std::size_t index = next_pixel.fetch_add(1, std::memory_order_relaxed);
            if (index >= pixel_count) break;
            const int x = static_cast<int>(index % image_width);
            const int y = image_height - 1 - static_cast<int>(index / image_width);
            RandomGenerator rng{0xC0FFEEu + static_cast<std::uint64_t>(index)};
            Vector3 pixel{};
            for (int sample = 0; sample < samples_per_pixel; ++sample) {
                const double u = (x + rng.uniform()) / (image_width - 1.0);
                const double v = (y + rng.uniform()) / (image_height - 1.0);
                const Vector3 direction = forward + right * ((u - 0.5) * viewport_width) +
                                          up * ((v - 0.5) * viewport_height);
                pixel = pixel + trace_path(Ray{camera_origin, direction}, scene, rng);
            }
            pixels[index] = pixel / samples_per_pixel;
        }
    };
    thread_count = std::min(thread_count, static_cast<unsigned int>(pixel_count));
    std::vector<std::thread> workers;
    workers.reserve(thread_count);
    for (unsigned int i = 0; i < thread_count; ++i) workers.emplace_back(render_worker);
    for (std::thread& worker : workers) worker.join();
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    if (!write_bmp(output_path, pixels, image_width, image_height)) {
        std::cerr << "Failed while writing BMP image: " << output_path << '\n';
        return 1;
    }
    std::cout << "Rendered " << image_width << 'x' << image_height << " image to "
              << output_path << " using " << thread_count << " thread(s) in "
              << elapsed << " s\n";
    return 0;
}
