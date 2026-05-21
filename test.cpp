#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

// ----------------------------------------------------------------------
// 3D vector with the necessary operations
struct Vector3 {
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vector3 operator-(const Vector3 &v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }

    // Cross product
    Vector3 cross(const Vector3 &v) const {
        return {y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x};
    }

    double dot(const Vector3 &v) const { return x * v.x + y * v.y + z * v.z; }
    double lengthSquared() const { return x * x + y * y + z * z; }
    double length() const { return std::sqrt(lengthSquared()); }

    Vector3 normalized() const {
        double len = length();
        if (len < 1e-12) return *this;
        return *this / len;
    }
};

// Möller–Trumbore ray‑triangle intersection
// Returns true and sets `t` to the ray parameter if an intersection occurs.
bool mollerTrumbore(const Vector3 &origin, const Vector3 &direction,
                    const Vector3 &v0, const Vector3 &v1, const Vector3 &v2,
                    double &t) {
    const double EPSILON = 1e-8;
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 h = direction.cross(edge2);
    double a = edge1.dot(h);

    if (std::fabs(a) < EPSILON)
        return false; // parallel

    double f = 1.0 / a;
    Vector3 s = origin - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;

    Vector3 q = s.cross(edge1);
    double v = f * direction.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;

    t = f * edge2.dot(q);
    return (t > EPSILON);
}

// ----------------------------------------------------------------------
// Main
int main() {
    // Image dimensions (matching the original Rust code)
    const int width = 200;
    const int height = 200;

    // Camera parameters (same as in the original)
    Vector3 origin(0.0, -5.0, 0.0);

    // ---- Define a scene: list of triangles and an associated colour ----
    // Each triangle: three vertices
    // Each colour: 8‑bit RGB (we store as unsigned char)
    struct ColouredTriangle {
        Vector3 v0, v1, v2;
        uint8_t r, g, b;
    };

    // A few test triangles placed roughly in front of the camera
    // The camera looks upward from y = -5 to y = 0, with x,z in [-2,2].
    std::vector<ColouredTriangle> scene = {
        // Red triangle (visible)
        {Vector3(-1.0, 0.0, -1.0), Vector3(1.0, 0.0, -1.0), Vector3(0.0, 0.0, 1.0), 255, 0, 0},
        // Green triangle (partially behind the red one from this view)
        {Vector3(-0.5, -0.2, -0.5), Vector3(1.5, -0.2, -0.5), Vector3(0.5, -0.2, 1.5), 0, 255, 0},
        // Blue triangle further back (smaller visible area)
        {Vector3(-0.2, -0.5, 0.0), Vector3(1.2, -0.5, 0.0), Vector3(0.5, -0.5, 1.2), 0, 0, 255},
    };

    // ---- Ray trace ----
    std::vector<uint8_t> image(width * height * 3, 0); // RGB buffer, background black

    for (int y = 0; y < height; ++y) {   // y is the vertical index (Z in the original)
        for (int x = 0; x < width; ++x) { // x is the horizontal index (X in the original)
            // Compute the point on the image plane (at y = 0)
            double px = (x - 100.0) / 50.0;
            double pz = (y - 100.0) / 50.0;
            Vector3 pointOnPlane(px, 0.0, pz);
            Vector3 direction = pointOnPlane - origin; // not normalised, just as in the original

            double closestT = std::numeric_limits<double>::max();
            int hitIndex = -1;

            // Test every triangle
            for (size_t i = 0; i < scene.size(); ++i) {
                double t;
                if (mollerTrumbore(origin, direction,
                                   scene[i].v0, scene[i].v1, scene[i].v2,
                                   t)) {
                    if (t > 1e-6 && t < closestT) {
                        closestT = t;
                        hitIndex = static_cast<int>(i);
                    }
                }
            }

            // Colour the pixel
            size_t pixelIdx = (y * width + x) * 3;
            if (hitIndex >= 0) {
                const auto &col = scene[hitIndex];
                image[pixelIdx + 0] = col.r;
                image[pixelIdx + 1] = col.g;
                image[pixelIdx + 2] = col.b;
            }
            // else stays black (already zeroed)
        }
    }

    // ---- Write PPM (binary P6) ----
    std::ofstream out("output.ppm", std::ios::binary);
    if (!out) {
        std::cerr << "Failed to open output.ppm\n";
        return 1;
    }
    out << "P6\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<const char *>(image.data()), image.size());
    out.close();
    std::cout << "Image saved as output.ppm\n";

    return 0;
}
