#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <random>
#include <memory>

const double PI = 3.14159;

// 3D vector
struct Vector3 {
    double x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vector3 operator-(const Vector3 &v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vector3 operator*(const Vector3 &v) const { return {x * v.x, y * v.y, z * v.z}; }

    Vector3 cross(const Vector3 &v) const {
        return {y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x};
    }
    double dot(const Vector3 &v) const { return x * v.x + y * v.y + z * v.z; }
    double lengthSquared() const { return x*x + y*y + z*z; }
    double length() const { return std::sqrt(lengthSquared()); }
    Vector3 normalized() const {
        double len = length();
        return len < 1e-12 ? *this : *this / len;
    }
};

// ----------------------------------------------------------------------
struct Material {
    Vector3 albedo;
    Vector3 emission;
};

struct Triangle {
    Vector3 v0, v1, v2;
    int materialIdx;
};

// Moller–Trumbore
bool intersectTriangle(const Vector3 &origin, const Vector3 &dir,
                       const Vector3 &v0, const Vector3 &v1, const Vector3 &v2,
                       double &t) {
    const double EPS = 1e-8;
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 h = dir.cross(edge2);
    double a = edge1.dot(h);
    if (std::fabs(a) < EPS) return false;
    double f = 1.0 / a;
    Vector3 s = origin - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    Vector3 q = s.cross(edge1);
    double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;
    t = f * edge2.dot(q);
    return (t > EPS);
}

Vector3 triangleNormal(const Triangle &tri) {
    Vector3 e1 = tri.v1 - tri.v0;
    Vector3 e2 = tri.v2 - tri.v0;
    return e1.cross(e2).normalized();
}

// Uniform point on triangle, standard tecnique.
Vector3 randomPointOnTriangle(const Vector3 &v0, const Vector3 &v1, const Vector3 &v2,
                              double u, double v) {
    double su = std::sqrt(u);
    double a = 1.0 - su;
    double b = su * (1.0 - v);
    double c = su * v;
    return v0 * a + v1 * b + v2 * c;
}

// helper - add rectangle with normal normal right way round
void addRectangle(std::vector<Triangle> &triangles, int matIdx,
                  const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &d,
                  const Vector3 &desiredNormal) {

    Triangle t1{a, b, c, matIdx};
    Triangle t2{a, c, d, matIdx};

    // Flip t1 if needed
    Vector3 n1 = triangleNormal(t1);
    if (n1.dot(desiredNormal) < 0) {
        std::swap(t1.v1, t1.v2);
    }

    // Flip t2 if needed
    Vector3 n2 = triangleNormal(t2);
    if (n2.dot(desiredNormal) < 0) {
        std::swap(t2.v1, t2.v2);
    }

    triangles.push_back(t1);
    triangles.push_back(t2);
}

// helper to make cube
void addCube(std::vector<Triangle> &triangles, int matIdx,
             const Vector3 &center, double sx, double sy, double sz) {
    double x = center.x, y = center.y, z = center.z;
    double hx = sx*0.5, hy = sy*0.5, hz = sz*0.5;
    // Front face (z = +hz), normal +Z
    addRectangle(triangles, matIdx,
        {x-hx, y-hy, z+hz}, {x+hx, y-hy, z+hz}, {x+hx, y+hy, z+hz}, {x-hx, y+hy, z+hz},
        Vector3(0,0,1));
    // Back face (z = -hz), normal -Z
    addRectangle(triangles, matIdx,
        {x-hx, y-hy, z-hz}, {x+hx, y-hy, z-hz}, {x+hx, y+hy, z-hz}, {x-hx, y+hy, z-hz},
        Vector3(0,0,-1));
    // Left face (x = -hx), normal -X
    addRectangle(triangles, matIdx,
        {x-hx, y-hy, z-hz}, {x-hx, y+hy, z-hz}, {x-hx, y+hy, z+hz}, {x-hx, y-hy, z+hz},
        Vector3(-1,0,0));
    // Right face (x = +hx), normal +X
    addRectangle(triangles, matIdx,
        {x+hx, y-hy, z-hz}, {x+hx, y+hy, z-hz}, {x+hx, y+hy, z+hz}, {x+hx, y-hy, z+hz},
        Vector3(1,0,0));
    // Bottom face (y = -hy), normal -Y
    addRectangle(triangles, matIdx,
        {x-hx, y-hy, z-hz}, {x+hx, y-hy, z-hz}, {x+hx, y-hy, z+hz}, {x-hx, y-hy, z+hz},
        Vector3(0,-1,0));
    // Top face (y = +hy), normal +Y
    addRectangle(triangles, matIdx,
        {x-hx, y+hy, z-hz}, {x+hx, y+hy, z-hz}, {x+hx, y+hy, z+hz}, {x-hx, y+hy, z+hz},
        Vector3(0,1,0));
}

struct HitInfo {
    bool hit;
    double t;
    int triIdx;
    Vector3 point;
    Vector3 normal;
};

HitInfo intersectScene(const Vector3 &origin, const Vector3 &dir,
                       const std::vector<Triangle> &triangles) {
    HitInfo info;
    info.hit = false;
    info.t = std::numeric_limits<double>::max();
    for (size_t i = 0; i < triangles.size(); ++i) {
        double t;
        const Triangle &tri = triangles[i];
        if (intersectTriangle(origin, dir, tri.v0, tri.v1, tri.v2, t)) {
            if (t > 1e-6 && t < info.t) {
                info.hit = true;
                info.t = t;
                info.triIdx = static_cast<int>(i);
                info.point = origin + dir * t;
                info.normal = triangleNormal(tri);
            }
        }
    }
    return info;
}


bool occluded(const Vector3 &from, const Vector3 &to,
              const std::vector<Triangle> &triangles, int ignoreTriIdx) {
    Vector3 dir = to - from;
    double dist = dir.length();
    dir = dir / dist;
    double t;
    for (size_t i = 0; i < triangles.size(); ++i) {
        if (static_cast<int>(i) == ignoreTriIdx) continue;
        const Triangle &tri = triangles[i];
        if (intersectTriangle(from, dir, tri.v0, tri.v1, tri.v2, t)) {
            if (t > 1e-6 && t < dist - 1e-6) return true;
        }
    }
    return false;
}

// Perspective camera
struct Camera {
    Vector3 pos;
    Vector3 lookAt;
    Vector3 up;
    double fov; // degrees
    int width, height;

    Vector3 getRayDir(double u, double v) const {
        // u,v in [0,1] over image plane
        double aspect = (double)width / height;
        double theta = fov * PI / 180.0;
        double halfHeight = std::tan(theta * 0.5);
        double halfWidth = aspect * halfHeight;
        Vector3 w = (pos - lookAt).normalized();
        Vector3 uVec = up.cross(w).normalized();
        Vector3 vVec = w.cross(uVec);
        double x = (2.0 * u - 1.0) * halfWidth;
        double y = (1.0 - 2.0 * v) * halfHeight;
        return (uVec * x + vVec * y - w).normalized();
    }
};

// ----------------------------------------------------------------------
int main() {
    const int width = 400;
    const int height = 400;
    const int spp = 16;          // samples per pixel (primary rays)
    const int lightSamples = 4;  // samples per light per hit

    // scene made by ai for simplicity
    Camera cam;
    cam.pos = Vector3(0, 1.2, 3.5);
    cam.lookAt = Vector3(0, 0.8, 0);
    cam.up = Vector3(0, 1, 0);
    cam.fov = 60.0;
    cam.width = width;
    cam.height = height;

    // Materials
    std::vector<Material> materials;
    int matGray = materials.size(); materials.push_back({{0.7,0.7,0.7}, {0,0,0}});
    int matRed  = materials.size(); materials.push_back({{0.8,0.2,0.2}, {0,0,0}});
    int matGreen= materials.size(); materials.push_back({{0.2,0.8,0.2}, {0,0,0}});
    int matWhite= materials.size(); materials.push_back({{0.9,0.9,0.9}, {0,0,0}});
    int matLight= materials.size(); materials.push_back({{0,0,0}, {10.0,10.0,10.0}});

    std::vector<Triangle> triangles;

    // Room: floor Y=0, ceiling Y=2, left X=-2, right X=2, back Z=-2, open front (camera side)
    // Floor (normal +Y)
    addRectangle(triangles, matGray,
        {-2.0, 0.0, -2.0}, { 2.0, 0.0, -2.0}, { 2.0, 0.0,  2.0}, {-2.0, 0.0,  2.0},
        Vector3(0,1,0));
    // Ceiling (normal -Y)
    addRectangle(triangles, matGray,
        {-2.0, 2.0, -2.0}, { 2.0, 2.0, -2.0}, { 2.0, 2.0,  2.0}, {-2.0, 2.0,  2.0},
        Vector3(0,-1,0));
    // Back wall (Z = -2), normal +Z
    addRectangle(triangles, matGray,
        {-2.0, 0.0, -2.0}, { 2.0, 0.0, -2.0}, { 2.0, 2.0, -2.0}, {-2.0, 2.0, -2.0},
        Vector3(0,0,1));
    // Left wall (X = -2), normal +X
    addRectangle(triangles, matRed,
        {-2.0, 0.0, -2.0}, {-2.0, 2.0, -2.0}, {-2.0, 2.0,  2.0}, {-2.0, 0.0,  2.0},
        Vector3(1,0,0));
    // Right wall (X = 2), normal -X
    addRectangle(triangles, matGreen,
        { 2.0, 0.0, -2.0}, { 2.0, 2.0, -2.0}, { 2.0, 2.0,  2.0}, { 2.0, 0.0,  2.0},
        Vector3(-1,0,0));

    // Area light (emissive) on ceiling, slightly inset
    addRectangle(triangles, matLight,
        {-1.5, 1.98, -1.5}, { 1.5, 1.98, -1.5}, { 1.5, 1.98,  1.5}, {-1.5, 1.98,  1.5},
        Vector3(0,-1,0));

    // White diffuse cube in the centre
    addCube(triangles, matWhite, {0.0, 0.5, 0.0}, 1.0, 1.0, 1.0);

    // Pre‑compute light triangles
    struct LightTri {
        int triIdx;
        Vector3 emission;
        double area;
        Vector3 normal;
    };
    std::vector<LightTri> lights;
    for (size_t i = 0; i < triangles.size(); ++i) {
        const Triangle &tri = triangles[i];
        const Material &mat = materials[tri.materialIdx];
        if (mat.emission.x > 0 || mat.emission.y > 0 || mat.emission.z > 0) {
            double area = 0.5 * ((tri.v1 - tri.v0).cross(tri.v2 - tri.v0)).length();
            Vector3 normal = triangleNormal(tri);
            lights.push_back({(int)i, mat.emission, area, normal});
        }
    }

    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    std::vector<uint8_t> image(width * height * 3, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vector3 pixelColor(0,0,0);
            for (int s = 0; s < spp; ++s) {
                double u = x / width;
                double v = y / height;
                Vector3 dir = cam.getRayDir(u, v);
                HitInfo hit = intersectScene(cam.pos, dir, triangles);
                if (!hit.hit) continue;

                const Material &hitMat = materials[triangles[hit.triIdx].materialIdx];
                // Directly emit if hitting light - ONLY VALID IF PRIMARY RAY - ONLY EMMISSIVE OR NOT (NOT BOTH)
                if (hitMat.emission.x > 0 || hitMat.emission.y > 0 || hitMat.emission.z > 0) {
                    pixelColor = pixelColor + hitMat.emission;
                    continue;
                }

                // NEE: sample all lights
                Vector3 direct(0,0,0);
                for (const LightTri &light : lights) {
                    for (int ls = 0; ls < lightSamples; ++ls) {
                        double u1 = dist(rng), u2 = dist(rng);
                        Vector3 lightPoint = randomPointOnTriangle(
                            triangles[light.triIdx].v0,
                            triangles[light.triIdx].v1,
                            triangles[light.triIdx].v2, u1, u2);


                        Vector3 toLight = lightPoint - hit.point;
                        double dist2 = toLight.lengthSquared();
                        double distL = std::sqrt(dist2);
                        Vector3 lightDir = toLight / distL;

                        double cosHit = std::max(0.0, hit.normal.dot(lightDir));
                        if (cosHit == 0.0) continue;

                        Vector3 toHitDir = lightDir*-1;
                        double cosLight = std::max(0.0, light.normal.dot(toHitDir));
                        if (cosLight == 0.0) continue;

                        Vector3 shadowOrigin = hit.point + hit.normal * 1e-4;
                        if (occluded(shadowOrigin, lightPoint, triangles, light.triIdx))
                            continue;

                        double G = cosHit * cosLight / dist2;
                        Vector3 brdf = hitMat.albedo / PI;
                        Vector3 contrib = brdf * light.emission * (G * light.area);
                        direct = direct + contrib;
                    }
                }
                direct = direct / lightSamples;
                pixelColor = pixelColor + direct;
            }
            pixelColor = pixelColor / spp;

            int idx = (y * width + x) * 3;
            image[idx+0] = (uint8_t)(std::min(1.0, pixelColor.x) * 255);
            image[idx+1] = (uint8_t)(std::min(1.0, pixelColor.y) * 255);
            image[idx+2] = (uint8_t)(std::min(1.0, pixelColor.z) * 255);
        }
    }

    std::ofstream out("direct_lighting_fixed.ppm", std::ios::binary);
    out << "P6\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.data()), image.size());
    out.close();
    std::cout << "Saved direct_lighting_fixed.ppm\n";
    return 0;
}
