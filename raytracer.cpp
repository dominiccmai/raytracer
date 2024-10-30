#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <chrono>
#include <memory>
#include <random>
#include <algorithm>

#define M_PI   3.14159265358979323846264338327950288
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator*(double d) const { return Vec3(x * d, y * d, z * d); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; } 
    Vec3 normalize() const {
        double mg = sqrt(x*x + y*y + z*z);
        return Vec3(x/mg, y/mg, z/mg);
    }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
};

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {}
};

class Texture {
    int width, height, channels;
    std::unique_ptr<unsigned char[]> data;

public:
    Texture(const char* filename) {
        data.reset(stbi_load(filename, &width, &height, &channels, 0));
        if (!data) {
            std::cerr << "ERROR: Could not load texture file " << filename << std::endl;
        }
    }

    Vec3 getColor(double u, double v) const {
        if (!data) return Vec3(1, 0, 1); // Return magenta if texture is not loaded

        int x = static_cast<int>(u * width);
        int y = static_cast<int>(v * height);
        x = std::clamp(x, 0, width - 1);
        y = std::clamp(y, 0, height - 1);

        int index = (y * width + x) * channels;
        return Vec3(
            data[index] / 255.0,
            data[index + 1] / 255.0,
            data[index + 2] / 255.0
        );
    }

    Vec3 getNormal(double u, double v) const {
        Vec3 color = getColor(u, v);
        return Vec3(color.x * 2 - 1, color.y * 2 - 1, color.z * 2 - 1).normalize();
    }
};

struct Material {
    Vec3 color;
    double ambient, diffuse, specular, shininess;
    double reflectivity;
    double transparency, refractive_index;
    std::shared_ptr<Texture> texture;
    std::shared_ptr<Texture> normal_map;
    Material(const Vec3& c, double a, double d, double s, double sh, double r = 0, double t = 0, double ri = 1, 
             std::shared_ptr<Texture> tex = nullptr, std::shared_ptr<Texture> norm = nullptr)
        : color(c), ambient(a), diffuse(d), specular(s), shininess(sh), reflectivity(r), 
          transparency(t), refractive_index(ri), texture(tex), normal_map(norm) {}
};

class Object {
public:
    Material material;
    Object(const Material& m) : material(m) {}
    virtual bool intersect(const Ray& ray, double& t) const = 0;
    virtual Vec3 getNormal(const Vec3& point) const = 0;
    virtual void getUV(const Vec3& point, double& u, double& v) const = 0;
    virtual void getTangentSpace(const Vec3& point, Vec3& tangent, Vec3& bitangent) const = 0;
};

class Sphere : public Object {
    Vec3 center;
    double radius;
public:
    Sphere(const Vec3& c, double r, const Material& m) : Object(m), center(c), radius(r) {}
    bool intersect(const Ray& ray, double& t) const override {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;
        double sqrtd = sqrt(discriminant);
        double t0 = (-b - sqrtd) / (2.0 * a);
        double t1 = (-b + sqrtd) / (2.0 * a);
        t = (t0 < t1) ? t0 : t1;
        return t > 0;
    }
    Vec3 getNormal(const Vec3& point) const override {
        return (point - center).normalize();
    }
    void getUV(const Vec3& point, double& u, double& v) const override {
        Vec3 normal = getNormal(point);
        u = 0.5 + atan2(normal.z, normal.x) / (2 * M_PI);
        v = 0.5 - asin(normal.y) / M_PI;
    }
    void getTangentSpace(const Vec3& point, Vec3& tangent, Vec3& bitangent) const override {
        Vec3 normal = getNormal(point);
        tangent = Vec3(normal.y, normal.z, -normal.x).normalize();
        bitangent = normal.cross(tangent);
    }
};

class Plane : public Object {
    Vec3 point;
    Vec3 normal;
public:
    Plane(const Vec3& p, const Vec3& n, const Material& m) : Object(m), point(p), normal(n.normalize()) {}
    bool intersect(const Ray& ray, double& t) const override {
        double denom = normal.dot(ray.direction);
        if (std::abs(denom) > 1e-6) {
            Vec3 p0l0 = point - ray.origin;
            t = p0l0.dot(normal) / denom;
            return (t >= 0);
        }
        return false;
    }
    Vec3 getNormal(const Vec3& point) const override {
        return normal;
    }
    void getUV(const Vec3& point, double& u, double& v) const override {
        Vec3 d = point - this->point;
        u = d.x - floor(d.x);
        v = d.z - floor(d.z);
    }
    void getTangentSpace(const Vec3& point, Vec3& tangent, Vec3& bitangent) const override {
        Vec3 v1 = Vec3(normal.y, normal.z, -normal.x);
        tangent = v1.cross(normal).normalize();
        bitangent = normal.cross(tangent);
    }
};


class Triangle : public Object {
    Vec3 v0, v1, v2;
    Vec3 normal;
    Vec3 edge1, edge2;
public:
    Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Material& m)
        : Object(m), v0(v0), v1(v1), v2(v2) {
        edge1 = v1 - v0;
        edge2 = v2 - v0;
        normal = edge1.cross(edge2).normalize();
    }
    bool intersect(const Ray& ray, double& t) const override {
        const double EPSILON = 0.0000001;
        Vec3 edge1, edge2, h, s, q;
        double a, f, u, v;
        edge1 = v1 - v0;
        edge2 = v2 - v0;
        h = ray.direction.cross(edge2);
        a = edge1.dot(h);
        if (a > -EPSILON && a < EPSILON)
            return false;    // This ray is parallel to this triangle.
        f = 1.0 / a;
        s = ray.origin - v0;
        u = f * s.dot(h);
        if (u < 0.0 || u > 1.0)
            return false;
        q = s.cross(edge1);
        v = f * ray.direction.dot(q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q);
        if (t > EPSILON) // ray intersection
            return true;
        else // This means that there is a line intersection but not a ray intersection.
            return false;
    }
    Vec3 getNormal(const Vec3& point) const override {
        return normal;
    }
    void getUV(const Vec3& point, double& u, double& v) const override {
        // Simple barycentric coordinates for UV mapping
        Vec3 e1 = v1 - v0;
        Vec3 e2 = v2 - v0;
        Vec3 p = point - v0;
        double d11 = e1.dot(e1);
        double d12 = e1.dot(e2);
        double d22 = e2.dot(e2);
        double dp1 = p.dot(e1);
        double dp2 = p.dot(e2);
        double denom = d11 * d22 - d12 * d12;
        u = (d22 * dp1 - d12 * dp2) / denom;
        v = (d11 * dp2 - d12 * dp1) / denom;
    }
    void getTangentSpace(const Vec3& point, Vec3& tangent, Vec3& bitangent) const override {
        tangent = edge1.normalize();
        bitangent = normal.cross(tangent);
    }

};

struct PointLight {
    Vec3 position;
    Vec3 color;
    PointLight(const Vec3& p, const Vec3& c) : position(p), color(c) {}
};

class Scene {
    std::vector<std::unique_ptr<Object>> objects;
    std::vector<PointLight> lights;
public:
    void addObject(std::unique_ptr<Object> obj) { objects.push_back(std::move(obj)); }
    void addLight(const PointLight& light) { lights.push_back(light); }
    bool intersect(const Ray& ray, double& t, const Object*& hitObject) const {
        t = std::numeric_limits<double>::infinity();
        hitObject = nullptr;
        for (const auto& obj : objects) {
            double tHit;
            if (obj->intersect(ray, tHit) && tHit < t) {
                t = tHit;
                hitObject = obj.get();
            }
        }
        return hitObject != nullptr;
    }
    const std::vector<PointLight>& getLights() const { return lights; }
};

class Camera {
    Vec3 position;
    Vec3 direction;
    Vec3 up;
    double fov;
    double aspect;
public:
    Camera(const Vec3& pos, const Vec3& dir, const Vec3& up, double fov, double aspect)
        : position(pos), direction(dir.normalize()), up(up.normalize()), fov(fov), aspect(aspect) {}
    
    Ray generateRay(double x, double y) const {
        Vec3 right = direction.cross(up).normalize();
        Vec3 newUp = right.cross(direction);
        
        double tanFov = tan(fov * 0.5 * M_PI / 180.0);
        double screenHeight = 2 * tanFov;
        double screenWidth = screenHeight * aspect;

        Vec3 screenCenter = position + direction;
        Vec3 screenPoint = screenCenter + right * ((x - 0.5) * screenWidth) + newUp * ((0.5 - y) * screenHeight);
        
        return Ray(position, (screenPoint - position).normalize());
    }
};


class Renderer {
private:
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;

public:
    Renderer() : gen(std::random_device()()), dis(0.0, 1.0) {}

    void render(const Scene& scene, const Camera& camera, int width, int height, const std::string& filename, int samples_per_pixel) {
        std::vector<unsigned char> image(width * height * 3);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Vec3 color(0, 0, 0);
                for (int s = 0; s < samples_per_pixel; ++s) {
                    double u = (x + dis(gen)) / width;
                    double v = (y + dis(gen)) / height;
                    Ray ray = camera.generateRay(u, v);
                    color += traceRay(scene, ray, 0);
                }
                color = color * (1.0 / samples_per_pixel);

                // Apply simple tone mapping (gamma correction)
                color = Vec3(std::sqrt(color.x), std::sqrt(color.y), std::sqrt(color.z));
                
                int index = 3 * (y * width + x);
                image[index] = std::min(255, std::max(0, int(color.x * 255)));
                image[index + 1] = std::min(255, std::max(0, int(color.y * 255)));
                image[index + 2] = std::min(255, std::max(0, int(color.z * 255)));
            }
            if (y % 10 == 0) {
                std::cout << "\rRendering: " << (y * 100 / height) << "% complete" << std::flush;
            }
        }
        std::cout << "\rRendering: 100% complete\n" << std::endl;

        stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    }

private:
    Vec3 traceRay(const Scene& scene, const Ray& ray, int depth) {
        if (depth > 5) return Vec3();

        double t;
        const Object* hitObject;
        if (scene.intersect(ray, t, hitObject)) {
            Vec3 hitPoint = ray.origin + ray.direction * t;
            Vec3 normal = hitObject->getNormal(hitPoint);
            double u, v;
            hitObject->getUV(hitPoint, u, v);
            Vec3 color = shade(scene, hitObject, hitPoint, normal, -ray.direction, u, v);

            // Reflection
            if (hitObject->material.reflectivity > 0) {
                Vec3 reflectDir = reflect(ray.direction, normal);
                Ray reflectRay(hitPoint + normal * 0.001, reflectDir);
                Vec3 reflectColor = traceRay(scene, reflectRay, depth + 1);
                color += reflectColor * hitObject->material.reflectivity;
            }

            // Refraction
            if (hitObject->material.transparency > 0) {
                double n1 = 1.0; // Air refractive index
                double n2 = hitObject->material.refractive_index;
                double n = n1 / n2;
                double cosI = -normal.dot(ray.direction);
                double sinT2 = n * n * (1.0 - cosI * cosI);
                
                if (sinT2 <= 1.0) {
                    Vec3 refractDir = ray.direction * n + normal * (n * cosI - sqrt(1.0 - sinT2));
                    Ray refractRay(hitPoint - normal * 0.001, refractDir);
                    Vec3 refractColor = traceRay(scene, refractRay, depth + 1);
                    color = color * (1 - hitObject->material.transparency) + refractColor * hitObject->material.transparency;
                }
            }

            return color;
        }
        
        // Background color (sky)
        t = 0.5 * (ray.direction.y + 1.0);
        return Vec3(1.0, 1.0, 1.0) * (1.0 - t) + Vec3(0.5, 0.7, 1.0) * t;
    }

    Vec3 shade(const Scene& scene, const Object* obj, const Vec3& point, const Vec3& normal, const Vec3& view, double u, double v) {
        Vec3 color(0, 0, 0);
        const Material& mat = obj->material;

        // Get texture color if available
        Vec3 texColor = mat.texture ? mat.texture->getColor(u, v) : mat.color;

        // Get normal from normal map if available
        Vec3 N = normal;
        if (mat.normal_map) {
            Vec3 tangent, bitangent;
            obj->getTangentSpace(point, tangent, bitangent);
            Vec3 mappedNormal = mat.normal_map->getNormal(u, v);
            N = (tangent * mappedNormal.x + bitangent * mappedNormal.y + normal * mappedNormal.z).normalize();
        }

        for (const auto& light : scene.getLights()) {
            Vec3 lightDir = (light.position - point).normalize();
            double lightDistance = (light.position - point).dot(light.position - point);

            // Check for shadows
            Ray shadowRay(point + N * 0.001, lightDir);
            double t;
            const Object* shadowObj;
            bool inShadow = scene.intersect(shadowRay, t, shadowObj) && t * t < lightDistance;

            if (!inShadow) {
                // Diffuse
                double diff = std::max(N.dot(lightDir), 0.0);
                Vec3 diffuse = texColor * light.color * diff * mat.diffuse;

                // Specular
                Vec3 reflectDir = reflect(-lightDir, N);
                double spec = std::pow(std::max(view.dot(reflectDir), 0.0), mat.shininess);
                Vec3 specular = light.color * spec * mat.specular;

                color += diffuse + specular;
            }
        }

        // Ambient
        color += texColor * mat.ambient;

        return color;
    }

    Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - n * 2 * v.dot(n);
    }
};

int main() {
    Scene scene;
    Camera camera(Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0), 90, 16.0/9.0);
    Renderer renderer;

    // Load textures and normal maps
    auto earthTexture = std::make_shared<Texture>("earth_texture.jpg");
    auto earthNormalMap = std::make_shared<Texture>("earth_normal_map.jpg");
    auto checkerTexture = std::make_shared<Texture>("checker_texture.png");

    // Add objects
    scene.addObject(std::make_unique<Sphere>(Vec3(0, 0, -5), 1, 
        Material(Vec3(1, 1, 1), 0.1, 0.9, 0.8, 32, 0.5, 0, 1, earthTexture, earthNormalMap))); // Earth sphere with normal map
    scene.addObject(std::make_unique<Sphere>(Vec3(-2, 0, -6), 1, 
        Material(Vec3(0.3, 0.3, 0.7), 0.1, 0.9, 0.8, 32, 0, 0.8, 1.5))); // Blue transparent sphere
    scene.addObject(std::make_unique<Sphere>(Vec3(2, 0, -4), 1, 
        Material(Vec3(0.3, 0.7, 0.3), 0.1, 0.9, 0.8, 32))); // Green diffuse sphere

    // Add a plane (floor) with checker texture
    scene.addObject(std::make_unique<Plane>(Vec3(0, -2, 0), Vec3(0, 1, 0), 
        Material(Vec3(1, 1, 1), 0.1, 0.9, 0.0, 1, 0, 0, 1, checkerTexture)));
    // Add a triangle
    scene.addObject(std::make_unique<Triangle>(
        Vec3(-2, 2, -6),
        Vec3(2, 2, -6),
        Vec3(0, 4, -6),
        Material(Vec3(1.0, 1.0, 0.0), 0.1, 0.9, 0.8, 32) // Yellow triangle
    ));

    // Add lights
    scene.addLight(PointLight(Vec3(5, 5, -5), Vec3(1, 1, 1)));
    scene.addLight(PointLight(Vec3(-5, 5, -5), Vec3(0.5, 0.5, 0.5)));

    std::cout << "Starting ray tracing..." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    int samples_per_pixel = 4;
    renderer.render(scene, camera, 800, 450, "output.png", samples_per_pixel);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    std::cout << "Ray tracing completed in " << diff.count() << " seconds." << std::endl;
    std::cout << "Output image saved as 'output.png'" << std::endl;

    return 0;
}