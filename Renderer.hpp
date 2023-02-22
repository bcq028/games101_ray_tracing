#pragma once
#include "Scene.hpp"
#include <optional>

constexpr float kInfinity = std::numeric_limits<float>::max();

struct hit_payload {
  float tNear;
  uint32_t index;
  Vector2f uv;
  Object *hit_obj;
};

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

inline Vector3f reflect(const Vector3f &I, const Vector3f &N) {
  return I - 2 * dotProduct(I, N) * N;
}

inline Vector3f refract(const Vector3f &I, const Vector3f &N,
                        const float &ior) {
  float cosi = fabs(clamp(-1, 1, dotProduct(I, N)));
  auto t=1 - 1/ior * 1/ior * (1 - cosi * cosi);
  if(t<0) return 0;
  float coso =sqrtf(t);
  return 1/ior * I + (1/ior * cosi - coso) * N;
}

inline float fresnel(const Vector3f &I, const Vector3f &N, const float &ior) {
  auto cosi = fabs(clamp(-1, 1, dotProduct(I, N)));
  auto R0 = pow((1 - ior) / (1 + ior), 2);
  return R0 + (1 - R0) * pow((1 - cosi), 5);
}

inline std::optional<hit_payload>
trace(const Vector3f &orig, const Vector3f &dir,
      const std::vector<std::unique_ptr<Object>> &objects) {
  float tNear = kInfinity;
  std::optional<hit_payload> payload;
  for (const auto &object : objects) {
    float tNearK = kInfinity;
    uint32_t indexK;
    Vector2f uvK;
    if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear) {
      payload.emplace();
      payload->hit_obj = object.get();
      payload->tNear = tNearK;
      payload->index = indexK;
      payload->uv = uvK;
      tNear = tNearK;
    }
  }
  return payload;
}

inline Vector3f castRay(const Vector3f &orig, const Vector3f &dir,
                        const Scene &scene, int depth) {
  if (depth > scene.maxDepth) {
    return Vector3f(0.0, 0.0, 0.0);
  }

  Vector3f hitColor = scene.backgroundColor;
  if (auto payload = trace(orig, dir, scene.get_objects()); payload) {
    Vector3f hitPoint = orig + dir * payload->tNear;
    Vector3f N;  // normal
    Vector2f st; // st coordinates
    payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index,
                                           payload->uv, N, st);
    switch (payload->hit_obj->materialType) {
    case REFLECTION_AND_REFRACTION: {
      Vector3f reflectionDirection = normalize(reflect(dir, N));
      Vector3f refractionDirection =
          normalize(refract(dir, N, payload->hit_obj->ior));
      Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0)
                                       ? hitPoint - N * scene.epsilon
                                       : hitPoint + N * scene.epsilon;
      Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0)
                                       ? hitPoint - N * scene.epsilon
                                       : hitPoint + N * scene.epsilon;
      Vector3f reflectionColor =
          castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
      Vector3f refractionColor =
          castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
      float kr = fresnel(dir, N, payload->hit_obj->ior);
      hitColor = reflectionColor * kr + refractionColor * (1 - kr);
      break;
    }
    case REFLECTION: {
      float kr = fresnel(dir, N, payload->hit_obj->ior);
      Vector3f reflectionDirection = reflect(dir, N);
      Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0)
                                       ? hitPoint + N * scene.epsilon
                                       : hitPoint - N * scene.epsilon;
      hitColor =
          castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) *
          kr;
      break;
    }
    default: {
      // [comment]
      // We use the Phong illumation model in the default case. The phong model
      // is composed of a diffuse and a specular reflection component.
      // [/comment]
      Vector3f lightAmt = 0, specularColor = 0;
      Vector3f shadowPointOrig = (dotProduct(dir, N) < 0)
                                     ? hitPoint + N * scene.epsilon
                                     : hitPoint - N * scene.epsilon;
      // [comment]
      // Loop over all lights in the scene and sum their contribution up
      // We also apply the lambert cosine law
      // [/comment]
      for (auto &light : scene.get_lights()) {
        Vector3f lightDir = light->position - hitPoint;
        // square of the distance between hitPoint and the light
        float lightDistance2 = dotProduct(lightDir, lightDir);
        lightDir = normalize(lightDir);
        float LdotN = std::max(0.f, dotProduct(lightDir, N));
        // is the point in shadow, and is the nearest occluding object closer to
        // the object than the light itself?
        auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
        bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear <
                                       lightDistance2);

        lightAmt += inShadow ? 0 : light->intensity * LdotN;
        Vector3f reflectionDirection = reflect(-lightDir, N);

        specularColor +=
            powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                 payload->hit_obj->specularExponent) *
            light->intensity;
      }

      hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) *
                     payload->hit_obj->Kd +
                 specularColor * payload->hit_obj->Ks;
      break;
    }
    }
  }

  return hitColor;
}

class Renderer {
public:
  void Render(const Scene &scene) {
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j) {
      for (int i = 0; i < scene.width; ++i) {
        // generate primary ray direction
        // Screen space to NDC space
        float nx = (i + 0.5f) * 2 / scene.width - 1.0f;
        float ny = (j + 0.5f) * 2 / scene.height - 1.0f;

        // NDC space to world space

        // Project matrix
        /*
         *   [ n/r ,0   ,0       ,0      ]
         *   [ 0   ,n/t ,0       ,0      ]
         *   [ 0   ,0   ,n+f/n-f ,2nf/f-n]
         *   [ 0   ,0   ,1       ,0      ]
         */

        //
        // 在投影矩阵中 x 的系数为 n/r
        // 在投影矩阵中 y 的系数为 n/t
        // 现在要做一个逆操作，所以我们用 NDC空间的坐标分别除以投影矩阵中的系数
        // x = nx / n / r
        // y = ny / n / t
        // 其中 n(相机到近投影面距离为 默认情况下为1）
        // =>
        // x = nx * r
        // y = ny * t
        // 其中 r = tan(fov/2)*aspect * |n|， t=tan(fov/2) * |n| , |n| = 1
        // 所以可得,世界空间中坐标为
        // x = nx * tan(fov/2)*aspect
        // y = ny * tan(fov/2)*aspect

        float x = nx * scale * imageAspectRatio;
        float y = -ny * scale;

        Vector3f dir =
            Vector3f(x, y, -1); // Don't forget to normalize this direction!
        dir = normalize(dir);
        framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
      }
      UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
      static unsigned char color[3];
      color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
      color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
      color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
      fwrite(color, 1, 3, fp);
    }
    fclose(fp);
  }

private:
};
