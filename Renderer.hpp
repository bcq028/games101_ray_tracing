#pragma once
#include "Scene.hpp"
#include <optional>
#include <vector>
#include <iostream>
#include <fstream>

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
      Vector3f lightAmt = 0, specularColor = 0;
      Vector3f shadowPointOrig = (dotProduct(dir, N) < 0)
                                     ? hitPoint + N * scene.epsilon
                                     : hitPoint - N * scene.epsilon;
      for (auto &light : scene.get_lights()) {
        Vector3f lightDir = light->position - hitPoint;
        float lightDistance2 = dotProduct(lightDir, lightDir);
        lightDir = normalize(lightDir);
        float LdotN = std::max(0.f, dotProduct(lightDir, N));
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
    std::vector<std::vector<Vector3f>> frame_buffer(scene.height,std::vector<Vector3f>(scene.width));
    Vector3f eye_pos(0);
    float focal_length=1.0;
    float viewport_height=std::tan(deg2rad(scene.fov * 0.5f))*focal_length;
    float viewport_width=viewport_height*scene.width / (float)scene.height;
    for(int i=0;i<frame_buffer.size();++i){
      for(int j=0;j<frame_buffer[0].size();++j){
            float ndc_x = (j + 0.5f) /scene.width *2 - 1.0f;
            float ndc_y = (i + 0.5f) /scene.height*2 - 1.0f;
            float y = ndc_y * viewport_height;
            float x = ndc_x * viewport_width;
            Vector3f dir = normalize(Vector3f(x, y, -1)); 
            frame_buffer[i][j] = 255*castRay(eye_pos, dir, scene, 0);
      }
    }    
    const std::string path="binary.ppm";
    write_ppm_header(path, frame_buffer.size(), frame_buffer[0].size());
    write_ppm_data(path, frame_buffer);
  }

private:
  void write_ppm_header(std::string path, int width,int height){
    std::ofstream file;

    file.open(path,std::ios::out);
    if(!file.is_open()){
      throw "write header error:file cannot open";
    }
    file<<"P6\n";
    file<<height<<" "<<width<<'\n';
    file<<255<<'\n';
  }
  void write_ppm_data(std::string path, const std::vector<std::vector<Vector3f>>& frame_buffer){
    std::ofstream file;
    file.open(path,std::ios::app);
    if(!file.is_open()){
      throw "write header error:file cannot open";
    }
    for(int i=frame_buffer.size()-1;i>=0;--i){
      for(int j=0;j<frame_buffer[0].size();++j){
         file<<(char)frame_buffer[i][j].x <<(char)frame_buffer[i][j].y<<(char)frame_buffer[i][j].z;
      }
    }
  }
};
