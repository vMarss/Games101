//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // 深度大于最大深度
    if (depth > maxDepth)
    {
        return Vector3f(0.f, 0.f, 0.f);
    }

    Intersection intersection = intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = backgroundColor;
    Vector2f uv;
    uint32_t index = 0;

    // 没有命中场景中的物体，返回场景底色
    if (intersection.happened)
    {   
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates

        switch (m->getType())
        {
        case DIFFUSE:
            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (uint32_t i = 0; i < get_lights().size(); ++i)
            {
                auto area_ptr = dynamic_cast<AreaLight *>(this->get_lights()[i].get());
                if (area_ptr)
                {
                    // Do nothing for this assignment
                    // 不处理局部光照
                }
                else
                {
                    Vector3f lightDir = get_lights()[i]->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    Object *shadowHitObject = nullptr;
                    float tNearShadow = kInfinity;
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                    lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                          m->specularExponent) *
                                     get_lights()[i]->intensity;
                }
            }
            hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
            break;

        default:
            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (uint32_t i = 0; i < get_lights().size(); ++i)
            {
                auto area_ptr = dynamic_cast<AreaLight *>(this->get_lights()[i].get());
                if (area_ptr)
                {
                    // Do nothing for this assignment
                    // 不处理局部光照
                }
                else
                {
                    Vector3f lightDir = get_lights()[i]->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    Object *shadowHitObject = nullptr;
                    float tNearShadow = kInfinity;
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                    lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                          m->specularExponent) *
                                     get_lights()[i]->intensity;
                }
            }
            hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
            break;
        }
    }

    return hitColor;
}