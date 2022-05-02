//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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

    Vector3f L_dir = {0,0,0};                       //direct illumination
    Vector3f L_indir = {0,0,0};                     //indirect illumination

    Intersection inter = this->intersect(ray);      //this ray intersect with object/light/null
    if(inter.happened == false)                     //if intersect with null
        return {0,0,0};
    if(inter.m->hasEmission() == true)              //if intersect with light
        return inter.m->getEmission();

    Intersection pos_light;          
    float pdf_light = 0.0f;
    sampleLight(pos_light, pdf_light);                                              //return light's pos and pdf
    Vector3f inter_to_light_dir = pos_light.coords - inter.coords;                  //shoot a ray from p to x
    Ray inter_to_light_ray(inter.coords, inter_to_light_dir.normalized());   
    Intersection inter_to_light_block = this->intersect(inter_to_light_ray);        //what our shoot's ray intersect with

    if(std::fabs((inter_to_light_block.coords - pos_light.coords).norm()) < 0.0001)            //if it is not blocked calculate L_dir
    {
        L_dir = pos_light.emit * inter.m->eval(ray.direction, inter_to_light_dir.normalized(), inter.normal)
                * dotProduct(inter_to_light_dir.normalized(), inter.normal) 
                * dotProduct(-inter_to_light_dir.normalized(), inter_to_light_block.normal)
                / (std::pow(inter_to_light_dir.norm(), 2)) / pdf_light;
    }

    if(get_random_float() > RussianRoulette)        //RussianRoulette
        return  L_dir;
    
    Vector3f sample_dir = inter.m->sample(ray.direction, inter.normal).normalized();
    Ray sample_to_obj(inter.coords, sample_dir);
    Intersection hit_obj = this->intersect(sample_to_obj);

    if(hit_obj.happened == true && hit_obj.m->hasEmission() == false)
    {
        L_indir = castRay(sample_to_obj, depth++) * inter.m->eval(ray.direction, sample_dir, inter.normal) * dotProduct(sample_dir, inter.normal)
                / inter.m->pdf(ray.direction, sample_dir, inter.normal) / RussianRoulette;
    }

    return L_dir + L_indir; 

}