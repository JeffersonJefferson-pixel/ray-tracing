#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable {
    public:
        sphere(const point3& center, double radius, shared_ptr<material> mat) 
            : center1(center), radius(fmax(0, radius)), mat(mat), is_moving(false) {}

        sphere(const point3& center1, const point3& center2, double radius, shared_ptr<material> mat)
            : center1(center1), radius(std::fmax(0, radius)), mat(mat), is_moving(true) {
                center_vec = center2 - center1;
            }

    bool hit(const ray&r, interval ray_t, hit_record& rec) const override {
        // get center of sphere based on ray's time
        point3 center = is_moving ? sphere_center(r.time()) : center1;
        vec3 oc = center - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius*radius;

        auto discriminant = h*h - a*c;
        if (discriminant < 0)
            return false;
        
        auto sqrtd = sqrt(discriminant);

        // find the nearest root that lies in the acceptable range.
        auto root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

    private:
        point3 center1;
        double radius;
        shared_ptr<material> mat;
        bool is_moving;
        vec3 center_vec;

        point3 sphere_center(double time) const {
            return center1 + time*center_vec;
        }
};

#endif