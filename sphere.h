#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable {
    public:
        // stationary sphere
        sphere(const point3& static_center, double radius, shared_ptr<material> mat) 
            : center(static_center, vec3(0, 0, 0)), radius(fmax(0, radius)), mat(mat) {
                // construct bounding box.
                auto rvec = vec3(radius, radius, radius);
                bbox = aabb(static_center - rvec, static_center + rvec);
            }
        // moving sphere.
        sphere(const point3& center1, const point3& center2, double radius, shared_ptr<material> mat)
            : center(center1, center2 - center1), radius(std::fmax(0, radius)), mat(mat) {
                // bounding box at t=0 and t=1.
                // bounding box around the two boxes.
                auto rvec = vec3(radius, radius, radius);
                aabb box1(center.at(0) - rvec, center.at(0) + rvec);
                aabb box2(center.at(1) - rvec, center.at(1) + rvec);
                bbox = aabb(box1, box2);
            }

        bool hit(const ray&r, interval ray_t, hit_record& rec) const override {
            // get center of sphere based on ray's time
            point3 current_center = center.at(r.time());
            vec3 oc = current_center - r.origin();
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
            vec3 outward_normal = (rec.p - current_center) / radius;
            rec.set_face_normal(r, outward_normal);
            get_sphere_uv(outward_normal, rec.u, rec.v);
            rec.mat = mat;

            return true;
        }

        aabb bounding_box() const override {
            return bbox;
        }

    private:
        ray center;
        double radius;
        shared_ptr<material> mat;
        aabb bbox;

        /** 
         * Takes points on the unit sphere centered at the origin, 
         * and computes the texture coordinate u and v.
         */
        static void get_sphere_uv(const point3& p, double& u, double &v) {
            auto theta = std::acos(-p.y());
            auto phi = std::atan2(-p.z(), p.x()) + pi;

            u = phi / (2*pi);
            v = theta = theta / pi;
        }
};

#endif