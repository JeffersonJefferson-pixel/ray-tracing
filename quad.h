#ifndef QUAD_H
#define QUAD_H

#include "hittable.h"

class quad : public hittable {
  public: 
    quad(const point3& Q, const vec3& u, const vec3& v, shared_ptr<material> mat) 
        : Q(Q), u(u), v(v), mat(mat) 
      {
        // find normal vector of quad.
        auto n = cross(u, v);
        normal = unit_vector(n);
        D = dot(normal, Q);
        w = n / dot(n, n);

        set_bounding_box();
      }
    
    virtual void set_bounding_box() {
      // compute bounding box of all four vertices.
      auto bbox_diagonal1 = aabb(Q, Q + u + v);
      auto bbox_diagonal2 = aabb(Q + u, Q + v);
      bbox = aabb(bbox_diagonal1, bbox_diagonal2);
    }

    aabb bounding_box() const override {
      return bbox;
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
      // t = (D - np) / nd 
      
      auto denom = dot(normal, r.direction());

      // no hit if ray parallel to plane.
      if (std::fabs(denom) < 1e-8) {
        return false;
      }

      // find hit point.
      auto t = (D - dot(normal, r.origin())) / denom;
      // check if hit point is in ray interval.
      if (!ray_t.contains(t)) {
        return false;
      }

      auto intersection = r.at(t);
      // check if hit point within quad using its plane coordiantes.
      // p = P - Q
      vec3 planar_hitpt_vector = intersection - Q;
      // a = w . (p x v)
      auto alpha = dot(w, cross(planar_hitpt_vector, v));
      // beta = w . (u x p)
      auto beta = dot(w, cross(u, planar_hitpt_vector));

      if (!is_interior(alpha, beta, rec)) {
        return false;
      } 
      
      rec.t = t;
      rec.p = intersection;
      rec.mat = mat;
      rec.set_face_normal(r, normal);

      return true;
    }

  virtual bool is_interior(double a, double b, hit_record& rec) const {
    // 0 <= a <= 1
    // o <= b <= 1
    interval unit_interval = interval(0, 1);

    if (!unit_interval.contains(a) || !unit_interval.contains(b)) {
      return false;
    }

    rec.u = a;
    rec.v = b;
    return true;
  }

  private:
    point3 Q; // the starting corner.
    vec3 u, v; // the vectors representing first and second side.
    shared_ptr<material> mat;
    aabb bbox;
    vec3 normal; // normal vector of quad.
    vec3 w; // w is a constant vector for a given quad.
    double D;
};

#endif