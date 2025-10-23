#pragma once
#include "geom.h"

struct Shape; // fwd

struct Hit {
    float t = kInf;
    Vec3 p;         // hit point (world)
    Vec3 n;         // shading normal (world, unit)
    Vec3 uvw;       // generic uv or barycentric scratch
    const Shape* shape = nullptr;
};

struct Shape {
    virtual ~Shape() = default;
    virtual bool intersect(const Ray& ray, Hit& hit) const = 0;
};

// ---------------- Sphere ----------------
struct Sphere : public Shape {
    Vec3 center;
    float radius;
    Sphere(const Vec3& c, float r) : center(c), radius(r) {}
    bool intersect(const Ray& r, Hit& h) const override {
        Vec3 oc = r.origin - center;
        float a = dot(r.dir, r.dir);
        float b = 2.0f * dot(oc, r.dir);
        float c = dot(oc, oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if (disc < 0.0f) return false;
        float s = std::sqrt(disc);
        float t0 = (-b - s) / (2*a);
        float t1 = (-b + s) / (2*a);
        if (t0 > r.tmin && t0 < r.tmax && t0 < h.t) {
            h.t = t0; h.p = r.origin + r.dir * t0; h.n = normalize(h.p - center); h.shape=this; return true;
        }
        if (t1 > r.tmin && t1 < r.tmax && t1 < h.t) {
            h.t = t1; h.p = r.origin + r.dir * t1; h.n = normalize(h.p - center); h.shape=this; return true;
        }
        return false;
    }
};

// ------------- PlaneQuad (finite) -------------
// Defined by 4 corners (world) in CCW order; normal from (p1-p0)x(p3-p0)
struct PlaneQuad : public Shape {
    Vec3 p0,p1,p2,p3; // CCW (p0->p1->p2->p3)
    Vec3 normal;      // precomputed unit normal
    PlaneQuad(const Vec3& P0,const Vec3& P1,const Vec3& P2,const Vec3& P3)
        : p0(P0),p1(P1),p2(P2),p3(P3)
    {
        normal = normalize(cross(p1-p0, p3-p0));
    }
    bool intersect(const Ray& r, Hit& h) const override {
        float denom = dot(normal, r.dir);
        if (std::fabs(denom) < 1e-7f) return false; // parallel
        float t = dot(p0 - r.origin, normal) / denom;
        if (t <= r.tmin || t >= r.tmax || t >= h.t) return false;
        Vec3 P = r.origin + r.dir * t;

        // Inside-quad test: P on same side of each edge (using normal for orientation)
        auto insideEdge = [&](const Vec3& a, const Vec3& b){
            Vec3 edge = b - a;
            Vec3 cprod = cross(edge, P - a);
            return dot(cprod, normal) >= -kEps;
        };
        if ( insideEdge(p0,p1) && insideEdge(p1,p2) && insideEdge(p2,p3) && insideEdge(p3,p0) ) {
            h.t = t; h.p = P; h.n = (denom<0.0f)? normal : (normal * -1.0f); // face camera
            h.shape = this;
            return true;
        }
        return false;
    }
};

// ---------------- Cube (oriented box) ----------------
// Blender export: translation (tx,ty,tz), rotation Euler (rx,ry,rz), and uniform scale s.
// We model a unit cube in object space spanning [-0.5,0.5]^3, and build world matrix M = T * R * S.
struct Cube : public Shape {
    Mat4 M;          // object->world
    Mat4 invM;       // world->object
    Mat4 invT;       // inverse-transpose for normals

    Cube(const Vec3& translate, const Vec3& eulerXYZ, float uniformScale){
        Mat4 T = Mat4::translate(translate);
        Mat4 R = Mat4::rotateXYZ(eulerXYZ);
        Mat4 S = Mat4::scale(uniformScale);
        M = T * R * S;
        invM = inverse(M);
        invT = inverseTranspose(M);
    }

    bool intersect(const Ray& rW, Hit& h) const override {
        // Transform ray to object space
        Vec3 ro = invM.transformPoint(rW.origin);
        Vec3 rd = invM.transformVector(rW.dir);
        // Slabs with unit cube [-0.5,0.5]
        float tmin = rW.tmin, tmax = std::min(rW.tmax, h.t);
        auto slab = [&](float roC, float rdC, float minC, float maxC, float& t0, float& t1){
            if (std::fabs(rdC) < kEps){
                if (roC < minC || roC > maxC) return false; // parallel and outside
                t0 = -kInf; t1 = kInf; return true;
            }
            float inv = 1.0f/rdC;
            float tA = (minC - roC)*inv;
            float tB = (maxC - roC)*inv;
            if (tA > tB) std::swap(tA, tB);
            t0 = tA; t1 = tB; return true;
        };
        float tx0,tx1, ty0,ty1, tz0,tz1;
        if (!slab(ro.x, rd.x, -0.5f, 0.5f, tx0,tx1)) return false;
        if (!slab(ro.y, rd.y, -0.5f, 0.5f, ty0,ty1)) return false;
        if (!slab(ro.z, rd.z, -0.5f, 0.5f, tz0,tz1)) return false;

        float t0 = std::max({tx0,ty0,tz0});
        float t1 = std::min({tx1,ty1,tz1});
        if (t1 < t0) return false;
        // choose entry
        float tObj = (t0 > rW.tmin) ? t0 : t1; // handle if origin starts inside
        if (tObj <= rW.tmin || tObj >= tmax) return false;

        // Hit point & normal in object space
        Vec3 pObj = ro + rd * tObj;
        Vec3 nObj(0,0,0);
        const float kFace = 0.5f;
        if (std::fabs(pObj.x - kFace) < 1e-3f) nObj = { +1, 0, 0 };
        else if (std::fabs(pObj.x + kFace) < 1e-3f) nObj = { -1, 0, 0 };
        else if (std::fabs(pObj.y - kFace) < 1e-3f) nObj = { 0, +1, 0 };
        else if (std::fabs(pObj.y + kFace) < 1e-3f) nObj = { 0, -1, 0 };
        else if (std::fabs(pObj.z - kFace) < 1e-3f) nObj = { 0, 0, +1 };
        else                                      nObj = { 0, 0, -1 };

        // Back to world
        Vec3 pW = M.transformPoint(pObj);
        // Convert t from object to world: solve rW.origin + rW.dir * tW = pW
        float tW = (std::fabs(rW.dir.x) > kEps) ? (pW.x - rW.origin.x)/rW.dir.x :
                    (std::fabs(rW.dir.y) > kEps) ? (pW.y - rW.origin.y)/rW.dir.y :
                    (pW.z - rW.origin.z)/rW.dir.z;

        if (tW <= rW.tmin || tW >= tmax) return false;

        Vec3 nW = normalize(invT.transformVector(nObj));
        // Flip to oppose ray if needed
        if (dot(nW, rW.dir) > 0.0f) nW = nW * -1.0f;

        h.t = tW; h.p = pW; h.n = nW; h.shape = this;
        return true;
    }
};
