#pragma once
#include "geom.h"
#include "aabb.h"

struct Shape; // fwd

struct Hit {
    float t = kInf;
    Vec3  p;         // hit point (world)
    Vec3  n;         // shading normal (world, unit)
    Vec3  uvw;       // (u,v,extra) for texturing
    const Shape* shape = nullptr;
};

struct Shape {
    virtual ~Shape() = default;
    virtual bool intersect(const Ray& ray, Hit& hit) const = 0;
    virtual AABB bounds() const = 0;
};

// ===================== Sphere / Ellipsoid =====================
// Supports both uniform spheres and non-uniform ellipsoids via per-axis scale.
struct Sphere : public Shape {
    Vec3 center;  // world-space center
    Vec3 scale;   // per-axis scale; uniform sphere if x=y=z

    // Uniform sphere: radius r
    Sphere(const Vec3& c, float r)
        : center(c), scale(r, r, r) {}

    // Ellipsoid: per-axis scale
    Sphere(const Vec3& c, const Vec3& s)
        : center(c), scale(s) {}

    bool intersect(const Ray& rW, Hit& h) const override {
        // Avoid division by zero in degenerate scales
        Vec3 invS(
            (std::fabs(scale.x) > kEps) ? 1.0f/scale.x : 0.0f,
            (std::fabs(scale.y) > kEps) ? 1.0f/scale.y : 0.0f,
            (std::fabs(scale.z) > kEps) ? 1.0f/scale.z : 0.0f
        );

        // Transform ray into "unit-sphere space" where ellipsoid is x^2+y^2+z^2 = 1
        Vec3 ocW = rW.origin - center;
        Vec3 ro( ocW.x * invS.x, ocW.y * invS.y, ocW.z * invS.z );
        Vec3 rd( rW.dir.x * invS.x, rW.dir.y * invS.y, rW.dir.z * invS.z );

        float a = dot(rd, rd);
        float b = 2.0f * dot(ro, rd);
        float c = dot(ro, ro) - 1.0f;   // radius=1 in this scaled space

        float disc = b*b - 4*a*c;
        if (disc < 0.0f) return false;
        float s = std::sqrt(disc);

        float t0 = (-b - s) / (2*a);
        float t1 = (-b + s) / (2*a);

        auto tryHit = [&](float t) -> bool {
            if (t <= rW.tmin || t >= rW.tmax || t >= h.t) return false;
            Vec3 pW = rW.origin + rW.dir * t;

            // Normal = gradient of ellipsoid at p:
            // F(x,y,z) = (x-cx)^2/sx^2 + ... - 1
            Vec3 d = pW - center;
            Vec3 n(
                (std::fabs(scale.x) > kEps) ? d.x / (scale.x*scale.x) : 0.0f,
                (std::fabs(scale.y) > kEps) ? d.y / (scale.y*scale.y) : 0.0f,
                (std::fabs(scale.z) > kEps) ? d.z / (scale.z*scale.z) : 0.0f
            );
            n = normalize(n);

            // Spherical UVs using unit-sphere direction
            // Use the normalized point in "unit sphere" space.
            Vec3 pLocal(
                d.x * invS.x,
                d.y * invS.y,
                d.z * invS.z
            );
            pLocal = normalize(pLocal);
            const float PI = 3.14159265358979323846f;

            float u = 0.5f + std::atan2(pLocal.z, pLocal.x) / (2.0f * PI);
            float v = 0.5f - std::asin(pLocal.y) / PI;

            h.t   = t;
            h.p   = pW;
            h.n   = n;
            h.uvw = Vec3(u, v, 0.0f);
            h.shape = this;
            return true;
        };

        bool hit = false;
        hit |= tryHit(t0);
        hit |= tryHit(t1);
        return hit;
    }

    AABB bounds() const override {
        // Simple axis-aligned bounds ignoring any rotation (spheres have none here)
        Vec3 ext(std::fabs(scale.x), std::fabs(scale.y), std::fabs(scale.z));
        return AABB(center - ext, center + ext);
    }
};

// ===================== PlaneQuad (finite) =====================
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

        // Inside-quad test: P on same side of each edge
        auto insideEdge = [&](const Vec3& a, const Vec3& b){
            Vec3 edge = b - a;
            Vec3 cprod = cross(edge, P - a);
            return dot(cprod, normal) >= -kEps;
        };
        if ( insideEdge(p0,p1) && insideEdge(p1,p2) &&
             insideEdge(p2,p3) && insideEdge(p3,p0) ) {

            // Compute UVs using bilinear coordinates (from p0 toward p1 and p3)
            Vec3 e1 = p1 - p0;
            Vec3 e3 = p3 - p0;
            Vec3 v  = P  - p0;

            float u = 0.0f;
            float vcoord = 0.0f;
            float e1len2 = dot(e1,e1);
            float e3len2 = dot(e3,e3);
            if (e1len2 > kEps) u = dot(v, e1) / e1len2;
            if (e3len2 > kEps) vcoord = dot(v, e3) / e3len2;

            u      = std::max(0.0f, std::min(1.0f, u));
            vcoord = std::max(0.0f, std::min(1.0f, vcoord));

            Vec3 n = (denom < 0.0f) ? normal : (normal * -1.0f);

            h.t   = t;
            h.p   = P;
            h.n   = n;
            h.uvw = Vec3(u, vcoord, 0.0f);
            h.shape = this;
            return true;
        }
        return false;
    }

    AABB bounds() const override {
        AABB b;
        b.expand(p0); b.expand(p1); b.expand(p2); b.expand(p3);
        Vec3 eps(1e-4f, 1e-4f, 1e-4f);
        b.min = b.min - eps;
        b.max = b.max + eps;
        return b;
    }
};

// ===================== Cube (oriented box) =====================
// Unit cube in object space [-0.5,0.5]^3; world transform via M.
// Blender export: translation (tx,ty,tz), rotation Euler (rx,ry,rz), per-axis scale.
struct Cube : public Shape {
    Mat4 M;          // object->world
    Mat4 invM;       // world->object
    Mat4 invT;       // inverse-transpose for normals

    Cube(const Vec3& translate, const Vec3& eulerXYZ, float uniformScale){
        Vec3 Svec(uniformScale, uniformScale, uniformScale);
        init(translate, eulerXYZ, Svec);
    }

    Cube(const Vec3& translate, const Vec3& eulerXYZ, const Vec3& scaleVec){
        init(translate, eulerXYZ, scaleVec);
    }

    void init(const Vec3& translate, const Vec3& eulerXYZ, const Vec3& scaleVec){
        Mat4 T = Mat4::translate(translate);
        Mat4 R = Mat4::rotateXYZ(eulerXYZ);
        Mat4 S = Mat4::scale(scaleVec);
        M      = T * R * S;
        invM   = inverse(M);
        invT   = inverseTranspose(M);
    }

    bool intersect(const Ray& rW, Hit& h) const override {
        // Transform ray to object space
        Vec3 ro = invM.transformPoint(rW.origin);
        Vec3 rd = invM.transformVector(rW.dir);

        // Slabs with unit cube [-0.5,0.5]
        float tmin = rW.tmin, tmax = std::min(rW.tmax, h.t);
        tmin = tmin+1;
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
        //const float kFace = 0.5f;

        float ax = std::fabs(pObj.x);
        float ay = std::fabs(pObj.y);
        float az = std::fabs(pObj.z);

        // Determine which face we hit (largest axis)
        if (ax >= ay && ax >= az) {
            nObj = (pObj.x > 0.0f) ? Vec3(+1,0,0) : Vec3(-1,0,0);
        } else if (ay >= ax && ay >= az) {
            nObj = (pObj.y > 0.0f) ? Vec3(0,+1,0) : Vec3(0,-1,0);
        } else {
            nObj = (pObj.z > 0.0f) ? Vec3(0,0,+1) : Vec3(0,0,-1);
        }

        // Compute UVs per face in object space
        float u = 0.0f, v = 0.0f;
        // X faces: use (z,y)
        if (std::fabs(nObj.x) > 0.5f){
            u = (pObj.z + 0.5f);
            v = (pObj.y + 0.5f);
        }
        // Y faces: use (x,z)
        else if (std::fabs(nObj.y) > 0.5f){
            u = (pObj.x + 0.5f);
            v = (pObj.z + 0.5f);
        }
        // Z faces: use (x,y)
        else {
            u = (pObj.x + 0.5f);
            v = (pObj.y + 0.5f);
        }
        // Clamp to [0,1]
        u = std::max(0.0f, std::min(1.0f, u));
        v = std::max(0.0f, std::min(1.0f, v));

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

        h.t   = tW;
        h.p   = pW;
        h.n   = nW;
        h.uvw = Vec3(u, v, 0.0f);
        h.shape = this;
        return true;
    }

    AABB bounds() const override {
        const float k = 0.5f;
        Vec3 corners[8] = {
            {-k,-k,-k},{+k,-k,-k},{-k,+k,-k},{+k,+k,-k},
            {-k,-k,+k},{+k,-k,+k},{-k,+k,+k},{+k,+k,+k}
        };
        AABB b;
        for (auto& c : corners)
            b.expand(M.transformPoint(c));
        return b;
    }
};
