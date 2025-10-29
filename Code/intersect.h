#pragma once
#include "geom.h"
#include "aabb.h"

// Forward decl
struct Shape;

// ---------------- Hit record ----------------
struct Hit {
    float t = kInf;
    Vec3  p;                  // hit point (world)
    Vec3  n;                  // shading normal (world, unit)
    Vec3  uvw;                // generic payload (uv/bary/etc.)
    const Shape* shape = nullptr;
};

// ---------------- Abstract Shape ----------------
struct Shape {
    virtual ~Shape() = default;

    // Performs ray intersection test.
    virtual bool intersect(const Ray& ray, Hit& hit) const = 0;

    // Returns the world-space axis-aligned bounding box.
    virtual AABB bounds() const = 0;
};

// ---------------- Sphere (now supports non-uniform scale → ellipsoid) ----------------
struct Sphere : public Shape {
    // Represent the shape by a unit sphere at origin in object space,
    // with object->world transform M = T(center) * Scale(sx,sy,sz).
    Mat4 M;    // object->world
    Mat4 invM; // world->object
    Mat4 invT; // inverse-transpose for transforming normals

    // Non-uniform (ellipsoid) constructor
    Sphere(const Vec3& center, const Vec3& scaleVec) {
        Mat4 T = Mat4::translate(center);
        Mat4 S = Mat4::scale(scaleVec);    // requires Mat4::scale(const Vec3&) in geom.h
        M = T * S;
        invM = inverse(M);
        invT = inverseTranspose(M);
    }

    // Back-compat: uniform sphere (radius r)
    Sphere(const Vec3& center, float r)
        : Sphere(center, Vec3(r, r, r)) {}

    bool intersect(const Ray& rW, Hit& h) const override {
        // Transform ray into object space
        Vec3 ro = invM.transformPoint(rW.origin);
        Vec3 rd = invM.transformVector(rW.dir);

        // Intersect unit sphere at origin: ||ro + t rd||^2 = 1
        float a = dot(rd, rd);
        float b = 2.0f * dot(ro, rd);
        float c = dot(ro, ro) - 1.0f;
        float disc = b*b - 4*a*c;
        if (disc < 0.0f) return false;

        float s = std::sqrt(disc);
        float inv2a = 0.5f / a;
        float t0 = (-b - s) * inv2a;
        float t1 = (-b + s) * inv2a;

        auto accept = [&](float tObj, Hit& out)->bool{
            if (tObj <= rW.tmin) return false;

            // Object-space hit point and normal
            Vec3 pObj = ro + rd * tObj;
            Vec3 nObj = normalize(pObj);           // gradient of x^2+y^2+z^2-1

            // Transform to world
            Vec3 pW = M.transformPoint(pObj);
            Vec3 nW = normalize(invT.transformVector(nObj));

            // Convert to world t (solve rW.origin + rW.dir * tW = pW)
            float tW;
            if (std::fabs(rW.dir.x) > kEps)      tW = (pW.x - rW.origin.x)/rW.dir.x;
            else if (std::fabs(rW.dir.y) > kEps) tW = (pW.y - rW.origin.y)/rW.dir.y;
            else                                  tW = (pW.z - rW.origin.z)/rW.dir.z;

            if (tW <= rW.tmin || tW >= std::min(rW.tmax, out.t)) return false;

            // Ensure normal faces the ray
            if (dot(nW, rW.dir) > 0.0f) nW = nW * -1.0f;

            out.t = tW; out.p = pW; out.n = nW; out.shape = this;
            return true;
        };

        Hit tmp = h;
        bool hit = false;
        hit |= accept(t0, tmp);
        hit |= accept(t1, tmp);
        if (hit) { h = tmp; return true; }
        return false;
    }

    AABB bounds() const override {
        // Transform the 8 corners of the unit sphere’s bounding cube [-1,1]^3
        const float k = 1.0f;
        const Vec3 c[8] = {
            {-k,-k,-k},{+k,-k,-k},{-k,+k,-k},{+k,+k,-k},
            {-k,-k,+k},{+k,-k,+k},{-k,+k,+k},{+k,+k,+k}
        };
        AABB b;
        for (const Vec3& p : c) b.expand(M.transformPoint(p));
        return b;
    }
};


// ------------- PlaneQuad (finite) -------------
// Defined by 4 corners (world) in CCW order; normal from (p1-p0) x (p3-p0)
struct PlaneQuad : public Shape {
    Vec3 p0,p1,p2,p3; // CCW (p0->p1->p2->p3)
    Vec3 normal;      // unit normal

    PlaneQuad(const Vec3& P0,const Vec3& P1,const Vec3& P2,const Vec3& P3)
        : p0(P0),p1(P1),p2(P2),p3(P3)
    {
        normal = normalize(cross(p1 - p0, p3 - p0));
    }

    bool intersect(const Ray& r, Hit& h) const override {
        float denom = dot(normal, r.dir);
        if (std::fabs(denom) < 1e-7f) return false; // parallel

        float t = dot(p0 - r.origin, normal) / denom;
        if (t <= r.tmin || t >= r.tmax || t >= h.t) return false;

        Vec3 P = r.origin + r.dir * t;

        // Inside-quad test: P must be on the same side of all edges
        auto insideEdge = [&](const Vec3& a, const Vec3& b){
            Vec3 edge = b - a;
            Vec3 cprod = cross(edge, P - a);
            return dot(cprod, normal) >= -kEps;
        };

        if ( insideEdge(p0,p1) && insideEdge(p1,p2) &&
             insideEdge(p2,p3) && insideEdge(p3,p0) )
        {
            h.t = t; h.p = P;
            // Face camera (flip if we hit the back face)
            h.n = (denom < 0.0f) ? normal : (normal * -1.0f);
            h.shape = this;
            return true;
        }
        return false;
    }

    AABB bounds() const override {
        AABB b;
        b.expand(p0); b.expand(p1); b.expand(p2); b.expand(p3);
        const Vec3 eps(1e-4f,1e-4f,1e-4f);
        b.min = b.min - eps;
        b.max = b.max + eps;
        return b;
    }
};

// ---------------- Cube (oriented box) ----------------
// Blender export: translation (tx,ty,tz), rotation Euler (rx,ry,rz), and scale (uniform or non-uniform).
// We model a unit cube in object space spanning [-0.5,0.5]^3, and build world matrix M = T * R * S.
struct Cube : public Shape {
    Mat4 M;    // object->world
    Mat4 invM; // world->object
    Mat4 invT; // inverse-transpose for normals

    // New: non-uniform scale constructor
    Cube(const Vec3& translate, const Vec3& eulerXYZ, const Vec3& scaleVec)
        : M(), invM(), invT()
    {
        Mat4 T = Mat4::translate(translate);
        Mat4 R = Mat4::rotateXYZ(eulerXYZ);
        Mat4 S = Mat4::scale(scaleVec);  // needs Mat4::scale(const Vec3&) in geom.h
        M = T * R * S;
        invM = inverse(M);
        invT = inverseTranspose(M);
    }

    // Back-compat: uniform scale constructor (so existing code with float still compiles)
    Cube(const Vec3& translate, const Vec3& eulerXYZ, float uniformScale)
        : Cube(translate, eulerXYZ, Vec3(uniformScale, uniformScale, uniformScale))
    {}

    bool intersect(const Ray& rW, Hit& h) const override {
        // Transform ray to object space
        Vec3 ro = invM.transformPoint(rW.origin);
        Vec3 rd = invM.transformVector(rW.dir);

        // Slab test with unit cube [-0.5,0.5]
        float tmaxLimit = std::min(rW.tmax, h.t);

        auto slab = [](float roC, float rdC, float minC, float maxC, float& t0, float& t1){
            if (std::fabs(rdC) < kEps){
                if (roC < minC || roC > maxC) return false; // parallel and outside
                t0 = -kInf; t1 = +kInf; return true;
            }
            float inv = 1.0f / rdC;
            float tA = (minC - roC) * inv;
            float tB = (maxC - roC) * inv;
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

        // Choose entry; handle origin-inside case
        float tObj = (t0 > rW.tmin) ? t0 : t1;
        if (tObj <= rW.tmin || tObj >= tmaxLimit) return false;

        // Hit point & normal in object space
        Vec3 pObj = ro + rd * tObj;
        Vec3 nObj(0,0,0);
        const float kFace = 0.5f;
        const float eps   = 1e-3f;
        if      (std::fabs(pObj.x - kFace) < eps) nObj = { +1, 0, 0 };
        else if (std::fabs(pObj.x + kFace) < eps) nObj = { -1, 0, 0 };
        else if (std::fabs(pObj.y - kFace) < eps) nObj = { 0, +1, 0 };
        else if (std::fabs(pObj.y + kFace) < eps) nObj = { 0, -1, 0 };
        else if (std::fabs(pObj.z - kFace) < eps) nObj = { 0, 0, +1 };
        else                                       nObj = { 0, 0, -1 };

        // Back to world
        Vec3 pW = M.transformPoint(pObj);

        // Convert t from object to world: solve rW.origin + rW.dir * tW = pW
        float tW;
        if (std::fabs(rW.dir.x) > kEps)      tW = (pW.x - rW.origin.x) / rW.dir.x;
        else if (std::fabs(rW.dir.y) > kEps) tW = (pW.y - rW.origin.y) / rW.dir.y;
        else                                  tW = (pW.z - rW.origin.z) / rW.dir.z;

        if (tW <= rW.tmin || tW >= tmaxLimit) return false;

        Vec3 nW = normalize(invT.transformVector(nObj));
        if (dot(nW, rW.dir) > 0.0f) nW = nW * -1.0f; // face the ray

        h.t = tW; h.p = pW; h.n = nW; h.shape = this;
        return true;
    }

    AABB bounds() const override {
        // Transform 8 corners of the unit cube and take min/max
        const float k = 0.5f;
        const Vec3 corners[8] = {
            {-k,-k,-k},{+k,-k,-k},{-k,+k,-k},{+k,+k,-k},
            {-k,-k,+k},{+k,-k,+k},{-k,+k,+k},{+k,+k,+k}
        };
        AABB b;
        for (const Vec3& c : corners)
            b.expand(M.transformPoint(c));
        return b;
    }
};
