#pragma once
#include "geom.h"
#include <algorithm>

struct AABB {
    Vec3 min, max;
    AABB() : min( {+1e30f, +1e30f, +1e30f} ), max( {-1e30f, -1e30f, -1e30f} ) {}
    AABB(const Vec3& a, const Vec3& b) : min(a), max(b) {}

    void expand(const Vec3& p){
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }
    void expand(const AABB& b){ expand(b.min); expand(b.max); }

    int maxExtent() const {
        Vec3 d = max - min;
        if (d.x > d.y && d.x > d.z) return 0;
        if (d.y > d.z) return 1;
        return 2;
    }
};

// Ray–AABB slabs
inline bool intersectAABB(const AABB& b, const Ray& r, float tmax_in, float& tmin_out, float& tmax_out){
    float tmin = r.tmin, tmax = std::min(r.tmax, tmax_in);
    for (int a=0; a<3; ++a){
        float ro = (a==0? r.origin.x : (a==1? r.origin.y : r.origin.z));
        float rd = (a==0? r.dir.x    : (a==1? r.dir.y    : r.dir.z));
        float invD = 1.0f / (std::fabs(rd) > 1e-12f ? rd : (rd>=0? 1e-12f : -1e-12f));
        float t0 = ( (a==0? b.min.x : (a==1? b.min.y : b.min.z)) - ro) * invD;
        float t1 = ( (a==0? b.max.x : (a==1? b.max.y : b.max.z)) - ro) * invD;
        if (t0 > t1) std::swap(t0, t1);
        tmin = t0 > tmin ? t0 : tmin;
        tmax = t1 < tmax ? t1 : tmax;
        if (tmax < tmin) return false;
    }
    tmin_out = tmin; tmax_out = tmax; return true;
}
