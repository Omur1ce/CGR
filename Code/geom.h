#pragma once
#include <cmath>
#include <limits>
#include <algorithm>

constexpr float kEps = 1e-5f;
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Vec3 {
    float x,y,z;
    Vec3():x(0),y(0),z(0){} Vec3(float X,float Y,float Z):x(X),y(Y),z(Z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator*(float s) const { return {x*s,y*s,z*s}; }
    Vec3 operator/(float s) const { return {x/s,y/s,z/s}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x;y+=b.y;z+=b.z; return *this; }
};
inline Vec3 operator*(float s,const Vec3& v){ return v*s; }
inline Vec3 operator*(const Vec3& a, const Vec3& b){
    return { a.x*b.x, a.y*b.y, a.z*b.z };
}
inline float dot(const Vec3& a,const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(const Vec3& a,const Vec3& b){
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
inline float length(const Vec3& v){ return std::sqrt(dot(v,v)); }
inline Vec3 normalize(const Vec3& v){ float L=length(v); return (L>kEps)? v/L : Vec3(0,0,0); }

struct Ray {
    Vec3 origin;
    Vec3 dir; // should be normalized
    float tmin = 1e-4f; // avoid self-intersections
    float tmax = kInf;
};

struct Mat4 {
    // column-major (OpenGL style)
    float m[16] = {0};
    static Mat4 identity(){
        Mat4 r; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1.0f; return r;
    }
    static Mat4 translate(const Vec3& t){
        Mat4 r = identity(); r.m[12]=t.x; r.m[13]=t.y; r.m[14]=t.z; return r;
    }
    static Mat4 scale(float s){
        Mat4 r = identity();
        r.m[0]=r.m[5]=r.m[10]=s;
        return r;
    }
    // NEW: non-uniform scale
    static Mat4 scale(const Vec3& s){
        Mat4 r = identity();
        r.m[0] = s.x;
        r.m[5] = s.y;
        r.m[10]= s.z;
        return r;
    }
    // ZYX Euler rotation in radians
    static Mat4 rotateXYZ(const Vec3& rot){
        float cx=std::cos(rot.x), sx=std::sin(rot.x);
        float cy=std::cos(rot.y), sy=std::sin(rot.y);
        float cz=std::cos(rot.z), sz=std::sin(rot.z);
        Mat4 Rx = identity(); Rx.m[5]=cx; Rx.m[6]=sx; Rx.m[9]=-sx; Rx.m[10]=cx;
        Mat4 Ry = identity(); Ry.m[0]=cy; Ry.m[2]=-sy; Ry.m[8]=sy; Ry.m[10]=cy;
        Mat4 Rz = identity(); Rz.m[0]=cz; Rz.m[1]=sz; Rz.m[4]=-sz; Rz.m[5]=cz;
        return Rz * Ry * Rx; // XYZ (apply X, then Y, then Z)
    }
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for(int c=0;c<4;++c){
            for(int rI=0;rI<4;++rI){
                r.m[c*4+rI] = m[0*4+rI]*b.m[c*4+0] + m[1*4+rI]*b.m[c*4+1] +
                              m[2*4+rI]*b.m[c*4+2] + m[3*4+rI]*b.m[c*4+3];
            }
        }
        return r;
    }
    Vec3 transformPoint(const Vec3& p) const {
        float x = m[0]*p.x + m[4]*p.y + m[8]*p.z + m[12];
        float y = m[1]*p.x + m[5]*p.y + m[9]*p.z + m[13];
        float z = m[2]*p.x + m[6]*p.y + m[10]*p.z + m[14];
        float w = m[3]*p.x + m[7]*p.y + m[11]*p.z + m[15];
        if (std::fabs(w) > kEps) { x/=w; y/=w; z/=w; }
        return {x,y,z};
    }
    Vec3 transformVector(const Vec3& v) const {
        return { m[0]*v.x + m[4]*v.y + m[8]*v.z,
                 m[1]*v.x + m[5]*v.y + m[9]*v.z,
                 m[2]*v.x + m[6]*v.y + m[10]*v.z };
    }
};

inline float det3(float a00,float a01,float a02,
                  float a10,float a11,float a12,
                  float a20,float a21,float a22){
    return a00*(a11*a22-a12*a21) - a01*(a10*a22-a12*a20) + a02*(a10*a21-a11*a20);
}

// General 4x4 inverse; adequate for TRS matrices
inline Mat4 inverse(const Mat4& A){
    // Compute inverse via adjugate/Det. For brevity we use a standard expanded form.
    // (This is compact but perfectly fine for small scene counts.)
    Mat4 inv;
    const float* m = A.m; float* o = inv.m;

    o[0] =  det3(m[5],m[6],m[7], m[9],m[10],m[11], m[13],m[14],m[15]);
    o[1] = -det3(m[1],m[2],m[3], m[9],m[10],m[11], m[13],m[14],m[15]);
    o[2] =  det3(m[1],m[2],m[3], m[5],m[6],m[7],   m[13],m[14],m[15]);
    o[3] = -det3(m[1],m[2],m[3], m[5],m[6],m[7],   m[9],m[10],m[11]);

    o[4] = -det3(m[4],m[6],m[7], m[8],m[10],m[11], m[12],m[14],m[15]);
    o[5] =  det3(m[0],m[2],m[3], m[8],m[10],m[11], m[12],m[14],m[15]);
    o[6] = -det3(m[0],m[2],m[3], m[4],m[6],m[7],   m[12],m[14],m[15]);
    o[7] =  det3(m[0],m[2],m[3], m[4],m[6],m[7],   m[8],m[10],m[11]);

    o[8] =  det3(m[4],m[5],m[7], m[8],m[9],m[11],  m[12],m[13],m[15]);
    o[9] = -det3(m[0],m[1],m[3], m[8],m[9],m[11],  m[12],m[13],m[15]);
    o[10]=  det3(m[0],m[1],m[3], m[4],m[5],m[7],   m[12],m[13],m[15]);
    o[11]= -det3(m[0],m[1],m[3], m[4],m[5],m[7],   m[8],m[9],m[11]);

    o[12]= -det3(m[4],m[5],m[6], m[8],m[9],m[10],  m[12],m[13],m[14]);
    o[13]=  det3(m[0],m[1],m[2], m[8],m[9],m[10],  m[12],m[13],m[14]);
    o[14]= -det3(m[0],m[1],m[2], m[4],m[5],m[6],   m[12],m[13],m[14]);
    o[15]=  det3(m[0],m[1],m[2], m[4],m[5],m[6],   m[8],m[9],m[10]);

    float det = m[0]*o[0] + m[4]*o[1] + m[8]*o[2] + m[12]*o[3];
    if (std::fabs(det) < kEps) return Mat4::identity();
    float invDet = 1.0f / det;
    for (int i=0;i<16;++i) o[i] *= invDet;
    return inv;
}

inline Mat4 inverseTranspose(const Mat4& M){
    Mat4 invM = inverse(M);
    // transpose
    Mat4 r;
    for(int c=0;c<4;++c) for(int rI=0;rI<4;++rI) r.m[c*4+rI] = invM.m[rI*4+c];
    return r;
}
