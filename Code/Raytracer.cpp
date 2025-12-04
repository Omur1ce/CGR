#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <chrono>
#include <unordered_map>
#include <random>

#include "geom.h"
#include "intersect.h"
#include "camera.h"
#include "bvh.h"
#include "texture.h"

bool g_enable_textures = true;
static const float PI = 3.14159265358979323846f;

// ---------------- Small helpers ----------------
static inline std::string trim(const std::string& s){
    size_t a=s.find_first_not_of(" \t\r\n");
    size_t b=s.find_last_not_of(" \t\r\n");
    return (a==std::string::npos)? "" : s.substr(a,b-a+1);
}

template<typename T=float>
std::vector<T> parseArray1D(std::ifstream& f, std::string line){
    std::vector<T> out;
    size_t s=line.find('[');
    if(s!=std::string::npos) line=line.substr(s+1);
    while(true){
        size_t e=line.find(']');
        std::string chunk=(e!=std::string::npos)? line.substr(0,e):line;
        std::stringstream ss(chunk);
        std::string item;
        while(std::getline(ss,item,',')){
            item=trim(item);
            if(item.empty()||item=="["||item=="]"||item.front()=='"'||item.front()=='\'') continue;
            try{ out.push_back((T)std::stod(item)); }catch(...){}
        }
        if(e!=std::string::npos) break;
        if(!std::getline(f,line)) break;
        line=trim(line);
    }
    return out;
}

static inline int countChar(const std::string& s, char c){
    int n=0; for(char ch: s) if(ch==c) ++n; return n;
}

// ---------- Random helpers for soft shadows / DOF / motion blur ----------
static std::mt19937 g_rng(12345);  // fixed seed for reproducible renders

inline float rand01() {
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(g_rng);
}

// Random point inside unit sphere
inline Vec3 random_in_unit_sphere() {
    while (true) {
        float x = rand01()*2.0f - 1.0f;
        float y = rand01()*2.0f - 1.0f;
        float z = rand01()*2.0f - 1.0f;
        float r2 = x*x + y*y + z*z;
        if (r2 <= 1.0f) return Vec3(x,y,z);
    }
}

inline void random_in_unit_disk(float &x, float &y) {
    float r = std::sqrt(rand01());
    float phi = 2.0f * PI * rand01();
    x = r * std::cos(phi);
    y = r * std::sin(phi);
}

// Jitter a perfect reflection direction into a "glossy" lobe
inline Vec3 jitter_reflection(const Vec3& R, float roughness, const Vec3& normal)
{
    // R should be unit length
    Vec3 dir = R;
    if (roughness > 0.0f) {
        // Small random vector in unit sphere, scaled by roughness
        Vec3 jitter = random_in_unit_sphere() * roughness;
        dir = normalize(R + jitter);
        // Make sure the glossy ray is still on the same side of the surface as the ideal reflection
        if (dot(dir, normal) < 0.0f) {
            dir = dir * -1.0f;
        }
    }
    return dir;
}


// ---------------- Lighting & Materials ----------------
struct PointLight {
    Vec3 position;
    float intensity;   // scalar; 1/r^2 falloff
    float radius = 0.0f; // 0 => hard shadow, >0 => soft shadow area light
};

struct Material {
    Vec3  kd = Vec3(0.8f,0.8f,0.8f); // diffuse
    Vec3  ks = Vec3(0.2f,0.2f,0.2f); // specular
    float shininess    = 32.0f;      // Blinn exponent
    float reflectivity = 0.0f;       // 0..1
    float transmissive = 0.0f;       // 0..1
    float ior          = 1.5f;       // refractive index
};

// Parse a "material" block inline within an open JSON object.
static inline void maybeParseMaterial(std::ifstream& f, const std::string& line, Material& M){
    std::string L = line;
    if (L.find("\"kd\"") != std::string::npos){
        auto a = parseArray1D<double>(f, L);
        if(a.size()==3) M.kd = Vec3((float)a[0],(float)a[1],(float)a[2]);
    } else if (L.find("\"ks\"") != std::string::npos){
        auto a = parseArray1D<double>(f, L);
        if(a.size()==3) M.ks = Vec3((float)a[0],(float)a[1],(float)a[2]);
    } else if (L.find("\"shininess\"") != std::string::npos){
        size_t c=L.find(':'); if(c!=std::string::npos){
            std::string v=trim(L.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
            try{ M.shininess = (float)std::stod(v);}catch(...){}
        }
    } else if (L.find("\"reflectivity\"") != std::string::npos){
        size_t c=L.find(':'); if(c!=std::string::npos){
            std::string v=trim(L.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
            try{ M.reflectivity = (float)std::stod(v);}catch(...){}
        }
    } else if (L.find("\"transmissive\"") != std::string::npos){
        size_t c=L.find(':'); if(c!=std::string::npos){
            std::string v=trim(L.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
            try{ M.transmissive = (float)std::stod(v);}catch(...){}
        }
    } else if (L.find("\"ior\"") != std::string::npos){
        size_t c=L.find(':'); if(c!=std::string::npos){
            std::string v=trim(L.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
            try{ M.ior = (float)std::stod(v);}catch(...){}
        }
    }
}

inline Vec3 clamp01(const Vec3& v){
    auto c = [](float x){ return std::max(0.f, std::min(1.f, x)); };
    return Vec3(c(v.x), c(v.y), c(v.z));
}
inline Vec3 reflect(const Vec3& I, const Vec3& N){
    return I - N * (2.0f * dot(I, N));
}
inline bool refract(const Vec3& I, const Vec3& N, float eta, Vec3& T){
    float cosi = -std::max(-1.f, std::min(1.f, dot(I, N)));
    float sint2 = eta*eta * (1.0f - cosi*cosi);
    if (sint2 > 1.0f) return false; // total internal reflection
    float cost = std::sqrt(std::max(0.f, 1.0f - sint2));
    T = I * eta + N * (eta * cosi - cost);
    return true;
}
inline float fresnel_schlick(float cosTheta, float F0){
    return F0 + (1.0f - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

static const int   kMaxDepth   = 4;
static const float kShadowBias = 1e-3f;

// How many shadow samples per light (area light sampling)
static const int kShadowSamples = 8;

static const int   kGlossySamples   = 2;    // number of reflection rays
static const float kMinGlossRough   = 0.02f;
static const float kMaxGlossRough   = 0.3f;

// -------- Feature toggles & DOF / motion blur params --------
bool  g_enable_dof           = true;   // depth of field
bool  g_enable_motion_blur   = true;   // per-ray time sampling
bool  g_enable_soft_shadows  = true;   // area-light sampling
bool  g_enable_glossy        = true;   // glossy reflections
bool  g_enable_aa            = true;  // anti aliasing

float g_lens_radius  = 0.01f;        // radius of lens in world units (tune!)
float g_focus_dist   = 2.5f;          // distance from camera to focal plane



// Motion blur shutter interval in normalized time [0,1]
static const float g_shutter_open  = 0.0f;
static const float g_shutter_close = 1.0f;

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}



Vec3 shadeHit(const Hit& hit, const Vec3& viewDir,
              float rayTime,
              const std::vector<PointLight>& lights,
              const std::unordered_map<const Shape*, Material>& matOf,
              const std::vector<std::unique_ptr<Shape>>& shapes,
              const BVH* bvh,
              const Texture* tex)

{
    auto it = matOf.find(hit.shape);
    Material mat = (it != matOf.end()) ? it->second : Material{};

    Vec3 N = hit.n;
    Vec3 V = viewDir;
    Vec3 color(0,0,0);

    // Decide once whether we actually use textures
    bool useTex = (tex != nullptr) && tex->valid() && g_enable_textures;

    for (const auto& Ls : lights) {
        // Decide if this light should behave as an area light
        bool soft = (g_enable_soft_shadows && Ls.radius > 0.0f && kShadowSamples > 1);
        int  samples = soft ? kShadowSamples : 1;

        Vec3 lightAccum(0,0,0);

        for (int s = 0; s < samples; ++s) {
            // Point or area light position
            Vec3 lightPos = Ls.position;
            if (soft) {
                // Only jitter when soft shadows are enabled
                Vec3 jitter = random_in_unit_sphere() * Ls.radius;
                lightPos += jitter;
            }

            Vec3 Ldir = lightPos - hit.p;
            float dist2 = dot(Ldir, Ldir);
            float dist  = std::sqrt(std::max(dist2, 1e-12f));
            Ldir = Ldir / dist;

            // Shadow ray
            Ray shadowRay;
            shadowRay.origin = hit.p + N * kShadowBias;
            shadowRay.dir    = Ldir;
            shadowRay.tmin   = 0.0f;
            shadowRay.tmax   = dist - 1e-3f;
            shadowRay.time   = rayTime;

            bool occluded = false;
            if (bvh) {
                Hit occ;
                occluded = bvh->intersect(shadowRay, occ);
            } else {
                for (const auto& s : shapes) {
                    Hit htmp;
                    if (s->intersect(shadowRay, htmp)) { occluded = true; break; }
                }
            }
            if (occluded) continue;

            // --- Blinn-Phong for this sample ---
            float NdotL = std::max(0.f, dot(N, Ldir));
            if (NdotL <= 0.0f) continue;

            // Sample texture / albedo
            Vec3 albedo(1.0f, 1.0f, 1.0f);
            if (useTex) {
                float u = hit.uvw.x;
                float v = hit.uvw.y;
                albedo = tex->sample(u, v);   // already in [0,1]
            }

            // Component-wise multiply kd by albedo
            Vec3 kd_eff(
                mat.kd.x * albedo.x,
                mat.kd.y * albedo.y,
                mat.kd.z * albedo.z
            );

            Vec3 diffuse = kd_eff * NdotL;

            Vec3 H = normalize(Ldir + V);
            float NdotH = std::max(0.f, dot(N, H));
            Vec3 spec = mat.ks * std::pow(NdotH, mat.shininess);

            float atten = Ls.intensity / std::max(1.f, dist2);
            lightAccum += (diffuse + spec) * atten;
        }

        if (samples > 0) {
            color += lightAccum * (1.0f / float(samples));
        }
    }

    return clamp01(color);
}


// Radiance along a ray with recursion for reflections/refractions
Vec3 traceRay(const Ray& r,
              const std::vector<std::unique_ptr<Shape>>& shapes,
              const BVH* bvh,
              const std::unordered_map<const Shape*, Material>& matOf,
              const std::vector<PointLight>& lights,
              const Texture* tex,
              int depth)
{
    if (depth > kMaxDepth) return Vec3(0,0,0);

    Hit hit; bool ok=false;
    if (bvh) ok = bvh->intersect(r, hit);
    else {
        for(const auto& s: shapes){
            Hit h; 
            if (s->intersect(r,h) && h.t < hit.t) { 
                hit = h; 
                ok  = true; 
            }
        }
    }
    if (!ok) return Vec3(0.2f,0.3f,0.5f); // background

    const Material& mat = (matOf.count(hit.shape)? matOf.at(hit.shape) : Material{});
    Vec3 V = normalize((r.dir) * -1.0f);

    // Direct lighting (with optional texture)
    Vec3 Lo = shadeHit(hit, V, r.time, lights, matOf, shapes, bvh, tex);


    // Secondary rays
    Vec3 N = hit.n;
    Vec3 accum = Lo;

    // Reflection (glossy via distributed ray tracing, toggleable)
    if (mat.reflectivity > 0.0f){
        Vec3 Rideal = normalize(reflect(r.dir, N));

        Vec3 reflAccum(0,0,0);

        if (g_enable_glossy) {
            // Glossy: multiple jittered rays in a lobe
            float glossRough = 0.5f / std::sqrt(std::max(1.0f, mat.shininess));
            glossRough = std::max(kMinGlossRough, std::min(kMaxGlossRough, glossRough));

            int  nSamples = kGlossySamples;
            for (int i = 0; i < nSamples; ++i) {
                Vec3 Rdir = jitter_reflection(Rideal, glossRough, N);

                Ray rRef;
                rRef.origin = hit.p + N * kShadowBias;
                rRef.dir    = Rdir;
                rRef.time   = r.time;    // propagate shutter time to reflection ray

                reflAccum += traceRay(rRef, shapes, bvh, matOf, lights, tex, depth+1);
            }
            reflAccum = reflAccum * (1.0f / float(nSamples));
        } else {
            // Plain mirror: single perfect reflection
            Ray rRef;
            rRef.origin = hit.p + N * kShadowBias;
            rRef.dir    = Rideal;
            rRef.time   = r.time;

            reflAccum = traceRay(rRef, shapes, bvh, matOf, lights, tex, depth+1);
        }

        accum += reflAccum * mat.reflectivity;
    }

    // Refraction (glass)
    if (mat.transmissive > 0.0f){
        float n1 = 1.0f, n2 = mat.ior;
        Vec3  Nf = N;
        float cosi = std::max(-1.f, std::min(1.f, dot(r.dir, N)));
        if (cosi > 0.0f) { // exiting
            std::swap(n1, n2);
            Nf = N * -1.0f;
            cosi = -cosi;
        }
        float eta = n1 / n2;

        Vec3 Tdir;
        if (refract(r.dir, Nf, eta, Tdir)){
            float R0 = std::pow((n1 - n2)/(n1 + n2), 2.0f);
            float Fr = fresnel_schlick(std::fabs(cosi), R0);

            Ray rRefr;
            rRefr.origin = hit.p - N * kShadowBias;
            rRefr.dir    = normalize(Tdir);
            rRefr.time   = r.time;   // propagate shutter time to refraction ray

            Vec3 Lt = traceRay(rRefr, shapes, bvh, matOf, lights, tex, depth+1);

            // Mix local + transmission (reflection already added above)
            accum = accum * (1.0f - mat.transmissive)
                  + Lt * (mat.transmissive * (1.0f - Fr));
        }
        // else: TIR -> reflection term already present
    }

    return clamp01(accum);
}


int main(int argc, char** argv){
    // --- Parse command-line flags ---
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--textures" || arg == "-t") {
            g_enable_textures = true;
        }
        else if (arg == "--no-textures" || arg == "-nt") {
            g_enable_textures = false;
        }
        else if (arg == "--no-dof") {
            g_enable_dof = false;
        }
        else if (arg == "--no-motion-blur") {
            g_enable_motion_blur = false;
        }
        else if (arg == "--no-soft-shadows") {
            g_enable_soft_shadows = false;
        }
        else if (arg == "--no-glossy") {
            g_enable_glossy = false;
        }
        else if (arg == "--no-aa") {
            g_enable_aa = false;
        }
        else {
            std::cout << "Unknown argument: " << arg << "\n";
        }
    }

    std::cout << "Textures:       " << (g_enable_textures      ? "ON" : "OFF") << "\n";
    std::cout << "DOF:            " << (g_enable_dof           ? "ON" : "OFF") << "\n";
    std::cout << "Motion blur:    " << (g_enable_motion_blur   ? "ON" : "OFF") << "\n";
    std::cout << "Soft shadows:   " << (g_enable_soft_shadows  ? "ON" : "OFF") << "\n";
    std::cout << "Glossy refl:    " << (g_enable_glossy        ? "ON" : "OFF") << "\n";

    // ---------- Load camera ----------
    Camera cam;
    cam.read_from_file("../ASCII/current.json");
    cam.print_info();

    Vec3 camEye, camRight, camUp, camForward;
    cam.get_eye_and_basis(camEye, camRight, camUp, camForward);

    // ---------- Load shapes + materials from JSON ----------
    std::vector<std::unique_ptr<Shape>> shapes;
    std::unordered_map<const Shape*, Material> matOf;

    std::ifstream file("../ASCII/current.json");
    if(!file.is_open()){ std::cerr<<"cannot open scene\n"; return 1; }

    std::string line;
    enum {NONE, CUBES, PLANES, SPHERES} section = NONE;
    while(std::getline(file,line)){
        line=trim(line);
        if(line.find("\"cubes\"")   != std::string::npos){ section=CUBES;   continue; }
        if(line.find("\"planes\"")  != std::string::npos){ section=PLANES;  continue; }
        if(line.find("\"spheres\"") != std::string::npos){ section=SPHERES; continue; }

        if(line.find("{")!=std::string::npos){
            if(section==CUBES){
                Vec3 T(0,0,0), R(0,0,0), S(1,1,1);
                Material M; // parsed material for this object
                int depth = 1; // we saw one '{'
                while(std::getline(file,line)){
                    line=trim(line);
                    depth += countChar(line,'{');
                    depth -= countChar(line,'}');
                    if(line.find("\"translation\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) T=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"rotation\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) R=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"scale\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) S=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"material\"")!=std::string::npos || line.find("\"kd\"")!=std::string::npos ||
                              line.find("\"ks\"")!=std::string::npos || line.find("\"shininess\"")!=std::string::npos ||
                              line.find("\"reflectivity\"")!=std::string::npos || line.find("\"transmissive\"")!=std::string::npos ||
                              line.find("\"ior\"")!=std::string::npos){
                        maybeParseMaterial(file, line, M);
                    }
                    if(depth==0) break;
                }
                shapes.emplace_back(std::make_unique<Cube>(T,R,S));
                matOf[shapes.back().get()] = M;
            }
            else if(section==PLANES){
                Vec3 c[4]; int got=0;
                Material M;
                int depth=1;
                while(std::getline(file,line)){
                    line=trim(line);
                    depth += countChar(line,'{');
                    depth -= countChar(line,'}');
                    if(line.find('[')!=std::string::npos && got<4){
                        auto v=parseArray1D<double>(file,line);
                        if(v.size()==3) c[got++]=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"material\"")!=std::string::npos || line.find("\"kd\"")!=std::string::npos ||
                              line.find("\"ks\"")!=std::string::npos || line.find("\"shininess\"")!=std::string::npos ||
                              line.find("\"reflectivity\"")!=std::string::npos || line.find("\"transmissive\"")!=std::string::npos ||
                              line.find("\"ior\"")!=std::string::npos){
                        maybeParseMaterial(file, line, M);
                    }
                    if(depth==0) break;
                }
                if(got==4){
                    shapes.emplace_back(std::make_unique<PlaneQuad>(c[0],c[1],c[2],c[3]));
                    matOf[shapes.back().get()] = M;
                }
            }
            else if(section==SPHERES){
                Vec3 C(0,0,0);
                Vec3 S(1,1,1);
                bool haveScale  = false;
                float r = 1.0f;
                bool haveRadius = false;
                Material M;

                // NEW: velocity per sphere from JSON
                Vec3 vel(0,0,0);
                bool haveVel = false;

                int depth=1;
                while(std::getline(file,line)){
                    line=trim(line);
                    depth += countChar(line,'{');
                    depth -= countChar(line,'}');

                    if(line.find("\"location\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); 
                        if(v.size()==3) C=Vec3(v[0],v[1],v[2]);
                    } 
                    else if(line.find("\"scale\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); 
                        if(v.size()==3){ S=Vec3(v[0],v[1],v[2]); haveScale=true; }
                    } 
                    else if(line.find("\"radius\"")!=std::string::npos){
                        size_t cpos=line.find(':'); 
                        if(cpos!=std::string::npos){
                            std::string v=trim(line.substr(cpos+1)); 
                            if(!v.empty()&&v.back()==',') v.pop_back();
                            try{ r=(float)std::stod(v); haveRadius=true; }catch(...){}
                        }
                    }
                    // NEW: parse optional "velocity": [vx, vy, vz]
                    else if(line.find("\"velocity\"") != std::string::npos){
                        auto v = parseArray1D<double>(file, line);
                        if (v.size() == 3) {
                            vel = Vec3((float)v[0], (float)v[1], (float)v[2]);
                            haveVel = true;
                        }
                    }
                    else if(line.find("\"material\"")!=std::string::npos || line.find("\"kd\"")!=std::string::npos ||
                            line.find("\"ks\"")!=std::string::npos || line.find("\"shininess\"")!=std::string::npos ||
                            line.find("\"reflectivity\"")!=std::string::npos || line.find("\"transmissive\"")!=std::string::npos ||
                            line.find("\"ior\"")!=std::string::npos){
                        maybeParseMaterial(file, line, M);
                    }

                    if(depth==0) break;
                }

                Sphere* sphPtr = nullptr;
                if (haveScale) {
                    auto sph = std::make_unique<Sphere>(C, S);      // Ellipsoid
                    sphPtr = sph.get();
                    shapes.emplace_back(std::move(sph));
                } else {
                    if(!haveRadius && haveScale) 
                        r = std::max(S.x,std::max(S.y,S.z));
                    auto sph = std::make_unique<Sphere>(C, r);      // Uniform
                    sphPtr = sph.get();
                    shapes.emplace_back(std::move(sph));
                }

                // Apply parsed velocity if present
                if (sphPtr && haveVel) {
                    sphPtr->velocity = vel;
                }

                matOf[shapes.back().get()] = M;
            }

        }
        if(line.find("]")!=std::string::npos) section=NONE;
    }
    file.close();

    std::cout << "Loaded " << shapes.size() << " shapes.\n";

    // ---------- Load point lights ----------
    std::vector<PointLight> lights;
    {
        std::ifstream lf("../ASCII/current.json");
        if(lf.is_open()){
            std::string L; enum {LNONE, LPOINTS} lsec = LNONE;
            while(std::getline(lf,L)){
                L = trim(L);
                if(L.find("\"point_lights\"") != std::string::npos){ 
                    lsec = LPOINTS; 
                    continue; 
                }
                if(L.find("{")!=std::string::npos && lsec==LPOINTS){
                    Vec3 pos(0,0,0); 
                    float I   = 100.0f;
                    float rad = 0.0f;   // start fresh per-light
                    int depth = 1;

                    while(std::getline(lf,L)){
                        L = trim(L);
                        depth += countChar(L,'{');
                        depth -= countChar(L,'}');

                        if(L.find("\"location\"")!=std::string::npos){
                            auto v = parseArray1D<double>(lf, L);
                            if(v.size()==3) pos = Vec3(v[0],v[1],v[2]);
                        } 
                        else if(L.find("\"radiant_intensity\"")!=std::string::npos){
                            size_t c=L.find(':'); 
                            if(c!=std::string::npos){
                                std::string s=trim(L.substr(c+1)); 
                                if(!s.empty()&&s.back()==',') s.pop_back();
                                try{ I=(float)std::stod(s);}catch(...){}
                            }
                        } 
                        else if(L.find("\"radius\"")!=std::string::npos){
                            size_t c=L.find(':'); 
                            if(c!=std::string::npos){
                                std::string s=trim(L.substr(c+1));
                                if(!s.empty()&&s.back()==',') s.pop_back();
                                try{ rad=(float)std::stod(s);}catch(...){}
                            }
                        }

                        if(depth==0) break;
                    }

                    lights.push_back({pos, I, rad});
                }
                if(L.find("]")!=std::string::npos) lsec = LNONE;
            }
        }
    }
    std::cout << "Loaded " << lights.size() << " point lights.\n";
    for (const auto& L : lights) {
        std::cout << "  Light at (" << L.position.x << "," << L.position.y << "," << L.position.z
                  << ") I=" << L.intensity << " radius=" << L.radius << "\n";
    }

    // ---------- Load a single texture (no filenames from Blender) ----------
    Texture tex;
    if (!tex.loadPPM("../Textures/tex.ppm")) {
        std::cerr << "Warning: could not load ../Textures/tex1.ppm, using flat colors only.\n";
    }

    // ---------- BVH ----------
    std::vector<const Shape*> shapePtrs;
    shapePtrs.reserve(shapes.size());
    for (auto& p : shapes) shapePtrs.push_back(p.get());

    BVH bvh;
    auto tBuild0 = std::chrono::high_resolution_clock::now();
    bvh.build(shapePtrs);
    auto tBuild1 = std::chrono::high_resolution_clock::now();
    double build_ms = std::chrono::duration<double, std::milli>(tBuild1 - tBuild0).count();
    std::cout << "BVH build: " << build_ms << " ms\n";


    // ---------------- Auto-focus: cast a ray through the center pixel ----------------
    float focus_dist = 3.0f;   // fallback value

    {
        float cx = cam.film_width()  * 0.5f;
        float cy = cam.film_height() * 0.5f;

        float ox, oy, oz, dx, dy, dz;
        cam.pixel_to_ray(cx, cy, ox, oy, oz, dx, dy, dz);

        Ray centerRay{ Vec3(ox,oy,oz), normalize(Vec3(dx,dy,dz)) };
        centerRay.time = 0.5f * (g_shutter_open + g_shutter_close);

        Hit h;
        bool ok = bvh.intersect(centerRay, h);
        if (ok) {
            focus_dist = h.t;     // <-- auto-focus!
            std::cout << "Auto-focus distance = " << focus_dist << "\n";
        } else {
            std::cout << "Auto-focus failed (no hit), using fallback\n";
        }
    }


    // ---------- Render (Whitted + AA) ----------
    int W = cam.film_width() > 0 ? cam.film_width() : 640;
    int H = cam.film_height() > 0 ? cam.film_height() : 480;
    std::vector<Vec3> fb(W*H);

    auto render_and_time = [&](bool useBVH, int& hitCountOut) -> double {
        hitCountOut = 0;
        auto t0 = std::chrono::high_resolution_clock::now();

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {

                Vec3 accum(0,0,0);

                if (!g_enable_aa) {
                    // ----------------- NO AA: 1 sample per pixel -----------------
                    float px = x + 0.5f;
                    float py = y + 0.5f;

                    float ox,oy,oz,dx,dy,dz;
                    cam.pixel_to_ray(px,py,ox,oy,oz,dx,dy,dz);
                    Vec3 origin(ox,oy,oz);
                    Vec3 dir    = normalize(Vec3(dx,dy,dz));

                    float rayTime;
                    if (g_enable_motion_blur) {
                        float shutterStart = g_shutter_open;
                        float shutterEnd   = g_shutter_close;
                        rayTime = shutterStart + rand01() * (shutterEnd - shutterStart);
                    } else {
                        rayTime = 0.5f * (g_shutter_open + g_shutter_close);
                    }

                    if (g_enable_dof && g_lens_radius > 0.0f) {
                        Vec3 focusPoint = origin + dir * g_focus_dist;

                        float lx, ly;
                        random_in_unit_disk(lx, ly);
                        lx *= g_lens_radius;
                        ly *= g_lens_radius;

                        Vec3 lensOffset = camRight * lx + camUp * ly;
                        Vec3 newOrigin  = origin + lensOffset;
                        Vec3 newDir     = normalize(focusPoint - newOrigin);

                        origin = newOrigin;
                        dir    = newDir;
                    }

                    Ray ray;
                    ray.origin = origin;
                    ray.dir    = dir;
                    ray.time   = rayTime;

                    Hit hcount;
                    bool anyHit = useBVH ? bvh.intersect(ray, hcount)
                                        : ([&]{
                                                for (const auto& s : shapes) {
                                                    Hit h; 
                                                    if (s->intersect(ray, h) && h.t < hcount.t) { 
                                                        hcount = h; 
                                                        return true; 
                                                    }
                                                }
                                                return false;
                                            })();
                    if (anyHit) hitCountOut++;

                    Vec3 col = useBVH ? traceRay(ray, shapes, &bvh, matOf, lights,
                                                tex.valid() ? &tex : nullptr, 0)
                                    : traceRay(ray, shapes, nullptr, matOf, lights,
                                                tex.valid() ? &tex : nullptr, 0);
                    accum = col;
                } else {
                    // ----------------- AA ON: grid×grid samples per pixel -----------------
                    const int grid = 2;                 // <--- 4x4 = 16 samples
                    const int spp  = grid * grid;
                    const float invSpp = 1.0f / float(spp);

                    for (int sy = 0; sy < grid; ++sy) {
                        for (int sx = 0; sx < grid; ++sx) {
                            float jx = (sx + 0.5f) / float(grid);
                            float jy = (sy + 0.5f) / float(grid);
                            float px = x + jx;
                            float py = y + jy;

                            float ox,oy,oz,dx,dy,dz;
                            cam.pixel_to_ray(px,py,ox,oy,oz,dx,dy,dz);
                            Vec3 origin(ox,oy,oz);
                            Vec3 dir    = normalize(Vec3(dx,dy,dz));

                            float rayTime;
                            if (g_enable_motion_blur) {
                                float shutterStart = g_shutter_open;
                                float shutterEnd   = g_shutter_close;
                                rayTime = shutterStart + rand01() * (shutterEnd - shutterStart);
                            } else {
                                rayTime = 0.5f * (g_shutter_open + g_shutter_close);
                            }

                            if (g_enable_dof && g_lens_radius > 0.0f) {
                                Vec3 focusPoint = origin + dir * g_focus_dist;

                                float lx, ly;
                                random_in_unit_disk(lx, ly);
                                lx *= g_lens_radius;
                                ly *= g_lens_radius;

                                Vec3 lensOffset = camRight * lx + camUp * ly;
                                Vec3 newOrigin  = origin + lensOffset;
                                Vec3 newDir     = normalize(focusPoint - newOrigin);

                                origin = newOrigin;
                                dir    = newDir;
                            }

                            Ray ray;
                            ray.origin = origin;
                            ray.dir    = dir;
                            ray.time   = rayTime;

                            Hit hcount;
                            bool anyHit = useBVH ? bvh.intersect(ray, hcount)
                                                : ([&]{
                                                        for (const auto& s : shapes) {
                                                            Hit h; 
                                                            if (s->intersect(ray,h) && h.t < hcount.t){ 
                                                                hcount = h; 
                                                                return true; 
                                                            }
                                                        }
                                                        return false;
                                                    })();
                            if (anyHit) hitCountOut++;

                            Vec3 col = useBVH ? traceRay(ray, shapes, &bvh, matOf, lights,
                                                        tex.valid() ? &tex : nullptr, 0)
                                            : traceRay(ray, shapes, nullptr, matOf, lights,
                                                        tex.valid() ? &tex : nullptr, 0);
                            accum += col;
                        }
                    }

                    accum = accum * invSpp;
                }

                fb[y*W + x] = accum;
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double,std::milli>(t1 - t0).count();
    };



    int hits_bvh = 0;
    double ms_bvh = render_and_time(true, hits_bvh);
    std::cout << "BVH:         " << ms_bvh        << " ms, hits=" << hits_bvh << "\n";

    // ---------- Write output ----------
    std::ofstream out("../Output/output.ppm");
    out << "P3\n" << W << " " << H << "\n255\n";
    auto to8=[](float v){v=std::max(0.f,std::min(1.f,v));return(int)std::round(v*255.f);};
    for(int y=0;y<H;++y){
        for(int x=0;x<W;++x){
            Vec3 c=fb[y*W+x];
            out << to8(c.x) << " " << to8(c.y) << " " << to8(c.z) << " ";
        }
        out << "\n";
    }
    out.close();
    std::cout << "Wrote ../Output/output.ppm\n";
    return 0;
}
