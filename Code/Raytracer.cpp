#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <chrono>
#include <unordered_map>

#include "geom.h"
#include "intersect.h"
#include "camera.h"
#include "bvh.h"

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

// ---------------- Lighting & Materials ----------------
struct PointLight {
    Vec3 position;
    float intensity; // scalar; 1/r^2 falloff applied in shader
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
// We rely on the outer object loop using a brace depth counter,
// so we simply sniff for known keys as lines pass by.
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

// Shade a hit with Blinn-Phong and hard shadows
Vec3 shadeHit(const Hit& hit, const Vec3& viewDir,
              const std::vector<PointLight>& lights,
              const std::unordered_map<const Shape*, Material>& matOf,
              const std::vector<std::unique_ptr<Shape>>& shapes,
              const BVH* bvh)
{
    auto it = matOf.find(hit.shape);
    Material mat = (it!=matOf.end()) ? it->second : Material{};

    Vec3 N = hit.n;
    Vec3 V = viewDir;
    Vec3 color(0,0,0);

    for(const auto& Ls : lights){
        Vec3 Ldir = Ls.position - hit.p;
        float dist2 = dot(Ldir, Ldir);
        float dist  = std::sqrt(std::max(dist2, 1e-12f));
        Ldir = Ldir / dist;

        // Shadow ray toward the light
        Ray shadowRay;
        shadowRay.origin = hit.p + N * kShadowBias;
        shadowRay.dir    = Ldir;
        shadowRay.tmin   = 0.0f;
        shadowRay.tmax   = dist - 1e-3f;

        bool occluded = false;
        if (bvh){
            Hit occ;
            occluded = bvh->intersect(shadowRay, occ);
        } else {
            for(const auto& s: shapes){
                Hit htmp; if(s->intersect(shadowRay, htmp)){ occluded = true; break; }
            }
        }
        if (occluded) continue;

        // Blinn-Phong terms
        float NdotL = std::max(0.f, dot(N, Ldir));
        Vec3 diffuse = mat.kd * NdotL;

        Vec3 H = normalize(Ldir + V);
        float NdotH = std::max(0.f, dot(N, H));
        Vec3 spec = mat.ks * std::pow(NdotH, mat.shininess);

        float atten = Ls.intensity / std::max(1.f, dist2);
        color += (diffuse + spec) * atten;
    }

    return clamp01(color);
}

// Radiance along a ray with recursion for reflections/refractions
Vec3 traceRay(const Ray& r,
              const std::vector<std::unique_ptr<Shape>>& shapes,
              const BVH* bvh,
              const std::unordered_map<const Shape*, Material>& matOf,
              const std::vector<PointLight>& lights,
              int depth)
{
    if (depth > kMaxDepth) return Vec3(0,0,0);

    Hit hit; bool ok=false;
    if (bvh) ok = bvh->intersect(r, hit);
    else {
        for(const auto& s: shapes){
            Hit h; if (s->intersect(r,h) && h.t < hit.t) { hit = h; ok = true; }
        }
    }
    if (!ok) return Vec3(0.2f,0.3f,0.5f); // background

    const Material& mat = (matOf.count(hit.shape)? matOf.at(hit.shape) : Material{});
    Vec3 V = normalize((r.dir) * -1.0f);

    // Direct lighting
    Vec3 Lo = shadeHit(hit, V, lights, matOf, shapes, bvh);

    // Secondary rays
    Vec3 N = hit.n;
    Vec3 accum = Lo;

    // Reflection
    if (mat.reflectivity > 0.0f){
        Vec3 Rdir = reflect(r.dir, N);
        Ray rRef{ hit.p + N * kShadowBias, normalize(Rdir) };
        Vec3 Lr = traceRay(rRef, shapes, bvh, matOf, lights, depth+1);
        accum += Lr * mat.reflectivity;
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

            Ray rRefr{ hit.p - N * kShadowBias, normalize(Tdir) };
            Vec3 Lt = traceRay(rRefr, shapes, bvh, matOf, lights, depth+1);

            // Mix local + transmission (reflection already added above)
            accum = accum * (1.0f - mat.transmissive)
                  + Lt * (mat.transmissive * (1.0f - Fr));
        }
        // else: TIR -> reflection term already present
    }

    return clamp01(accum);
}

int main(){
    // ---------- Load camera ----------
    Camera cam;
    cam.read_from_file("../Blend/export.json");
    cam.print_info();

    // ---------- Load shapes + materials from JSON ----------
    std::vector<std::unique_ptr<Shape>> shapes;
    std::unordered_map<const Shape*, Material> matOf;

    std::ifstream file("../Blend/export.json");
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
                int depth=1;
                while(std::getline(file,line)){
                    line=trim(line);
                    depth += countChar(line,'{');
                    depth -= countChar(line,'}');
                    if(line.find("\"location\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) C=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"scale\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3){ S=Vec3(v[0],v[1],v[2]); haveScale=true; }
                    } else if(line.find("\"radius\"")!=std::string::npos){
                        size_t cpos=line.find(':'); 
                        if(cpos!=std::string::npos){
                            std::string v=trim(line.substr(cpos+1)); if(!v.empty()&&v.back()==',') v.pop_back();
                            try{ r=(float)std::stod(v); haveRadius=true; }catch(...){}
                        }
                    } else if(line.find("\"material\"")!=std::string::npos || line.find("\"kd\"")!=std::string::npos ||
                              line.find("\"ks\"")!=std::string::npos || line.find("\"shininess\"")!=std::string::npos ||
                              line.find("\"reflectivity\"")!=std::string::npos || line.find("\"transmissive\"")!=std::string::npos ||
                              line.find("\"ior\"")!=std::string::npos){
                        maybeParseMaterial(file, line, M);
                    }
                    if(depth==0) break;
                }

                if (haveScale) {
                    shapes.emplace_back(std::make_unique<Sphere>(C, S)); // Ellipsoid
                } else {
                    if(!haveRadius && haveScale) r = std::max(S.x,std::max(S.y,S.z));
                    shapes.emplace_back(std::make_unique<Sphere>(C, r)); // Uniform
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
        std::ifstream lf("../Blend/export.json");
        if(lf.is_open()){
            std::string L; enum {LNONE, LPOINTS} lsec = LNONE;
            while(std::getline(lf,L)){
                L = trim(L);
                if(L.find("\"point_lights\"") != std::string::npos){ lsec = LPOINTS; continue; }
                if(L.find("{")!=std::string::npos && lsec==LPOINTS){
                    Vec3 pos(0,0,0); float I=100.0f;
                    int depth=1;
                    while(std::getline(lf,L)){
                        L = trim(L);
                        depth += countChar(L,'{');
                        depth -= countChar(L,'}');
                        if(L.find("\"location\"")!=std::string::npos){
                            auto v = parseArray1D<double>(lf, L);
                            if(v.size()==3) pos = Vec3(v[0],v[1],v[2]);
                        } else if(L.find("\"radiant_intensity\"")!=std::string::npos){
                            size_t c=L.find(':'); if(c!=std::string::npos){
                                std::string s=trim(L.substr(c+1)); if(!s.empty()&&s.back()==',') s.pop_back();
                                try{ I=(float)std::stod(s);}catch(...){}
                            }
                        }
                        if(depth==0) break;
                    }
                    lights.push_back({pos,I});
                }
                if(L.find("]")!=std::string::npos) lsec = LNONE;
            }
        }
    }
    std::cout << "Loaded " << lights.size() << " point lights.\n";

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

    // ---------- Render (Whitted) ----------
    int W = cam.film_width() > 0 ? cam.film_width() : 640;
    int H = cam.film_height() > 0 ? cam.film_height() : 480;
    std::vector<Vec3> fb(W*H);

    auto render_and_time = [&](bool useBVH, int& hitCountOut) -> double {
        hitCountOut = 0;
        auto t0 = std::chrono::high_resolution_clock::now();

        for(int y=0;y<H;++y){
            for(int x=0;x<W;++x){
                float ox,oy,oz,dx,dy,dz;
                cam.pixel_to_ray((float)x,(float)y,ox,oy,oz,dx,dy,dz);
                Ray ray{Vec3(ox,oy,oz), normalize(Vec3(dx,dy,dz))};

                // Count any hit for stats
                Hit hcount;
                bool anyHit = useBVH ? bvh.intersect(ray, hcount)
                                     : ([&]{
                                            for(const auto& s:shapes){
                                                Hit h; if(s->intersect(ray,h) && h.t < hcount.t){ hcount=h; return true; }
                                            }
                                            return false;
                                        })();
                if (anyHit) hitCountOut++;

                // Whitted color
                Vec3 col = useBVH ? traceRay(ray, shapes, &bvh, matOf, lights, 0)
                                  : traceRay(ray, shapes, nullptr, matOf, lights, 0);
                fb[y*W + x] = col;
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double,std::milli>(t1 - t0).count();
    };

    // Warm-up (optional)
    int tmpHits = 0; render_and_time(true, tmpHits);

    // Measure brute-force
    int hits_bruteforce = 0;
    double ms_bruteforce = render_and_time(false, hits_bruteforce);

    // Measure BVH
    int hits_bvh = 0;
    double ms_bvh = render_and_time(true, hits_bvh);

    std::cout << "Brute force: " << ms_bruteforce << " ms, hits=" << hits_bruteforce << "\n";
    std::cout << "BVH:         " << ms_bvh        << " ms, hits=" << hits_bvh << "\n";
    if (ms_bvh > 0.0) std::cout << "Speedup:     " << (ms_bruteforce / ms_bvh) << "x\n";

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
