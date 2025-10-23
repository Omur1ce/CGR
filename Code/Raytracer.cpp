#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <memory>
#include "geom.h"
#include "intersect.h"
#include "camera.h"

// Simple safe helper for JSON number extraction
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

int main(){
    // ---------- Load camera ----------
    Camera cam;
    cam.read_from_file("../Blend/export.json");
    cam.print_info();

    // ---------- Load shapes from your JSON ----------
    std::vector<std::unique_ptr<Shape>> shapes;
    std::ifstream file("../Blend/export.json");
    if(!file.is_open()){ std::cerr<<"cannot open scene\n"; return 1; }

    std::string line;
    enum {NONE, CUBES, PLANES, SPHERES} section = NONE;
    while(std::getline(file,line)){
        line=trim(line);
        if(line.find("\"cubes\"")!=std::string::npos){ section=CUBES; continue; }
        if(line.find("\"planes\"")!=std::string::npos){ section=PLANES; continue; }
        if(line.find("\"spheres\"")!=std::string::npos){ section=SPHERES; continue; }

        if(line.find("{")!=std::string::npos){
            if(section==CUBES){
                Vec3 T(0,0,0),R(0,0,0); float S=1;
                while(std::getline(file,line)){
                    line=trim(line);
                    if(line.find("\"translation\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) T=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"rotation\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) R=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"scale\"")!=std::string::npos){
                        size_t c=line.find(':'); if(c!=std::string::npos){
                            std::string v=trim(line.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
                            try{ S=(float)std::stod(v);}catch(...){}
                        }
                    } else if(line.find("}")!=std::string::npos) break;
                }
                shapes.emplace_back(std::make_unique<Cube>(T,R,S));
            }
            else if(section==PLANES){
                Vec3 c[4]; int got=0;
                while(std::getline(file,line)){
                    line=trim(line);
                    if(line.find('[')!=std::string::npos){
                        auto v=parseArray1D<double>(file,line);
                        if(v.size()==3 && got<4) c[got++]=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("}")!=std::string::npos) break;
                }
                if(got==4) shapes.emplace_back(std::make_unique<PlaneQuad>(c[0],c[1],c[2],c[3]));
            }
            else if(section==SPHERES){
                Vec3 C(0,0,0); float r=1;
                while(std::getline(file,line)){
                    line=trim(line);
                    if(line.find("\"location\"")!=std::string::npos){
                        auto v=parseArray1D<double>(file,line); if(v.size()==3) C=Vec3(v[0],v[1],v[2]);
                    } else if(line.find("\"radius\"")!=std::string::npos){
                        size_t c=line.find(':'); if(c!=std::string::npos){
                            std::string v=trim(line.substr(c+1)); if(!v.empty()&&v.back()==',') v.pop_back();
                            try{ r=(float)std::stod(v);}catch(...){}
                        }
                    } else if(line.find("}")!=std::string::npos) break;
                }
                shapes.emplace_back(std::make_unique<Sphere>(C,r));
            }
        }
        if(line.find("]")!=std::string::npos) section=NONE;
    }
    file.close();

    std::cout << "Loaded " << shapes.size() << " shapes.\n";

    // ---------- Render ----------
    int W = cam.film_width() > 0 ? cam.film_width() : 640;
    int H = cam.film_height() > 0 ? cam.film_height() : 480;
    std::vector<Vec3> fb(W*H);

    int hitCount=0;
    for(int y=0;y<H;++y){
        for(int x=0;x<W;++x){
            float ox,oy,oz,dx,dy,dz;
            cam.pixel_to_ray((float)x,(float)y,ox,oy,oz,dx,dy,dz);
            Ray ray{Vec3(ox,oy,oz),normalize(Vec3(dx,dy,dz))};

            Hit best; bool hit=false;
            for(const auto& s:shapes){
                Hit h;
                if(s->intersect(ray,h) && h.t<best.t){ best=h; hit=true; }
            }
            if(hit) hitCount++;
            Vec3 col = hit ? (best.n*0.5f + Vec3(0.5f,0.5f,0.5f))
                           : Vec3(0.2f,0.3f,0.5f);
            fb[y*W+x]=col;
        }
    }

    std::cout << "Hit pixels: " << hitCount << " / " << (W*H) << "\n";

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
