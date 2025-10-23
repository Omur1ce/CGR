#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

#include "geom.h"
#include "intersect.h"
#include "camera.h"

struct Scene {
    Camera cam;
    std::vector<std::unique_ptr<Shape>> shapes;
};

namespace sl_detail {
    inline std::string trim(const std::string& s){
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        return (a==std::string::npos)? "" : s.substr(a, b-a+1);
    }
    template<typename T=float>
    inline std::vector<T> parseArray1D(std::ifstream& file, std::string line){
        std::vector<T> out;
        size_t s = line.find('[');
        if (s != std::string::npos) line = line.substr(s+1);
        while (true){
            size_t e = line.find(']');
            std::string chunk = (e!=std::string::npos)? line.substr(0,e) : line;
            std::stringstream ss(chunk);
            std::string it;
            while (std::getline(ss, it, ',')){
                it = trim(it);
                if (!it.empty()) out.push_back((T)std::stod(it));
            }
            if (e!=std::string::npos) break;
            if (!std::getline(file, line)) break;
            line = trim(line);
        }
        return out;
    }
}

class SceneLoader {
public:
    static bool load(const std::string& path, Scene& scene){
        // Let Camera parse itself
        scene.cam.read_from_file(path);

        std::ifstream file(path);
        if (!file.is_open()){
            std::cerr << "Error: could not open " << path << "\n";
            return false;
        }

        std::string line;
        enum Section { None, Cubes, Planes, Spheres } section = None;

        while (std::getline(file, line)){
            line = sl_detail::trim(line);
            if (line.empty()) continue;

            // Section detection (matches your JSON keys)
            if (line.find("\"cubes\"")   != std::string::npos) { section = Cubes;  continue; }
            if (line.find("\"planes\"")  != std::string::npos) { section = Planes; continue; }
            if (line.find("\"spheres\"") != std::string::npos) { section = Spheres;continue; }

            if (line.find('{') != std::string::npos){
                // parse object in current section
                if (section == Cubes)  parseCube(file, scene);
                if (section == Planes) parsePlane(file, scene);
                if (section == Spheres)parseSphere(file, scene);
            }
            if (line.find(']') != std::string::npos){
                section = None;
            }
        }
        file.close();

        std::cout << "Loaded " << scene.shapes.size() << " shapes.\n";
        return true;
    }

private:
    static void parseCube(std::ifstream& file, Scene& scene){
        Vec3 T(0,0,0), R(0,0,0);
        float S = 1.0f;
        std::string line;
        while (std::getline(file, line)){
            line = sl_detail::trim(line);
            if (line.find("\"translation\"") != std::string::npos){
                auto v = sl_detail::parseArray1D<double>(file, line);
                if (v.size()==3) T = Vec3((float)v[0], (float)v[1], (float)v[2]);
            } else if (line.find("\"rotation\"") != std::string::npos){
                auto v = sl_detail::parseArray1D<double>(file, line);
                if (v.size()==3) R = Vec3((float)v[0], (float)v[1], (float)v[2]); // assume radians (Blender)
            } else if (line.find("\"scale\"") != std::string::npos){
                auto colon = line.find(':');
                if (colon != std::string::npos)
                    S = (float)std::stod(sl_detail::trim(line.substr(colon+1)));
            } else if (line.find('}') != std::string::npos){
                break;
            }
        }
        scene.shapes.emplace_back(std::make_unique<Cube>(T, R, S));
    }

    static void parsePlane(std::ifstream& file, Scene& scene){
        // corners array of 4 triples
        Vec3 c[4];
        int got=0;
        std::string line;
        while (std::getline(file, line)){
            line = sl_detail::trim(line);
            if (line.find('[') != std::string::npos){
                auto v = sl_detail::parseArray1D<double>(file, line);
                if (v.size()==3 && got<4){
                    c[got++] = Vec3((float)v[0], (float)v[1], (float)v[2]);
                }
            } else if (line.find('}') != std::string::npos){
                break;
            }
        }
        if (got==4){
            scene.shapes.emplace_back(std::make_unique<PlaneQuad>(c[0],c[1],c[2],c[3]));
        }
    }

    static void parseSphere(std::ifstream& file, Scene& scene){
        Vec3 C(0,0,0);
        float R = 1.0f;
        std::string line;
        while (std::getline(file, line)){
            line = sl_detail::trim(line);
            if (line.find("\"location\"") != std::string::npos){
                auto v = sl_detail::parseArray1D<double>(file, line);
                if (v.size()==3) C = Vec3((float)v[0], (float)v[1], (float)v[2]);
            } else if (line.find("\"radius\"") != std::string::npos){
                auto colon = line.find(':');
                if (colon != std::string::npos)
                    R = (float)std::stod(sl_detail::trim(line.substr(colon+1)));
            } else if (line.find('}') != std::string::npos){
                break;
            }
        }
        scene.shapes.emplace_back(std::make_unique<Sphere>(C, R));
    }
};
