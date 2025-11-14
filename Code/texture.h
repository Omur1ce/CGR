// texture.h
#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cctype>
#include <algorithm>
#include "geom.h"

// Minimal PPM (P3/P6) loader and bilinear sampler (no external libs).
struct Texture {
    int w = 0, h = 0;                 // width, height
    std::vector<unsigned char> rgb;   // size = w*h*3 (RGB 8-bit)

    static inline void skipSpacesAndComments(std::istream& in){
        for(;;){
            int c = in.peek();
            if (c == '#'){ std::string dummy; std::getline(in, dummy); continue; }
            if (std::isspace(c)) { in.get(); continue; }
            break;
        }
    }

    bool loadPPM(const std::string& path){
        w = h = 0; rgb.clear();

        std::ifstream f(path, std::ios::binary);
        if(!f.is_open()) return false;

        std::string magic;
        f >> magic;
        if(magic != "P6" && magic != "P3") return false;

        skipSpacesAndComments(f);
        f >> w; skipSpacesAndComments(f);
        f >> h; skipSpacesAndComments(f);
        int maxv = 255;
        f >> maxv;
        if(maxv <= 0) return false;

        // consume 1 whitespace/newline after header
        f.get();

        rgb.resize(w*h*3);

        if(magic == "P6"){
            // raw binary RGB
            std::vector<unsigned char> buf(w*h*3);
            f.read(reinterpret_cast<char*>(buf.data()), buf.size());
            if(!f) return false;

            if(maxv == 255){
                rgb = std::move(buf);
            }else{
                // scale to 8-bit
                for(size_t i=0;i<buf.size();++i){
                    rgb[i] = static_cast<unsigned char>( (int(buf[i])*255) / maxv );
                }
            }
        }else{
            // P3 ASCII
            for(int i=0;i<w*h;++i){
                int r,g,b; 
                if(!(f >> r >> g >> b)) return false;
                rgb[i*3+0] = static_cast<unsigned char>( std::clamp((r*255)/maxv, 0, 255) );
                rgb[i*3+1] = static_cast<unsigned char>( std::clamp((g*255)/maxv, 0, 255) );
                rgb[i*3+2] = static_cast<unsigned char>( std::clamp((b*255)/maxv, 0, 255) );
            }
        }
        return true;
    }

    // Bilinear sample with optional wrap and optional V-flip (images are commonly top-left origin)
    Vec3 sample(float u, float v, bool wrap=true, bool flipV=true) const {
        if(w<=0 || h<=0) return Vec3(1,1,1);

        if(wrap){
            u = u - std::floor(u);
            v = v - std::floor(v);
        }else{
            u = std::max(0.f, std::min(1.f, u));
            v = std::max(0.f, std::min(1.f, v));
        }

        if(flipV) v = 1.0f - v;

        float x = u * (w - 1);
        float y = v * (h - 1);

        int x0 = (int)std::floor(x);
        int y0 = (int)std::floor(y);
        int x1 = std::min(x0+1, w-1);
        int y1 = std::min(y0+1, h-1);

        float tx = x - x0;
        float ty = y - y0;

        auto at = [&](int X,int Y){
            int idx = (Y*w + X)*3;
            return Vec3(rgb[idx]/255.f, rgb[idx+1]/255.f, rgb[idx+2]/255.f);
        };

        Vec3 c00 = at(x0,y0), c10 = at(x1,y0);
        Vec3 c01 = at(x0,y1), c11 = at(x1,y1);
        Vec3 c0 = c00*(1-tx) + c10*tx;
        Vec3 c1 = c01*(1-tx) + c11*tx;
        return c0*(1-ty) + c1*ty;
    }
};
