#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <cmath>
#include "geom.h"

// Simple PPM texture loader (P3/P6), stored as 8-bit RGB
class Texture {
public:
    Texture() : width(0), height(0) {}

    explicit Texture(const std::string& filename) {
        loadPPM(filename);
    }

    bool loadPPM(const std::string& filename) {
        width = height = 0;
        data.clear();

        std::ifstream f(filename, std::ios::binary);
        if (!f.is_open()) {
            std::cerr << "Texture: cannot open " << filename << "\n";
            return false;
        }

        std::string magic;
        f >> magic;
        if (magic != "P3" && magic != "P6") {
            std::cerr << "Texture: only P3/P6 PPM supported\n";
            return false;
        }

        auto skipComments = [&]() {
            int c;
            while ((c = f.peek()) == '#') {
                std::string dummy;
                std::getline(f, dummy);
            }
        };

        skipComments();
        f >> width >> height;
        skipComments();
        int maxv = 255;
        f >> maxv;
        f.get(); // consume one whitespace after maxv

        if (width <= 0 || height <= 0 || maxv <= 0) {
            std::cerr << "Texture: invalid header\n";
            return false;
        }

        data.resize(width * height * 3);

        auto mapVal = [&](int v) -> unsigned char {
            int mapped = (v * 255) / maxv;
            if (mapped < 0)   mapped = 0;
            if (mapped > 255) mapped = 255;
            return static_cast<unsigned char>(mapped);
        };

        if (magic == "P6") {
            std::vector<unsigned char> buf(width * height * 3);
            f.read(reinterpret_cast<char*>(buf.data()), buf.size());
            if (!f) {
                std::cerr << "Texture: binary PPM truncated\n";
                width = height = 0;
                data.clear();
                return false;
            }

            if (maxv == 255) {
                data = std::move(buf);
            } else {
                for (size_t i = 0; i < buf.size(); ++i) {
                    int v = buf[i];
                    data[i] = mapVal(v);
                }
            }
        } else { // P3 ASCII
            for (int i = 0; i < width * height; ++i) {
                int r, g, b;
                if (!(f >> r >> g >> b)) {
                    std::cerr << "Texture: ASCII PPM truncated\n";
                    width = height = 0;
                    data.clear();
                    return false;
                }
                data[i * 3 + 0] = mapVal(r);
                data[i * 3 + 1] = mapVal(g);
                data[i * 3 + 2] = mapVal(b);
            }
        }

        std::cout << "Loaded texture " << filename
                  << " (" << width << "x" << height << ")\n";
        return true;
    }

    bool valid() const {
        return width > 0 && height > 0 &&
               data.size() == static_cast<size_t>(width * height * 3);
    }

    // Sample with (u,v) ∈ ℝ; we wrap to [0,1) and nearest-neighbour sample.
    Vec3 sample(float u, float v) const {
        if (!valid()) return Vec3(1, 1, 1);

        // wrap u,v to [0,1)
        u -= std::floor(u);
        v -= std::floor(v);

        int x = static_cast<int>(u * (width  - 1) + 0.5f);
        int y = static_cast<int>((1.0f - v) * (height - 1) + 0.5f); // flip v

        if (x < 0) x = 0;
        if (x >= width) x = width - 1;
        if (y < 0) y = 0;
        if (y >= height) y = height - 1;

        int idx = (y * width + x) * 3;
        float r = data[idx + 0] / 255.0f;
        float g = data[idx + 1] / 255.0f;
        float b = data[idx + 2] / 255.0f;
        return Vec3(r, g, b);
    }

private:
    int width, height;
    std::vector<unsigned char> data;
};
