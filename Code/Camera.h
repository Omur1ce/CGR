#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

class Camera {
public:
    Camera() :
        position_x(0), position_y(0), position_z(0),
        forward_x(0), forward_y(0), forward_z(-1),
        // Default Blender world-up is Z+
        up_x(0), up_y(0), up_z(1),
        focal_length(35.0),               // default mm
        sensor_width(36.0), sensor_height(24.0),
        film_resolution_x(640), film_resolution_y(480) {}

    // ---------- Read camera data from Blender JSON ----------
    void read_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: could not open " << filename << "\n";
            return;
        }

        std::string line;
        bool found_camera_section = false;

        while (std::getline(file, line)) {
            line = trim(line);
            if (line.find("\"cameras\"") != std::string::npos) {
                // Skip until first camera object '{'
                while (std::getline(file, line) && line.find("{") == std::string::npos);
                parse_camera(file);
                found_camera_section = true;
                break;
            }
        }
        file.close();

        if (!found_camera_section)
            std::cerr << "No camera section found in file!\n";
    }

    // ---------- Convert pixel coordinates to world-space ray ----------
    void pixel_to_ray(float px, float py,
                      float& ox, float& oy, float& oz,
                      float& dx, float& dy, float& dz) const {

        if (film_resolution_x == 0 || film_resolution_y == 0 || focal_length == 0) {
            std::cerr << "Error: invalid camera parameters!\n";
            ox = oy = oz = dx = dy = dz = 0; return;
        }

        // Normalize forward
        double fx = forward_x, fy = forward_y, fz = forward_z;
        double fLen = std::sqrt(fx*fx + fy*fy + fz*fz);
        if (fLen < 1e-6) { fx = 0; fy = 0; fz = -1; fLen = 1; }
        fx /= fLen; fy /= fLen; fz /= fLen;

        // Use exported up_vector if present; default to Blender Z-up (0,0,1)
        double upx = up_x, upy = up_y, upz = up_z;
        // If nearly collinear with forward, choose a safe fallback (X+)
        double dotfu = fx*upx + fy*upy + fz*upz;
        if (std::fabs(dotfu) > 0.999) { upx = 1; upy = 0; upz = 0; }

        // right = normalize(cross(forward, up))
        double rx = fy*upz - fz*upy;
        double ry = fz*upx - fx*upz;
        double rz = fx*upy - fy*upx;
        double rLen = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (rLen < 1e-12) { rx = 1; ry = 0; rz = 0; rLen = 1; } // ultra-defensive
        rx /= rLen; ry /= rLen; rz /= rLen;

        // true up = cross(right, forward)
        double tux = ry*fz - rz*fy;
        double tuy = rz*fx - rx*fz;
        double tuz = rx*fy - ry*fx;

        // Normalized device coordinates [-1,1]
        double ndc_x = ((px + 0.5) / (double)film_resolution_x) * 2.0 - 1.0;
        double ndc_y = 1.0 - ((py + 0.5) / (double)film_resolution_y) * 2.0;

        // Convert to sensor plane mm
        // (This matches a symmetric pinhole with physical sensor size.)
        double sx = (sensor_width  * 0.5) * ndc_x;
        double sy = (sensor_height * 0.5) * ndc_y;

        // Direction in world space: f * focal + r * sx + up * sy
        double ddx = fx * focal_length + rx * sx + tux * sy;
        double ddy = fy * focal_length + ry * sx + tuy * sy;
        double ddz = fz * focal_length + rz * sx + tuz * sy;
        double dLen = std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
        ddx/=dLen; ddy/=dLen; ddz/=dLen;

        // Output
        ox = position_x; oy = position_y; oz = position_z;
        dx = ddx; dy = ddy; dz = ddz;
    }

    // ---------- Getters ----------
    int film_width()  const { return film_resolution_x; }
    int film_height() const { return film_resolution_y; }

    double pos_x() const { return position_x; }
    double pos_y() const { return position_y; }
    double pos_z() const { return position_z; }

    double fwd_x() const { return forward_x; }
    double fwd_y() const { return forward_y; }
    double fwd_z() const { return forward_z; }

    double up_vec_x() const { return up_x; }
    double up_vec_y() const { return up_y; }
    double up_vec_z() const { return up_z; }

    double focal_mm() const { return focal_length; }
    double sensor_w_mm() const { return sensor_width; }
    double sensor_h_mm() const { return sensor_height; }

    void print_info() const {
        std::cout << "Camera Info:\n";
        std::cout << "  Position: (" << position_x << ", " << position_y << ", " << position_z << ")\n";
        std::cout << "  Forward:  (" << forward_x << ", " << forward_y << ", " << forward_z << ")\n";
        std::cout << "  Up:       (" << up_x << ", " << up_y << ", " << up_z << ")\n";
        std::cout << "  Focal Length: " << focal_length << "\n";
        std::cout << "  Sensor: " << sensor_width << " x " << sensor_height << "\n";
        std::cout << "  Film Resolution: " << film_resolution_x << " x " << film_resolution_y << "\n";
    }

private:
    double position_x, position_y, position_z;
    double forward_x, forward_y, forward_z;
    double up_x, up_y, up_z;          // NEW: store exported up vector (world-space)
    double focal_length;
    double sensor_width, sensor_height;
    int film_resolution_x, film_resolution_y;

    // ---- Helpers ----
    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\n\r");
        size_t end = s.find_last_not_of(" \t\n\r");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

    template<typename T>
    std::vector<T> parse_array(std::ifstream& file, std::string line) {
        std::vector<T> result;
        size_t start = line.find('[');
        if (start != std::string::npos) line = line.substr(start + 1);
        while (true) {
            size_t end = line.find(']');
            std::string content = (end != std::string::npos) ? line.substr(0, end) : line;
            std::stringstream ss(content);
            std::string item;
            while (std::getline(ss, item, ',')) {
                item = trim(item);
                if (!item.empty()) {
                    try { result.push_back(static_cast<T>(std::stod(item))); }
                    catch (...) {}
                }
            }
            if (end != std::string::npos) break;
            if (!std::getline(file, line)) break;
            line = trim(line);
        }
        return result;
    }

    void parse_camera(std::ifstream& file) {
        std::string line;
        while (std::getline(file, line)) {
            line = trim(line);

            if (line.find("\"location\"") != std::string::npos) {
                auto loc = parse_array<double>(file, line);
                if (loc.size() == 3) { position_x=loc[0]; position_y=loc[1]; position_z=loc[2]; }
            }
            else if (line.find("\"gaze_vector\"") != std::string::npos) {
                auto gaze = parse_array<double>(file, line);
                if (gaze.size() == 3) { forward_x=gaze[0]; forward_y=gaze[1]; forward_z=gaze[2]; }
            }
            else if (line.find("\"up_vector\"") != std::string::npos) {
                auto up = parse_array<double>(file, line);
                if (up.size() == 3) { up_x=up[0]; up_y=up[1]; up_z=up[2]; }
            }
            else if (line.find("\"focal_length\"") != std::string::npos) {
                focal_length = std::stod(trim(line.substr(line.find(':') + 1)));
            }
            else if (line.find("\"sensor_width\"") != std::string::npos) {
                sensor_width = std::stod(trim(line.substr(line.find(':') + 1)));
            }
            else if (line.find("\"sensor_height\"") != std::string::npos) {
                sensor_height = std::stod(trim(line.substr(line.find(':') + 1)));
            }
            else if (line.find("\"film_resolution\"") != std::string::npos) {
                auto res = parse_array<int>(file, line);
                if (res.size() == 2) { film_resolution_x=res[0]; film_resolution_y=res[1]; }
            }
            else if (line.find("}") != std::string::npos) break;
        }
    }
};
