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
        focal_length(0), sensor_width(0), sensor_height(0),
        film_resolution_x(0), film_resolution_y(0) {}

    // Reads camera data from JSON file
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
                // Parse the camera
                parse_camera(file);
                found_camera_section = true;
                break;
            }
        }

        file.close();

        if (!found_camera_section) {
            std::cerr << "No camera section found in file!\n";
        }
    }

    // Converts pixel coordinates to a world-space ray
    void pixel_to_ray(float px, float py,
                      float& origin_x, float& origin_y, float& origin_z,
                      float& direction_x, float& direction_y, float& direction_z) const {
        if (film_resolution_x == 0 || film_resolution_y == 0) {
            std::cerr << "Error: film resolution not set!\n";
            direction_x = direction_y = direction_z = 0;
            origin_x = origin_y = origin_z = 0;
            return;
        }

        float sensor_x = (2.0f * (px / film_resolution_x) - 1.0f) * (sensor_width / 2.0f);
        float sensor_y = (1.0f - 2.0f * (py / film_resolution_y)) * (sensor_height / 2.0f);

        // Basic ray direction (approximation)
        direction_x = forward_x + sensor_x / focal_length;
        direction_y = forward_y + sensor_y / focal_length;
        direction_z = forward_z;

        // Normalize
        float norm = std::sqrt(direction_x * direction_x +
                               direction_y * direction_y +
                               direction_z * direction_z);
        if (norm > 0) {
            direction_x /= norm;
            direction_y /= norm;
            direction_z /= norm;
        }

        origin_x = position_x;
        origin_y = position_y;
        origin_z = position_z;
    }

    void print_info() const {
        std::cout << "Camera Info:\n";
        std::cout << "  Position: (" << position_x << ", " << position_y << ", " << position_z << ")\n";
        std::cout << "  Forward:  (" << forward_x << ", " << forward_y << ", " << forward_z << ")\n";
        std::cout << "  Focal Length: " << focal_length << "\n";
        std::cout << "  Sensor: " << sensor_width << " x " << sensor_height << "\n";
        std::cout << "  Film Resolution: " << film_resolution_x << " x " << film_resolution_y << "\n";
    }

private:
    double position_x, position_y, position_z;
    double forward_x, forward_y, forward_z;
    double focal_length;
    double sensor_width, sensor_height;
    int film_resolution_x, film_resolution_y;

    // ------------------- Parser Helpers -------------------
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
            std::string content;
            if (end != std::string::npos) {
                content = line.substr(0, end);
            } else {
                content = line;
            }

            std::stringstream ss(content);
            std::string item;
            while (std::getline(ss, item, ',')) {
                item = trim(item);
                if (!item.empty()) {
                    result.push_back(static_cast<T>(std::stod(item)));
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
                if (loc.size() == 3) {
                    position_x = loc[0];
                    position_y = loc[1];
                    position_z = loc[2];
                }
            }
            else if (line.find("\"gaze_vector\"") != std::string::npos) {
                auto gaze = parse_array<double>(file, line);
                if (gaze.size() == 3) {
                    forward_x = gaze[0];
                    forward_y = gaze[1];
                    forward_z = gaze[2];
                }
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
                if (res.size() == 2) {
                    film_resolution_x = res[0];
                    film_resolution_y = res[1];
                }
            }
            else if (line.find("}") != std::string::npos) {
                break; // End of this camera
            }
        }
    }
};

// ------------------- Example Usage -------------------
// int main() {
//     Camera cam;
//     cam.read_from_file("../Blend/export.json");
//     cam.print_info();

//     float ox, oy, oz, dx, dy, dz;
//     cam.pixel_to_ray(960.0f, 540.0f, ox, oy, oz, dx, dy, dz);

//     std::cout << "\nRay origin: (" << ox << ", " << oy << ", " << oz << ")\n";
//     std::cout << "Ray direction: (" << dx << ", " << dy << ", " << dz << ")\n";

//     std::cout << "\nPress Enter to exit...";
//     std::cin.get();

//     return 0;
// }
