#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

class Image {
public:
    // Constructor to read image from file
    Image(const std::string& filename) {
        read_image(filename);
    }

    // Read P3 PPM image
    void read_image(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << "\n";
            return;
        }

        std::string header;
        file >> header;
        if (header != "P3") {
            std::cerr << "Error: Only ASCII PPM (P3) format supported.\n";
            return;
        }

        // Skip comments
        char ch;
        file >> std::ws;
        while (file.peek() == '#') {
            std::string comment;
            std::getline(file, comment);
        }

        file >> width >> height;
        file >> max_color_value;

        pixels.resize(width * height * 3);

        for (int i = 0; i < width * height * 3; ++i) {
            int value;
            file >> value;
            pixels[i] = static_cast<unsigned char>(value);
        }

        file.close();
    }

    // Write image to file
    void write_image(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file for write: " << filename << "\n";
            return;
        }

        file << "P3\n";
        file << width << " " << height << "\n";
        file << max_color_value << "\n";

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = (y * width + x) * 3;
                file << static_cast<int>(pixels[idx]) << " "
                     << static_cast<int>(pixels[idx + 1]) << " "
                     << static_cast<int>(pixels[idx + 2]) << " ";
            }
            file << "\n";
        }

        file.close();
    }

    // Get pixel color
    std::vector<unsigned char> get_pixel(int x, int y) const {
        std::vector<unsigned char> color(3);
        if (x < 0 || x >= width || y < 0 || y >= height) {
            std::cerr << "Pixel out of bounds.\n";
            return {0, 0, 0};
        }
        int idx = (y * width + x) * 3;
        color[0] = pixels[idx];
        color[1] = pixels[idx + 1];
        color[2] = pixels[idx + 2];
        return color;
    }

    // Set pixel color
    void set_pixel(int x, int y, const std::vector<unsigned char>& color) {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            std::cerr << "Pixel out of bounds.\n";
            return;
        }
        int idx = (y * width + x) * 3;
        pixels[idx] = color[0];
        pixels[idx + 1] = color[1];
        pixels[idx + 2] = color[2];
    }

private:
    int width = 0, height = 0, max_color_value = 255;
    std::vector<unsigned char> pixels;
};

// int main() {
//     Image image("../Textures/input.ppm");

//     // Get pixel
//     auto pixel = image.get_pixel(1, 1);

//     std::cout << "Pixel (1,1): "
//               << (int)pixel[0] << ", "
//               << (int)pixel[1] << ", "
//               << (int)pixel[2] << "\n";

//     // Set pixel to red
//     image.set_pixel(1, 1, {255, 0, 0});

//     // Write new image
//     image.write_image("../Output/output.ppm");

//     std::cout << "Output written to output.ppm\n";
//     return 0;
// }
