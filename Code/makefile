# Makefile for CGR_CW1 Whitted Ray Tracer (placed inside Code/)

CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra

# Source and executable
SRC := Raytracer.cpp
EXE := raytracer

# Header dependencies (adjust if you add more headers)
HDR := geom.h intersect.h camera.h bvh.h texture.h

# Default build rule
all: $(EXE)

$(EXE): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(EXE)

# Build and run immediately
run: $(EXE)
	./$(EXE)

# Clean executable
clean:
	rm -f $(EXE)
