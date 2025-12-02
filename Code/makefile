# Makefile for CGR_CW1 Whitted Ray Tracer (placed inside Code/)

CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra

# Source and executable
SRC := Raytracer.cpp
EXE := raytracer

# Header dependencies (adjust if you add more headers)
HDR := geom.h intersect.h camera.h bvh.h texture.h

# ---------- Default build ----------
all: $(EXE)

$(EXE): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(EXE)

# ---------- Run helpers ----------

# Generic run: you can pass extra arguments via:
#   make run ARGS="--no-textures --no-dof"
run: $(EXE)
	./$(EXE) $(ARGS)

# Convenience presets (marker can just do `make run_basic`, etc.)

run_basic: ARGS=
run_basic: run

run_no_textures: ARGS=--no-textures
run_no_textures: run

run_no_dof: ARGS=--no-dof
run_no_dof: run

run_no_soft_shadows: ARGS=--no-soft-shadows
run_no_soft_shadows: run

run_no_glossy: ARGS=--no-glossy
run_no_glossy: run

run_no_motion_blur: ARGS=--no-motion-blur
run_no_motion_blur: run

run_no_aa: ARGS=--no-aa
run_no_aa: run

# You can also combine flags, e.g.:
#   make run_clean ARGS="--no-textures --no-dof --no-soft-shadows --no-glossy --no-motion-blur"
run_clean: ARGS=--no-textures --no-dof --no-soft-shadows --no-glossy --no-motion-blur --no-aa
run_clean: run

# ---------- Clean ----------
clean:
	rm -f $(EXE)
