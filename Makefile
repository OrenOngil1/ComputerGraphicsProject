# ── Project settings ──────────────────────────────────────────
TARGET   := bin/drone_sim
CXX      := g++
CC       := gcc
# -MMD -MP make the compiler emit a .d file beside each .o listing the headers
# that .o depends on (with phony targets for each header so deleting one doesn't
# break the build). `-include`d below, this gives correct incremental rebuilds
# when a header changes -- no more stale objects / mandatory `make clean`.
CXXFLAGS := -std=c++17 -Wall -O2 -MMD -MP -DGLFW_INCLUDE_NONE -I/usr/include/opencv4 -Iinclude -Isrc/engine
CFLAGS   := -Wall -O2 -MMD -MP -Iinclude

# ── Sources ───────────────────────────────────────────────────
SRC_DIR  := src
OBJ_DIR  := obj
CPP_SRCS := $(shell find $(SRC_DIR) -name "*.cpp")
C_SRCS   := $(SRC_DIR)/glad.c
OBJS     := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(CPP_SRCS)) \
            $(patsubst $(SRC_DIR)/%.c,   $(OBJ_DIR)/%.o, $(C_SRCS))
DEPS     := $(OBJS:.o=.d)   # auto-generated header dependency lists (-MMD)

# ── Libraries ─────────────────────────────────────────────────
OPENCV_FLAGS := $(shell pkg-config --cflags --libs opencv4)
GLFW_FLAGS   := $(shell pkg-config --cflags --libs glfw3)
GL_FLAGS     := -lGL -ldl

LIBS := $(OPENCV_FLAGS) $(GLFW_FLAGS) $(GL_FLAGS)

# ── Rules ─────────────────────────────────────────────────────
all: $(TARGET)

$(TARGET): $(OBJS)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) bin

# ── Headless checks ───────────────────────────────────────────
# Sanity-checks the GL-free math (PnP solvers, blob centroids, terrain
# normals) on synthetic inputs -- no window or GPU needed. The test binary
# links the few object files it exercises rather than the whole app (main.cpp
# defines main, and the GL-dependent objects would drag in a context).
CHECK_TARGET := bin/headless_checks

check: $(CHECK_TARGET)
	./$(CHECK_TARGET)

$(CHECK_TARGET): tests/headless_checks.cpp \
                 $(OBJ_DIR)/vision/Pnp.o \
                 $(OBJ_DIR)/vision/TrackerDetection.o \
                 $(OBJ_DIR)/loader/TerrainLoader.o
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBS)

# Pull in the per-object header dependencies emitted by -MMD (silent on the
# first build, before any .d exists).
-include $(DEPS)

.PHONY: all clean check
