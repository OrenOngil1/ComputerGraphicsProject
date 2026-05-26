# ── Project settings ──────────────────────────────────────────
TARGET   := bin/drone_sim
CXX      := g++
CC       := gcc
CXXFLAGS := -std=c++17 -Wall -O2 -DGLFW_INCLUDE_NONE -I/usr/include/opencv4 -Iinclude -Isrc/engine
CFLAGS   := -Wall -O2 -Iinclude

# ── Sources ───────────────────────────────────────────────────
SRC_DIR  := src
OBJ_DIR  := obj
CPP_SRCS := $(shell find $(SRC_DIR) -name "*.cpp")
C_SRCS   := $(SRC_DIR)/glad.c
OBJS     := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(CPP_SRCS)) \
            $(patsubst $(SRC_DIR)/%.c,   $(OBJ_DIR)/%.o, $(C_SRCS))

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

.PHONY: all clean
