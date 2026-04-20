# ── Project settings ──────────────────────────────────────────
TARGET   := bin/drone_sim
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -O2 -I/usr/include/opencv4

# ── Sources ───────────────────────────────────────────────────
SRC_DIR  := src
OBJ_DIR  := obj
SRCS  := $(shell find $(SRC_DIR) -name "*.cpp")
OBJS  := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))

# ── Libraries ─────────────────────────────────────────────────
OPENCV_FLAGS := $(shell pkg-config --cflags --libs opencv4)
GLFW_FLAGS   := $(shell pkg-config --cflags --libs glfw3)
GL_FLAGS     := -lGL -lGLU

LIBS := $(OPENCV_FLAGS) $(GLFW_FLAGS) $(GL_FLAGS)

# ── Rules ─────────────────────────────────────────────────────
all: $(TARGET)

$(TARGET): $(OBJS)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -Iinclude -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) bin

.PHONY: all clean