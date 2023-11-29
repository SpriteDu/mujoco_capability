# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I../include  -L../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib

all:
	$(CXX) $(COMMON) -std=c++17 -c lodepng.cpp
	$(CXX) $(COMMON) -std=c++17 -c simulate.cc
	$(CC)  $(COMMON) -std=c11   -c uitools.c
	$(CXX) $(COMMON) -std=c++17 -c glfw_adapter.cc
	$(CXX) $(COMMON) -std=c++17 -c glfw_dispatch.cc
	$(CXX) $(COMMON) -std=c++17 -c platform_ui_adapter.cc
	
	$(CXX) $(COMMON) -std=c++17 -c ../include/tool_kits.cpp
	$(CXX) $(COMMON) -std=c++17 main.cc simulate.o glfw_adapter.o glfw_dispatch.o  platform_ui_adapter.o uitools.o lodepng.o -lmujoco -lglfw  tool_kits.o -o mujoco_control
	
	
	rm uitools.o simulate.o glfw_adapter.o glfw_dispatch.o platform_ui_adapter.o lodepng.o


# -I../include/mujoco -I../include  adapter.o franka_ik_He.o
# $(CXX) $(COMMON) -std=c++17 -c ../include/franka_ik_He.hpp