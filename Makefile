CXXSRC = $(shell find ./back-end/ -name "*.cpp")
CXXSRC += $(shell find ./front-end/ -name "*.cpp")
CXXSRC += $(shell find ./diff/ -name "*.cpp")
CXXSRC += $(shell find ./memory/ -name "*.cpp")
# CXXSRC += ./rv_simu_mmu.cpp
CXXSRC += ./rv_simu_mmu_v2.cpp # cpp file with main function
CXXINCLUDE = -I./include/
CXXINCLUDE += -I./back-end/include/
CXXINCLUDE += -I./back-end/EXU/include/
CXXINCLUDE += -I./back-end/tools/include/
CXXINCLUDE += -I./diff/include/
CXXINCLUDE += -I./front-end/
CXXINCLUDE += -I./memory/include/

MEM_DIR=./baremetal
IMG=./baremetal/memory

default: $(CXXSRC) 
	g++ $(CXXINCLUDE) $(CXXSRC) -O3 -o build/rv_simu_mmu

run: 
# 	./build/rv_simu_mmu $(IMG)
	./build/rv_simu_mmu $(IMG)

clean:
	rm -f a.out
	rm -rf ./baremetal/memory
	rm -rf ./baremetal/test.code

gdb:
	g++ $(CXXINCLUDE) $(CXXSRC) -g
	gdb --args ./build/rv_simu_mmu $(IMG)

.PHONY: all clean mem run

