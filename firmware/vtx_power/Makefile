# Nano H743 AIO - digital VTX 9 V / 12 V BEC supervisor reference (Revision 13.0)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
            -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = vtx_power.cpp test_vtx_power.cpp

.PHONY: test clean
test: test_vtx_power
	./test_vtx_power

test_vtx_power: $(SRC) vtx_power.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f test_vtx_power
