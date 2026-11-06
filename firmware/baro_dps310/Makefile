# Nano H743 AIO - DPS310 robust I2C driver reference (Revision 12.0)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
            -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = dps310_robust.cpp test_dps310_robust.cpp

.PHONY: test clean
test: test_dps310_robust
	./test_dps310_robust

test_dps310_robust: $(SRC) dps310_robust.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f test_dps310_robust
