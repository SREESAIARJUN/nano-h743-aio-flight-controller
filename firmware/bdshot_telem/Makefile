# Nano H743 AIO - bidirectional DShot telemetry reference (Revision 10.0)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
            -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = bdshot_telem.cpp test_bdshot_telem.cpp

.PHONY: test clean
test: test_bdshot_telem
	./test_bdshot_telem

test_bdshot_telem: $(SRC) bdshot_telem.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f test_bdshot_telem
