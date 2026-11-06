# Nano H743 AIO - AT7456E analog OSD driver reference (Revision 13.0)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
            -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = osd_at7456e.cpp test_osd_at7456e.cpp

.PHONY: test clean
test: test_osd_at7456e
	./test_osd_at7456e

test_osd_at7456e: $(SRC) osd_at7456e.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f test_osd_at7456e
