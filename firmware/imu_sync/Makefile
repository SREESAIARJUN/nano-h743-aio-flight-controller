# Nano H743 AIO - dual ICM-42688-P synchronisation reference (Revision 11.0)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
            -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = imu_sync.cpp test_imu_sync.cpp

.PHONY: test clean
test: test_imu_sync
	./test_imu_sync

test_imu_sync: $(SRC) imu_sync.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f test_imu_sync
