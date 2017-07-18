# pcsrt1 specific Makefile

EIGEN_DIR=/opt/eigen-5a0156e40feb
BOOST_DIR=/opt/boost_1_64_0
CXXFLAGS=-std=c++11 -O3 -I $(EIGEN_DIR) -I $(BOOST_DIR)

all: mst++

mst++: main.cpp MST.hpp
	$(CXX) $(CXXFLAGS) $< -o mst++

clean:
	$(RM) mst++
