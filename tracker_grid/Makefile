# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Ofast -pthread -Wall -Wextra
# Add your Boost include path if it's not in the standard location
BOOST_INCLUDE = -I/usr/local/opt/boost/include
# Add your Boost library path if it's not in the standard location
BOOST_LIB = -L/usr/local/opt/boost

# Link against the required Boost libraries (e.g., system, filesystem)
# Replace 'boost_system' with any specific Boost libraries you use
BOOST_FLAGS = -lboost_system

# Program and data files
EXECUTABLE = tracker_grid
SOURCE = tracker_grid.cpp
INPUT_FILE = /Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/log_3.csv

# Build and run targets
.PHONY: all clean run

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(BOOST_INCLUDE) $(BOOST_LIB) -o $(EXECUTABLE) $(SOURCE) $(BOOST_FLAGS)

run: $(EXECUTABLE)
	./$(EXECUTABLE) $(INPUT_FILE)

clean:
	rm -f $(EXECUTABLE)
