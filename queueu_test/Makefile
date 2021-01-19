TARGET=main
TEST_TARGET=test_pi

CPPFLAGS=-Isrc -Itest_files
CXXFLAGS=-Wall -pedantic -ggdb -O0 $(CPPFLAGS)

SOURCES=$(wildcard src/*.cpp) 

HEADERS=$(wildcard src/*.h) \
        
TEST_SOURCES=$(wildcard test/*.cpp) \
	src/simpleQueue.cpp 
TEST_HEADERS=$(wildcard test/*.h) \
	src/simpleQueue.h 
# LDFLAGS= -lgtest -lgmock -lgmock_main -lpthread

CXX=g++

TEST_FLAGS= -lgtest -lgmock -lgmock_main -lpthread

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES)

$(TEST_TARGET): $(TEST_SOURCES) $(TEST_HEADERS)
	$(CXX) $(CXXFLAGS) $(TEST_SOURCES) -o $@ $(TEST_FLAGS)

clean: 
	@rm -rf $(TARGET) \
	$(TEST_TARGET)

test: $(TEST_TARGET)
	@./$(TEST_TARGET)