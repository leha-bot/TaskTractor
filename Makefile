CCFLAGS = -Wall -Wpedantic -Werror -std=c++11
INCLUDE = include
SOURCES = $(wildcard src/*.cpp)
SOURCE_DIRS = src Linux
simtractor:
	$(CC) $(CCFLAGS) -I$(INCLUDE) $(SOURCES) -lstdc++ -lm -obin/$@

#.cpp.o
# $< is the automatic variable. Takes 1st name from deps list.
#	$(CC) -c $<
