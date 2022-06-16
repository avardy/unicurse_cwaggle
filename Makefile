CC=clang++
CFLAGS=-O3 -std=c++17 -g

LDFLAGS=-O3 -lsfml-graphics -lsfml-window -lsfml-system

detected_OS=$(shell sh -c 'uname 2>/dev/null || echo Unknown')
$(info "Detected OS: $(detected_OS)")

ifeq ($(detected_OS),Darwin)
CFLAGS+=-Wno-deprecated-declarations
LDFLAGS+=-framework GLUT -framework OpenGL
else
LDFLAGS+=-lGL -lGLU
endif

INCLUDES=-I./include/ -I./src/utils/
SRC_EXAMPLE=$(wildcard src/example/*.cpp) 
OBJ_EXAMPLE=$(SRC_EXAMPLE:.cpp=.o)
SRC_RL=$(wildcard src/rl/*.cpp) 
OBJ_RL=$(SRC_RL:.cpp=.o)
SRC_UNICURSE=$(wildcard src/unicurse/*.cpp) 
OBJ_UNICURSE=$(SRC_UNICURSE:.cpp=.o)

#all: cwaggle_example cwaggle_rl cwaggle_unicurse
all: cwaggle_unicurse

cwaggle_example:$(OBJ_EXAMPLE) Makefile
	$(CC) $(OBJ_EXAMPLE) -o ./bin/$@ $(LDFLAGS)

cwaggle_rl:$(OBJ_RL) Makefile
	$(CC) $(OBJ_RL) -o ./bin/$@ $(LDFLAGS)

cwaggle_unicurse:$(OBJ_UNICURSE) Makefile
	$(CC) $(OBJ_UNICURSE) -o ./bin/$@ $(LDFLAGS)

.cpp.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm $(OBJ_EXAMPLE) $(OBJ_RL) $(OBJ_UNICURSE) bin/cwaggle_example bin/cwaggle_rl bin/cwaggle_unicurse
