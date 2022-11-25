CC = gcc
CXX = g++

CPPFLAGS = -I.. -I../../includes `pkg-config --cflags opencv4`
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O3
CXXFLAGS = -g -Wall -O3
LDFLAGS = -lpthread -lm 
LIBS=-L../../libs -lapriltag

TARGETS := test.exe

.PHONY: all
all: test

test: test.o 
	@echo "   [$@]"
	@$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
