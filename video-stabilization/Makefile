CC = g++
CFLAGS = -Iinclude -O2 `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

SRC = $(wildcard src/*.cpp)
TARGET = main

all:
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
