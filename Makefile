# Makefile for Linux and OSX

CFLAGS=-Wall -g
TARGET=cc2538-prog

all:
	gcc -o $(TARGET) $(CFLAGS) $(TARGET).c hex.c

clean:
	rm -f $(TARGET) $(TARGET).exe

