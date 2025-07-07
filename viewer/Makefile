all: simple-viewer

simple-viewer: simple-viewer.c
	gcc -g -o simple-viewer simple-viewer.c -ljpeg -lX11

clean:
	rm -f simple-viewer

.PHONY: all clean
