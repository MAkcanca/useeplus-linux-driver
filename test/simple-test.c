#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define BUFFER_SIZE (1024*1024)  // 1MB buffer for frame data

int main() {
    int fd = open("/dev/supercamera", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open camera");
        return 1;
    }

    unsigned char buffer[BUFFER_SIZE];
    int read_size;
    int frame_count = 0;
    int image_count = 0;

    while (frame_count < 10 && (read_size = read(fd, buffer, BUFFER_SIZE)) > 0) {
        printf("Processing frame %d (%d bytes)\n", frame_count, read_size);

        // Look for JPEG markers
        int pos = 0;
        while (pos < read_size - 1) {
            // Find JPEG SOI marker
            int soi_pos = -1;
            for (int i = pos; i < read_size - 1; i++) {
                if (buffer[i] == 0xFF && buffer[i+1] == 0xD8) {
                    soi_pos = i;
                    break;
                }
            }

            if (soi_pos == -1) break;  // No more SOI markers

            // Find next JPEG EOI marker
            int eoi_pos = -1;
            for (int i = soi_pos + 2; i < read_size - 1; i++) {
                if (buffer[i] == 0xFF && buffer[i+1] == 0xD9) {
                    eoi_pos = i;
                    break;
                }
            }

            if (eoi_pos == -1) break;  // No EOI marker

            // Extract the JPEG image
            int image_size = (eoi_pos + 2) - soi_pos;
            char filename[64];
            sprintf(filename, "image_%03d.jpg", image_count++);

            FILE *fp = fopen(filename, "wb");
            if (fp) {
                fwrite(buffer + soi_pos, 1, image_size, fp);
                fclose(fp);
                printf("Saved JPEG image: %s (%d bytes)\n", filename, image_size);
            }

            // Move position past this image
            pos = eoi_pos + 2;
        }

        frame_count++;
    }

    close(fd);
    return 0;
}
