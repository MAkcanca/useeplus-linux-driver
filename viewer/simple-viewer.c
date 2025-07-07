#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <X11/Xlib.h>
#include <stdio.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <time.h>

#define OPTIONS "hf:b:"

struct cmdline {
    char* filename;
    int buffersize;
    struct timespec delay;
};

struct streambuffer {
    int fd;
    unsigned char *buffer;
    size_t size;
    size_t avail;
    size_t pos;
};

#define JPEG_MARKER 0xff
#define JPEG_SOI    0xd8
//#define JPEG_EOI    0xd9

struct imagem {
    unsigned char *buffer;
    int w;
    int h;
    int c;
};

struct windowdata {
    Display         *display;
    Window          win;
    GC              gc;
    XGCValues       gcv;
    int             screen;
    int             depth;
};

struct error_mgr {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

typedef struct error_mgr * my_error_ptr;

static void my_error_exit(j_common_ptr cinfo) {
    my_error_ptr myerr = (my_error_ptr) cinfo->err;
    (*cinfo->err->output_message) (cinfo);
    longjmp(myerr->setjmp_buffer, 1);
}

static int read_JPEG(unsigned char * inbuffer, unsigned long insize, struct imagem *dest) {
    struct jpeg_decompress_struct cinfo;
    struct error_mgr jerr;
    JSAMPARRAY buffer;
    int row_stride, lines=0, total_size;
    unsigned char *buf_tmp;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;
    if (setjmp(jerr.setjmp_buffer)) {
        jpeg_destroy_decompress(&cinfo);
        return 1;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, inbuffer, insize);
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);
    row_stride = cinfo.output_width * cinfo.output_components;
    total_size= row_stride * cinfo.output_height;

    buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

    if (dest->w != cinfo.output_width || dest->h != cinfo.output_height || dest->c != cinfo.output_components) {
        dest->w = cinfo.output_width;
        dest->h = cinfo.output_height;
        dest->c = cinfo.output_components;
        free(dest->buffer);
        dest->buffer = malloc(total_size * sizeof(char));
    }
    buf_tmp=dest->buffer;

    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, buffer, 1);
        memcpy(buf_tmp, buffer[0], row_stride);
        buf_tmp+=row_stride;
        lines++;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    return (0);
}

static unsigned char find_next_marker(struct streambuffer *sb) {
    if (sb->avail > 0) {
        size_t i, n;
        for (i = sb->pos, n = sb->avail - 1; i < n; i++) {
            if (sb->buffer[i] == JPEG_MARKER) {
                unsigned char c = sb->buffer[i+1];
                if (c == JPEG_SOI || c == JPEG_EOI) {
                    sb->pos = i + 2;
                    return (c);
                }
            }
        }
    }
    return (0);
}

static int fetch(struct streambuffer *sb) {
    int read_size;

    if ((read_size = read(sb->fd, sb->buffer + sb->avail, sb->size - sb->avail)) > 0) {
        fprintf(stderr, "fetched %d\n", read_size);
        sb->avail += read_size;
    }
    return (read_size);
}

static int read_next_JPEG(struct streambuffer *sb, struct imagem *dest) {
    unsigned char marker;
    int soi, eoi;

    for (;;) {
        marker = find_next_marker(sb);
        if (marker == JPEG_SOI) {
            soi = sb->pos - 2;
            break;
        }
        if (sb->avail - sb->pos > 0) {
            // Keep last byte, might be 0xff
            sb->buffer[0] = sb->buffer[sb->avail - 1];
            sb->pos = 0;
            sb->avail = 1;
        } else {
            sb->pos = 0;
            sb->avail = 0;
        }
        if (fetch(sb) <= 0) {
            return (-1);
        }
    }

    for (;;) {
        marker = find_next_marker(sb);
        if (marker == JPEG_SOI) {
//            soi = sb->pos - 2;
            continue;
        }
        if (marker == JPEG_EOI) {
            eoi = sb->pos;
            break;
        }
        if (soi > sb->size/3) {
            memmove(sb->buffer, sb->buffer + soi, sb->avail - soi);
            sb->avail -= soi;
            soi = 0;
            sb->pos = 2;
        } else if (sb->avail > sb->size*2/3) {
            fprintf(stderr, "No JPEG found in block of %ld - buffer size too small?\n", sb->avail - sb->pos);
            return (-1);
        }
        if (fetch(sb) <= 0) {
            return (-1);
        }
    }

    fprintf(stderr, "Found JPEG with size %d\n", eoi - soi);
    return (read_JPEG(sb->buffer + soi, eoi - soi, dest));
}

static int open_JPEG(char* filename, int buffersize, struct streambuffer* sb) {
    if ((sb->fd = open(filename, O_RDONLY)) < 0) {
        return (sb->fd);
    }
    sb->size = buffersize;
    sb->buffer = malloc(sb->size);
    sb->avail = 0;
    sb->pos = 0;
    return (0);
}

static void close_JPEG(struct streambuffer* sb) {
    free(sb->buffer);
    sb->buffer = NULL;
    close(sb->fd);
    sb->fd = 0;
}

static void translate_image(struct imagem *imagem_p, unsigned char *translated_buffer, int depth) {
    switch(depth) {
        case 8:
            {
            int x,y,z,k,pixel;

            for(z=k=y=0;y!=imagem_p->h;y++)
                for(x=0;x!=imagem_p->w;x++) {
                    // for grayscale-only 8 bit depth
                    // can't work in 8 bit color display
                    pixel=(imagem_p->buffer[z++]+
                            imagem_p->buffer[z++]+
                            imagem_p->buffer[z++])/3;

                    translated_buffer[k++]=pixel;
                }
            }
            break;

        case 16:
            {
            unsigned x,y,z,k,pixel,r,g,b;
            unsigned short *word=(unsigned short *)translated_buffer;

            for(z=k=y=0;y!=imagem_p->h;y++)
                for(x=0;x!=imagem_p->w;x++) {
                    // for 16 bit depth, organization 565
                    r=imagem_p->buffer[z++]<<8;
                    g=imagem_p->buffer[z++]<<8;
                    b=imagem_p->buffer[z++]<<8;

                    r &= 0xf800;
                    g &= 0xfc00;
                    b &= 0xf800;

                    word[k++]=r|g>>5|b>>11;
                }
            }
            break;

        case 32:
        case 24:
            {
            unsigned x,y,z,k;

            for(z=k=y=0;y!=imagem_p->h;y++)
                for(x=0;x!=imagem_p->w;x++) {
                    // for 24 bit depth, organization BGRX
                    translated_buffer[k+0]=imagem_p->buffer[z+2];
                    translated_buffer[k+1]=imagem_p->buffer[z+1];
                    translated_buffer[k+2]=imagem_p->buffer[z+0];
                    k+=4; z+=3;
                }
            }
            break;

        default:
            break;
    }
}

static int create_window(unsigned int width, unsigned int height, char* name, struct windowdata* q) {
    Window        root;
    unsigned long black;

    q->display = XOpenDisplay(NULL);
    if (!q->display) {
        return (-1);
    }

    q->screen = DefaultScreen(q->display);
    root = RootWindow(q->display, q->screen);
    black = BlackPixel(q->display, q->screen);

    q->win = XCreateSimpleWindow(q->display, root, 0, 0, width, height, 0, black, black);
    XStoreName(q->display, q->win, name);
    XSelectInput(q->display, q->win, ExposureMask);
    XMapWindow(q->display, q->win);
    q->gc = XCreateGC(q->display, q->win, 0, &q->gcv);
    XFlush(q->display);
    q->depth = DefaultDepth(q->display, q->screen);
    return (0);
}

static void usage(const char* name) {
    fprintf(stderr,"Use: %s [-b buffer_kb] [-f fps] image.jpg\n", name);
}

static int parse_cmdline(int argc, char **argv, struct cmdline* args) {
    int c;

    if (argc < 2) {
        return (-1);
    }

    while ((c = getopt(argc, argv, OPTIONS)) != EOF) {
        switch (c) {
            case 'b':
                args->buffersize = atoi(optarg) * 1024;
                break;
            case 'f':
                args->delay.tv_nsec = 1000000000 / atoi(optarg);
                break;
            default:
                return (-1);
        }
    }

    args->filename = argv[optind];
    return (!args->filename);
}

int main(int argc, char **argv) {
    int           bpl;
    unsigned char *translated_buffer;
    XEvent x_event;
    XImage *ximage;
    struct cmdline args = {
            .filename = NULL,
            .buffersize = 3*64*1024,
            .delay = {
                    .tv_sec = 0,
                    .tv_nsec = 100*1000*1000
            }
    };
    struct streambuffer sb;
    struct imagem imagem_t = {
            .buffer = NULL,
            .w = 0,
            .h = 0,
            .c = 0
    };
    struct windowdata q;

    if (parse_cmdline(argc, argv, &args)) {
        usage(argv[0]);
        return (1);
    }

    if (open_JPEG(args.filename, args.buffersize, &sb)) {
        fprintf(stderr, "Can't open: %s\n", args.filename);
        return (1);
    }

    if (read_next_JPEG(&sb, &imagem_t) < 0) {
        free(imagem_t.buffer);
        close_JPEG(&sb);
        return (1);
    }
    
    if (create_window(imagem_t.w, imagem_t.h, args.filename, &q)) {
        fprintf(stderr, "Can't open display!\n");
        free(imagem_t.buffer);
        close_JPEG(&sb);
        return (1);
    }

    fprintf(stderr, "Default depth: %d\n", q.depth);
    switch (q.depth) {
        case 24:
            bpl=4;
            break;
        case 16:
        case 15:
            bpl=2;
            break;
        default:
            // in this default, you must allocate colors!
            bpl=1;
            break;
    }
    translated_buffer = (char *) malloc(bpl * imagem_t.w * imagem_t.h);

    translate_image(&imagem_t, translated_buffer, q.depth);

    ximage = XCreateImage(q.display,
            CopyFromParent,
            q.depth,
            ZPixmap,
            0,
            translated_buffer,
            imagem_t.w,
            imagem_t.h,
            bpl * 8,
            bpl * imagem_t.w);
    XFlush(q.display);

    nanosleep(&args.delay, NULL);

    for (;;) {
        while (XPending(q.display) > 0) {
            XNextEvent(q.display, &x_event);
            fprintf(stderr, "X event %d\n", x_event.type);
            switch (x_event.type) {
                //case CreateNotify:
                case Expose:
                    XPutImage(q.display,
                            q.win,
                            q.gc,
                            ximage,
                            0,0,0,0,
                            imagem_t.w,
                            imagem_t.h);
                    XFlush(q.display);
                    break;
            }
        }

        nanosleep(&args.delay, NULL);

        if (read_next_JPEG(&sb, &imagem_t) < 0) {
            break;
        }

        // TODO: Handle image size changes
        translate_image(&imagem_t, translated_buffer, q.depth);
        XPutImage(q.display,
                q.win,
                q.gc,
                ximage,
                0,0,0,0,
                imagem_t.w,
                imagem_t.h);
        XFlush(q.display);
    }

    free(imagem_t.buffer);
    close_JPEG(&sb);

    return 0;
}
