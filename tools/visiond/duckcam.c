/*
 * duckcam: the head camera (IMX219 on the A733's sunxi VIN) with the vendor ISP's auto exposure /
 * white balance running, streaming grey frames to stdout for visiond.
 *
 * Without the ISP the sensor sits at a fixed exposure (2026-10-17, daylight: mean 25 / 255 and a
 * magenta cast), and the ISP has to run in the process that streams: ispStart before STREAMON,
 * ispWaitToExit after STREAMOFF (the vendor's /opt/v4l2_opencv_demo). Only the luma plane goes out
 * (QR codes and faces need no colour), optionally downscaled, every Nth frame:
 *
 *   per frame: "DCAM" | u32 width | u32 height | u32 sequence | i64 timestamp_ns (CLOCK_MONOTONIC,
 *              the V4L2 buffer's) | u32 exposure_us | u32 gain (ISP units) | width*height bytes
 *
 *   gcc -O2 -o duckcam duckcam.c -lAWIspApi -lisp -lisp_ini -lpthread -ldl -lm
 *   ./duckcam [--every N] [--half | --nv12] [--frames N] [--no-isp] > frames.bin
 *
 * --nv12 sends colour instead: magic "DCNV" and the full frame as NV12 (width*height luma, then
 * width*height/2 interleaved chroma), for a photo Jev looks at. The driver's chroma plane starts
 * after the luma plane's 16-aligned height (1088) and its last rows are not written (memory:
 * opi-zero3w-camera) — crop the bottom 16 rows after conversion.
 *
 * Needs the vin modules loaded (/etc/modules-load.d/camera.conf) and the video group. Never run
 * `v4l2-ctl --list-ctrls-menus` on this device: it hangs the board.
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include <AWIspApi.h>

#define NBUF 4
#define WIDTH 1920
#define HEIGHT 1080

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

/* The frames go out on a private copy of stdout; fd 1 itself points at stderr, because the vendor
 * ISP library prints its banner and logs to stdout and would corrupt the stream (2026-10-17). */
static int frames_fd = 1;

static int write_all(const void *p, size_t n) {
    const uint8_t *b = p;
    while (n) {
        ssize_t w = write(frames_fd, b, n);
        if (w < 0) { if (errno == EINTR) continue; return -1; }
        b += w; n -= (size_t)w;
    }
    return 0;
}

int main(int argc, char **argv) {
    int every = 1, half = 0, nv12 = 0, use_isp = 1;
    long frames = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--every") && i + 1 < argc) every = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atol(argv[++i]);
        else if (!strcmp(argv[i], "--half")) half = 1;
        else if (!strcmp(argv[i], "--nv12")) nv12 = 1;
        else if (!strcmp(argv[i], "--no-isp")) use_isp = 0;
        else { fprintf(stderr, "usage: duckcam [--every N] [--half | --nv12] [--frames N] [--no-isp]\n"); return 2; }
    }
    if (every < 1) every = 1;
    frames_fd = dup(1);
    if (frames_fd < 0 || dup2(2, 1) < 0) { perror("duckcam: dup"); return 1; }
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);
    signal(SIGPIPE, on_signal);

    int fd = open("/dev/video0", O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) { perror("duckcam: /dev/video0"); return 1; }
    struct v4l2_input inp = { .index = 0 };
    if (xioctl(fd, VIDIOC_S_INPUT, &inp) < 0) { perror("duckcam: S_INPUT"); return 1; }
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 30;
    parm.parm.capture.capturemode = 0x0002;          /* V4L2_MODE_VIDEO in the sunxi headers */
    xioctl(fd, VIDIOC_S_PARM, &parm);

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = WIDTH;
    fmt.fmt.pix_mp.height = HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0 || xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("duckcam: S_FMT/G_FMT"); return 1;
    }
    unsigned w = fmt.fmt.pix_mp.width, h = fmt.fmt.pix_mp.height, nplanes = fmt.fmt.pix_mp.num_planes;
    /* The luma rows are `w` apart (the plane is w x 1088, the height aligned to 16). The driver's
     * bytesperline is not that: 2880 one run, 5760 after a RGB3 capture — trusting it overran the
     * buffer (segfault, 2026-10-17). */
    unsigned stride = w;
    fprintf(stderr, "duckcam: %ux%u NV12, %u plane(s), driver bytesperline %u (using %u)\n", w, h, nplanes,
            fmt.fmt.pix_mp.plane_fmt[0].bytesperline, stride);

    struct v4l2_requestbuffers req = { .count = NBUF, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                       .memory = V4L2_MEMORY_MMAP };
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) { perror("duckcam: REQBUFS"); return 1; }
    void *start[NBUF];
    size_t length[NBUF];
    for (unsigned i = 0; i < NBUF; i++) {
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof buf);
        memset(planes, 0, sizeof planes);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = nplanes;
        buf.m.planes = planes;
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) { perror("duckcam: QUERYBUF"); return 1; }
        length[i] = planes[0].length;
        if (length[i] < (size_t)stride * h) {
            fprintf(stderr, "duckcam: buffer %zu bytes, smaller than the luma plane %u x %u\n", length[i], stride, h);
            return 1;
        }
        start[i] = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, planes[0].m.mem_offset);
        if (start[i] == MAP_FAILED) { perror("duckcam: mmap"); return 1; }
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) { perror("duckcam: QBUF"); return 1; }
    }

    AWIspApi *isp = NULL;
    int isp_id = -1;
    if (use_isp) {
        isp = CreateAWIspApi();
        if (isp && isp->ispApiInit() >= 0 && (isp_id = isp->ispGetIspId(0)) >= 0 && isp->ispStart(isp_id) >= 0) {
            fprintf(stderr, "duckcam: ISP started (id %d)\n", isp_id);
        } else {
            fprintf(stderr, "duckcam: ISP did not start; fixed exposure\n");
            isp_id = -1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("duckcam: STREAMON"); return 1; }

    if (nv12) half = 0;
    unsigned ow = half ? w / 2 : w, oh = half ? h / 2 : h;
    size_t luma_rows = ((size_t)h + 15) & ~(size_t)15;          /* the chroma plane starts here */
    size_t payload = nv12 ? (size_t)ow * oh * 3 / 2 : (size_t)ow * oh;
    if (nv12 && length[0] < stride * luma_rows + (size_t)stride * h / 2) {
        fprintf(stderr, "duckcam: buffer %zu bytes, too small for NV12\n", length[0]);
        return 1;
    }
    uint8_t *out = malloc(payload);
    long sent = 0;
    unsigned seq = 0;
    while (!stop && (frames == 0 || sent < frames)) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv = { 2, 0 };
        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r < 0) { if (errno == EINTR) continue; perror("duckcam: select"); break; }
        if (r == 0) { fprintf(stderr, "duckcam: no frame in 2 s\n"); continue; }
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof buf);
        memset(planes, 0, sizeof planes);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = nplanes;
        buf.m.planes = planes;
        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) { if (errno == EAGAIN) continue; perror("duckcam: DQBUF"); break; }
        if (seq++ % (unsigned)every == 0) {
            const uint8_t *y = start[buf.index];
            if (half) {
                for (unsigned row = 0; row < oh; row++) {
                    const uint8_t *a = y + (size_t)(2 * row) * stride, *b = a + stride;
                    uint8_t *o = out + (size_t)row * ow;
                    for (unsigned col = 0; col < ow; col++)
                        o[col] = (uint8_t)((a[2 * col] + a[2 * col + 1] + b[2 * col] + b[2 * col + 1] + 2) / 4);
                }
            } else {
                for (unsigned row = 0; row < oh; row++) memcpy(out + (size_t)row * ow, y + (size_t)row * stride, ow);
                if (nv12) {
                    const uint8_t *uv = y + stride * luma_rows;
                    for (unsigned row = 0; row < oh / 2; row++)
                        memcpy(out + (size_t)ow * oh + (size_t)row * ow, uv + (size_t)row * stride, ow);
                }
            }
            unsigned exp_us = 0, gain = 0;
            if (isp_id >= 0) {
                unsigned num = 0, den = 1;
                if (isp->ispGetIspExp(isp_id, &num, &den) >= 0 && den) exp_us = (unsigned)(1000000.0 * num / den);
                int g = isp->ispGetIspGain(isp_id);
                gain = g > 0 ? (unsigned)g : 0;
            }
            int64_t ts = (int64_t)buf.timestamp.tv_sec * 1000000000LL + (int64_t)buf.timestamp.tv_usec * 1000LL;
            uint8_t hdr[4 + 4 * 3 + 8 + 4 * 2];
            memcpy(hdr, nv12 ? "DCNV" : "DCAM", 4);
            memcpy(hdr + 4, &ow, 4);
            memcpy(hdr + 8, &oh, 4);
            memcpy(hdr + 12, &buf.sequence, 4);
            memcpy(hdr + 16, &ts, 8);
            memcpy(hdr + 24, &exp_us, 4);
            memcpy(hdr + 28, &gain, 4);
            if (write_all(hdr, sizeof hdr) < 0 || write_all(out, payload) < 0) stop = 1;
            sent++;
        }
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) { perror("duckcam: QBUF"); break; }
    }

    xioctl(fd, VIDIOC_STREAMOFF, &type);
    if (isp_id >= 0) {
        isp->ispStop(isp_id);
        isp->ispWaitToExit(isp_id);
    }
    if (isp) { isp->ispApiUnInit(); DestroyAWIspApi(isp); }
    for (unsigned i = 0; i < NBUF; i++) munmap(start[i], length[i]);
    close(fd);
    free(out);
    fprintf(stderr, "duckcam: %ld frames sent\n", sent);
    return 0;
}
