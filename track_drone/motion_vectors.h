//
// Created by firas on 20.8.2020.
//

#ifndef DAIVITESTS_MOTION_VECTORS_H
#define DAIVITESTS_MOTION_VECTORS_H
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <stdint.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/motion_vector.h>
}
#include <chrono>
#include <opencv2/core.hpp>
#include <iostream>

#include <string>
#include <algorithm>
#include <vector>
#include <stdexcept>


void ffmpeg_print_error(int err);
void ffmpeg_log_callback_null(void *ptr, int level, const char *fmt, va_list vl);
void ffmpeg_init();
bool process_frame(AVPacket *pkt);
bool read_packets();
bool read_frame(int64_t& pts, char& pictType, std::vector<AVMotionVector>& motion_vectors);
std::vector<std::vector<std::vector<int>>> get_optical_flow_matrix(const char* vid_Path);



#endif //DAIVITESTS_MOTION_VECTORS_H
