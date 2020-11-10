//
// Created by firas on 20.8.2020.
//

#include "motion_vectors.h"
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

using namespace std;

AVFrame* ffmpeg_pFrame;
AVFormatContext* ffmpeg_pFormatCtx;
AVStream* ffmpeg_pVideoStream;
int ffmpeg_videoStreamIndex;
size_t ffmpeg_frameWidth, ffmpeg_frameHeight;

bool ARG_OUTPUT_RAW_MOTION_VECTORS, ARG_FORCE_GRID_8, ARG_FORCE_GRID_16, ARG_OUTPUT_OCCUPANCY, ARG_QUIET, ARG_HELP;
const char* ARG_VIDEO_PATH;

void ffmpeg_print_error(int err) // copied from cmdutils.c, originally called print_error
{
    char errbuf[128];
    const char *errbuf_ptr = errbuf;

    if (av_strerror(err, errbuf, sizeof(errbuf)) < 0)
        errbuf_ptr = strerror(AVUNERROR(err));
    av_log(NULL, AV_LOG_ERROR, "ffmpeg_print_error: %s\n", errbuf_ptr);
}

void ffmpeg_log_callback_null(void *ptr, int level, const char *fmt, va_list vl)
{
}

void ffmpeg_init()
{
    av_register_all();

    if(ARG_QUIET)
    {
        av_log_set_level(AV_LOG_ERROR);
        av_log_set_callback(ffmpeg_log_callback_null);
    }

    ffmpeg_pFrame = av_frame_alloc();
    ffmpeg_pFormatCtx = avformat_alloc_context();
    ffmpeg_videoStreamIndex = -1;

    int err = 0;

    if ((err = avformat_open_input(&ffmpeg_pFormatCtx, ARG_VIDEO_PATH, NULL, NULL)) != 0)
    {
        ffmpeg_print_error(err);
        throw std::runtime_error("Couldn't open file. Possibly it doesn't exist.");
    }

    if ((err = avformat_find_stream_info(ffmpeg_pFormatCtx, NULL)) < 0)
    {
        ffmpeg_print_error(err);
        throw std::runtime_error("Stream information not found.");
    }

    for(int i = 0; i < ffmpeg_pFormatCtx->nb_streams; i++)
    {
        AVCodecContext *enc = ffmpeg_pFormatCtx->streams[i]->codec;
        if( AVMEDIA_TYPE_VIDEO == enc->codec_type && ffmpeg_videoStreamIndex < 0 )
        {
            AVCodec *pCodec = avcodec_find_decoder(enc->codec_id);
            AVDictionary *opts = NULL;
            av_dict_set(&opts, "flags2", "+export_mvs", 0);
            if (!pCodec || avcodec_open2(enc, pCodec, &opts) < 0)
                throw std::runtime_error("Codec not found or cannot open codec.");

            ffmpeg_videoStreamIndex = i;
            ffmpeg_pVideoStream = ffmpeg_pFormatCtx->streams[i];
            ffmpeg_frameWidth = enc->width;
            ffmpeg_frameHeight = enc->height;

            break;
        }
    }

    if(ffmpeg_videoStreamIndex == -1)
        throw std::runtime_error("Video stream not found.");
}

bool process_frame(AVPacket *pkt)
{
    av_frame_unref(ffmpeg_pFrame);

    int got_frame = 0;
    int ret = avcodec_decode_video2(ffmpeg_pVideoStream->codec, ffmpeg_pFrame, &got_frame, pkt);
    if (ret < 0)
        return false;

    ret = FFMIN(ret, pkt->size); /* guard against bogus return values */
    pkt->data += ret;
    pkt->size -= ret;
    return got_frame > 0;
}

bool read_packets()
{
    static bool initialized = false;
    static AVPacket pkt, pktCopy;

    while(true)
    {
        if(initialized)
        {
            if(process_frame(&pktCopy))
                return true;
            else
            {
                av_packet_unref(&pkt);
                initialized = false;
            }
        }

        int ret = av_read_frame(ffmpeg_pFormatCtx, &pkt);
        if(ret != 0)
            break;

        initialized = true;
        pktCopy = pkt;
        if(pkt.stream_index != ffmpeg_videoStreamIndex)
        {
            av_packet_unref(&pkt);
            initialized = false;
            continue;
        }
    }

    return process_frame(&pkt);
}
bool read_frame(int64_t& pts, char& pictType, vector<AVMotionVector>& motion_vectors)
{
    if(!read_packets())
        return false;

//	pictType = av_get_picture_type_char(ffmpeg_pFrame->pict_type);
    // fragile, consult fresh f_select.c and ffprobe.c when updating ffmpeg
    pts = ffmpeg_pFrame->pkt_pts != AV_NOPTS_VALUE ? ffmpeg_pFrame->pkt_pts : (ffmpeg_pFrame->pkt_dts != AV_NOPTS_VALUE ? ffmpeg_pFrame->pkt_dts : pts + 1);
    bool noMotionVectors = av_frame_get_side_data(ffmpeg_pFrame, AV_FRAME_DATA_MOTION_VECTORS) == NULL;
    if(!noMotionVectors)
    {
        // reading motion vectors, see ff_print_debug_info2 in ffmpeg's libavcodec/mpegvideo.c for reference and a fresh doc/examples/extract_mvs.c
        AVFrameSideData* sd = av_frame_get_side_data(ffmpeg_pFrame, AV_FRAME_DATA_MOTION_VECTORS);
        AVMotionVector* mvs = (AVMotionVector*)sd->data;
        int mvcount = sd->size / sizeof(AVMotionVector);
        motion_vectors = vector<AVMotionVector>(mvs, mvs + mvcount);
        int a=0;
    }
    else
    {
        motion_vectors = vector<AVMotionVector>();
    }

    return true;
}



std::vector<std::vector<std::vector<int>>> get_optical_flow_matrix(const char* vid_Path)
{
// This is basically the mpegflow.cpp (specifically "output_vectors_std" function ) but i changed it to fit our needs ,
// i might be ables to remove more things out of it and basically just use the
// ffmpeg_init() function which is the only line that uses ffmpeg, all other lines in this function are either mine or from mpegflow
    ARG_VIDEO_PATH=vid_Path;
    ffmpeg_init();
    int64_t pts, prev_pts = -1;
    char pictType;
    const static size_t max_grid_size = 512;

    vector<AVMotionVector> motionVectors;
//    vector<vector<AVMotionVector>> allmotionVectors;
    std::vector<std::vector<std::vector<int>>> all_flow_matrices;
    std::vector<std::vector<int>> emptyframe(1);
    for(int frameIndex = 1; read_frame(pts, pictType, motionVectors); frameIndex++)
    {
        std::vector<std::vector<int>> flow_matrice((ffmpeg_frameHeight / 16)*2,std::vector<int>((ffmpeg_frameWidth / 16)));
        size_t gridStep = ARG_FORCE_GRID_8 ? 8 : 16;
        pair<size_t, size_t> shape = make_pair(min(ffmpeg_frameHeight / gridStep, max_grid_size), min(ffmpeg_frameWidth / gridStep, max_grid_size));
        for(auto & mv : motionVectors)
        {
            int mvdx = mv.dst_x - mv.src_x;
            int mvdy = mv.dst_y - mv.src_y;
            size_t i_clipped = max(size_t(0), min(mv.dst_y / gridStep,shape.first - 1));
            size_t j_clipped = max(size_t(0), min(mv.dst_x / gridStep, shape.second - 1));
            if(mvdx!=0 || mvdx!=0){
                int a=0;
            }
            flow_matrice[i_clipped][j_clipped]=mvdx;
            flow_matrice[i_clipped+((ffmpeg_frameHeight / 16 ))][j_clipped]=mvdy;
        }
        all_flow_matrices.emplace_back(flow_matrice);
        if(pts <= prev_pts && prev_pts != -1)
        {
            if(!ARG_QUIET)
                fprintf(stderr, "Skipping frame %d (frame with pts %d already processed).\n", int(frameIndex), int(pts));
            continue;
        }
        prev_pts = pts;
    }
    return all_flow_matrices;

}
