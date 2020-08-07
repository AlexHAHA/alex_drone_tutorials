# ffmpeg

**说明：**本文档记录了源码编译ffmpeg，并创建C++工程使用ffmpeg库函数进行视频解析和显示。

**作者：**薛远奎

## Version Log

| 序号 | 版本 | 时间       | 更改内容 | 更改者 |
| ---- | ---- | ---------- | -------- | ------ |
| 1    | V0.1 | 2020-08-01 | 创建     | 薛远奎 |
| 3    |      |            |          |        |
| 4    |      |            |          |        |

[TOC]

## ffmpeg简介

FFmpeg是一套可以用来记录、转换数字音频、视频，并能将其转化为流的开源计算机程序。它包括了领先的音/视频编码库libavcodec等。

**libavformat**：用于各种音视频[封装格式](https://baike.baidu.com/item/封装格式)的生成和解析，包括获取解码所需信息以生成解码上下文结构

和读取音视频帧等功能；

**libavcodec**：用于各种类型声音/图像编解码；

**libavutil**：包含一些公共的工具函数；

**libswscale**：用于视频场景比例缩放、色彩映射转换；

**libpostproc**：用于后期效果处理；

**ffmpeg**：该项目提供的一个工具，可用于格式转换、解码或[电视卡](https://baike.baidu.com/item/电视卡)即时编码等；

**ffsever**：一个 HTTP 多媒体即时广播串流服务器；

**ffplay**：是一个简单的播放器，使用ffmpeg 库解析和解码，通过SDL显示；

一大推流行的视频软件都集成了ffmpeg：暴风影音、QQ影音、KMP、GOM Player、PotPlayer等。

## 推荐教程

1、教学网站：http://dranger.com/ffmpeg/

2、示例代码：https://github.com/rambodrahmani/ffmpeg-video-player

## 编译

编译过程我参考了pyav官网给出的ubuntu下ffmpeg的编译过程。

Ref: https://pyav.org/docs/develop/overview/installation.html

### Ubuntu >= 18.04 LTS

On **Ubuntu 18.04 LTS** everything can come from the default sources:

```
# General dependencies
sudo apt-get install -y python-dev pkg-config

# Library components
sudo apt-get install -y \
    libavformat-dev libavcodec-dev libavdevice-dev \
    libavutil-dev libswscale-dev libswresample-dev libavfilter-dev
```

### Ubuntu < 18.04 LTS

On older Ubuntu releases you will be unable to satisfy these requirements with the default package sources. We recommend compiling and installing FFmpeg from source. For FFmpeg:

```
sudo apt install \
    autoconf \
    automake \
    build-essential \
    cmake \
    libass-dev \
    libfreetype6-dev \
    libjpeg-dev \
    libtheora-dev \
    libtool \
    libvorbis-dev \
    libx264-dev \
    pkg-config \
    wget \
    yasm \
    zlib1g-dev

wget http://ffmpeg.org/releases/ffmpeg-3.2.tar.bz2
tar -xjf ffmpeg-3.2.tar.bz2
cd ffmpeg-3.2

./configure --disable-static --enable-shared --disable-doc
make
sudo make install
```

编译安装后的库文件所在路径为：**/usr/local/lib**。

## 使用

安装好ffmpeg后可以使用 `ffplay` 和`ffmpeg` 工具。

- Watch a stream over UDP, with a max reordering delay of 0.5 seconds:

  ```
  ffplay -max_delay 500000 -rtsp_transport udp rtsp://server/video.mp4
  ```

- Watch a stream tunneled over HTTP:

  ```
  ffplay -rtsp_transport http rtsp://server/video.mp4
  ```

- Send a stream in realtime to a RTSP server, for others to watch:

  ```
  ffmpeg -re -i input -f rtsp -muxdelay 0.1 rtsp://server/live.sdp
  ```

- Receive a stream in realtime:

  ```
  ffmpeg -rtsp_flags listen -i rtsp://ownaddress/live.sdp output
  ```

## 编程

### 图片

使用ffmpeg解析的图片可以是**.ppm**格式，可以使用**XnView**软件打开，选择[**XnView-Minimal**](https://www.xnview.com/en/xnview/#downloads)免费版本即可，ubuntu下可以默认支持打开.ppm文件。

### 在c++中调用函数

由于ffmpeg是纯C代码，如果希望在C++文件中调用相关函数，则其头文件包含必须声明为C。

```c++
#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

#include <stdio.h>
#ifdef __cplusplus
}
#endif
```

### ffmpeg AVFrame 转 opencv Mat

```
Mat mat(avctx->height, avctx->width, CV_8UC3, framergb->data[0], 
                       framergb->linesize[0]);
imshow("frame", mat);
waitKey(10);
```



## 示例代码

|--test_ffmpeg/

​    |--Modules/

​        |--FindFFMPEG.cmake

​    |--CMakeLists.txt

​    |--main.cpp

### 各代码文件

#### FindFFMPEG.cmake

本文件主要是找到ffmpeg各相关库文件的路径。

```cmake
##
# Find FFmpeg Libraries.
#
# Once done this will define
#   FFMPEG_FOUND        - System has the all required components.
#   FFMPEG_INCLUDE_DIRS - Include directory necessary for using the required components headers.
#   FFMPEG_LIBRARIES    - Link these to use the required ffmpeg components.
##

##
# FFMPEG_FOUND set to TRUE as default.
# Will be set to FALSE in case one of the required components is not found.
##
set(${FFMPEG_FOUND} TRUE)

##
# Find libavcodec.
##
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

##
# Add in libavcodec if found.
##
if (AVCODEC_INCLUDE_DIR AND AVCODEC_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${AVCODEC_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${AVCODEC_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(AVCODEC_INCLUDE_DIR AND AVCODEC_LIBRARY)

##
# Find libavformat
##
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

##
# Add in libavformat if found.
##
if (AVFORMAT_INCLUDE_DIR AND AVFORMAT_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${AVFORMAT_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${AVFORMAT_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(AVFORMAT_INCLUDE_DIR AND AVFORMAT_LIBRARY)

##
# Find libavdevice.
##
find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

##
# Add in libavdevice if found.
##
if (AVDEVICE_INCLUDE_DIR AND AVDEVICE_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${AVDEVICE_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${AVDEVICE_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(AVDEVICE_INCLUDE_DIR AND AVDEVICE_LIBRARY)

##
# Find libavutil.
##
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

##
# Add in libavutil if found.
##
if (AVUTIL_INCLUDE_DIR AND AVUTIL_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${AVUTIL_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${AVUTIL_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(AVUTIL_INCLUDE_DIR AND AVUTIL_LIBRARY)

##
# Find libavfilter.
##
find_path(AVFILTER_INCLUDE_DIR libavfilter/avfilter.h)
find_library(AVFILTER_LIBRARY avfilter)

##
# Add in libavfilter if found.
##
if (AVFILTER_INCLUDE_DIR AND AVFILTER_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${AVFILTER_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${AVFILTER_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(AVFILTER_INCLUDE_DIR AND AVFILTER_LIBRARY)

##
# Find libswscale.
##
find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

##
# Add in libswscale if found.
##
if (SWSCALE_INCLUDE_DIR AND SWSCALE_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${SWSCALE_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${SWSCALE_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(SWSCALE_INCLUDE_DIR AND SWSCALE_LIBRARY)

##
# Find libswresample.
##
find_path(SWRESAMPLE_INCLUDE_DIR libswresample/swresample.h)
find_library(SWRESAMPLE_LIBRARY swresample)

##
# Add in libswresample if found.
##
if (SWRESAMPLE_INCLUDE_DIR AND SWRESAMPLE_LIBRARY)
    list(APPEND FFMPEG_INCLUDE_DIRS ${SWRESAMPLE_INCLUDE_DIR})
    list(APPEND FFMPEG_LIBRARIES ${SWRESAMPLE_LIBRARY})
else()
    set(${FFMPEG_FOUND} FALSE)
endif(SWRESAMPLE_INCLUDE_DIR AND SWRESAMPLE_LIBRARY)
```

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.4.1)
project(test_ffmpeg)
##
# Include module pkg-config for CMake.
##
INCLUDE(FindPkgConfig)
##
# CMAKE_MODULE_PATH:FILEPATH=./FFmpeg-Video-Player/Modules
##
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
find_package(OpenCV REQUIRED)
find_package(FFMPEG REQUIRED)
#find_package(SDL    REQUIRED)
#find_package(SDL2   REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
###########
## Build ##
###########
# generate executable file
#add_executable(${PROJECT_NAME} main.cpp)
add_executable(tutorial tutorial01.cpp)
target_include_directories(tutorial PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(tutorial PRIVATE ${OpenCV_LIBS} ${FFMPEG_LIBRARIES})

```

#### main.cpp

```c++
// Based on: https://github.com/rambodrahmani/ffmpeg-video-player/blob/master/tutorial01/tutorial01.c
// Code based on a tutorial by Martin Bohme (boehme@inb.uni-luebeckREMOVETHIS.de)
// Tested on Gentoo, CVS version 5/01/07 compiled with GCC 4.1.1
// A small sample program that shows how to use libavformat and libavcodec to
// read video from a file.
//
// Use the Makefile to build all examples.
//
// Run using
// tutorial01 myvideofile.mpg
//
// to write the first five frames from "myvideofile.mpg" to disk in PPM
// format.
#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

#include <stdio.h>
#ifdef __cplusplus
}
#endif
#include <opencv2/opencv.hpp>            // C++
#include <opencv2/highgui/highgui_c.h>   // C
#include <opencv2/imgproc/imgproc_c.h>   // C
using namespace cv;

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame) {
  FILE *pFile;
  char szFilename[32];
  int  y;
  
  // Open file
  sprintf(szFilename, "frame%d.ppm", iFrame);
  pFile=fopen(szFilename, "wb");
  if(pFile==NULL)
    return;
  
  // Write header
  fprintf(pFile, "P6\n%d %d\n255\n", width, height);
  
  // Write pixel data
  for(y=0; y<height; y++)
    fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);
  
  // Close file
  fclose(pFile);
}
int step = 0;
int main(int argc, char *argv[]) {
  AVFormatContext *pFormatCtx = NULL;
  int             i, videoStream;
  AVCodecContext  *pCodecCtx = NULL;
  AVCodec         *pCodec = NULL;
  AVFrame         *pFrame = NULL; 
  AVFrame         *pFrameRGB = NULL;
  AVPacket        packet;
  int             frameFinished;
  int             numBytes;
  uint8_t         *buffer = NULL;

  AVDictionary    *optionsDict = NULL;
  struct SwsContext      *sws_ctx = NULL;
  
  if(argc < 2) {
    printf("Please provide a movie file\n");
    return -1;
  }
  // Register all formats and codecs
  av_register_all();

  printf("Step %d:Open video file\n",++step);
  // Open video file
  if(avformat_open_input(&pFormatCtx, argv[1], NULL, NULL)!=0)
    return -1; // Couldn't open file
  
  // Retrieve stream information
  if(avformat_find_stream_info(pFormatCtx, NULL)<0)
    return -1; // Couldn't find stream information
  
  printf("Step %d:Dump information about file onto standard error\n",++step);
  // Dump information about file onto standard error
  av_dump_format(pFormatCtx, 0, argv[1], 0);
  
  printf("Step %d:Find the first video stream\n",++step);
  // Find the first video stream
  videoStream=-1;
  for(i=0; i<pFormatCtx->nb_streams; i++)
    if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
      videoStream=i;
      break;
    }
  if(videoStream==-1)
    return -1; // Didn't find a video stream
  
  // Get a pointer to the codec context for the video stream
  pCodecCtx=pFormatCtx->streams[videoStream]->codec;
  
  printf("Step %d:Find the decoder for the video stream\n",++step);
  // Find the decoder for the video stream
  pCodec=avcodec_find_decoder(pCodecCtx->codec_id);
  if(pCodec==NULL) {
    fprintf(stderr, "Unsupported codec!\n");
    return -1; // Codec not found
  }

  printf("Step %d:Open codec\n",++step);
  // Open codec
  if(avcodec_open2(pCodecCtx, pCodec, &optionsDict)<0)
    return -1; // Could not open codec
  
  printf("Step %d:Allocate video frame\n",++step);
  // Allocate video frame
  pFrame=av_frame_alloc();
  
  // Allocate an AVFrame structure
  pFrameRGB=av_frame_alloc();
  if(pFrameRGB==NULL)
    return -1;
  
  // Determine required buffer size and allocate buffer
  numBytes=avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width,
			      pCodecCtx->height);
  buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));

  sws_ctx =
    sws_getContext
    (
        pCodecCtx->width,
        pCodecCtx->height,
        pCodecCtx->pix_fmt,
        pCodecCtx->width,
        pCodecCtx->height,
        AV_PIX_FMT_RGB24,
        SWS_BILINEAR,
        NULL,
        NULL,
        NULL
    );
  // Assign appropriate parts of buffer to image planes in pFrameRGB
  // Note that pFrameRGB is an AVFrame, but AVFrame is a superset
  // of AVPicture
  avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_RGB24,
		 pCodecCtx->width, pCodecCtx->height);
  
  printf("Step %d:Read frames and save first five frames to disk\n",++step);
  // Read frames and save first five frames to disk
  i=0;
  while(av_read_frame(pFormatCtx, &packet)>=0) {
    printf("While loop:%d\n", i);
    // Is this a packet from the video stream?
    if(packet.stream_index==videoStream) {
      // Decode video frame
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, 
			   &packet);
      
      // Did we get a video frame?
      if(frameFinished) {
	// Convert the image from its native format to RGB
        sws_scale
        (
            sws_ctx,
            (uint8_t const * const *)pFrame->data,
            pFrame->linesize,
            0,
            pCodecCtx->height,
            pFrameRGB->data,
            pFrameRGB->linesize
        );
	// Save the frame to disk
	if(++i<=5)
	  SaveFrame(pFrameRGB, pCodecCtx->width, pCodecCtx->height, 
		    i);
      }
    }
    /**
     * convert to opencv Mat
     */
    Mat mat(pCodecCtx->height, pCodecCtx->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->linesize[0]);
    Mat img_show;

    resize(mat, img_show, Size(mat.cols/3, mat.rows/3));
    imshow("frame", img_show);
    waitKey(20);
    // Free the packet that was allocated by av_read_frame
    av_free_packet(&packet);
  }
  // Free the RGB image
  av_free(buffer);
  av_free(pFrameRGB);
  // Free the YUV frame
  av_free(pFrame);
  // Close the codec
  avcodec_close(pCodecCtx);
  // Close the video file
  avformat_close_input(&pFormatCtx);
  
  return 0;
}

```

### 运行

1、在工程目录下创建build文件夹，进入build运行cmake

```shell
$mkdir build
$cd build
$cmake ..
$make
```

2、在工程目录下放一个视频文件，运行

```
$./tutorial ../myvideo.mp4
$./tutorial ../myvideo.h265
```



## 问题汇总

### 1、PIX_FMT_RGB24

**描述：**编译报错error PIX_FMT_RGB24 was not declared in this scope

**解决：**使用AV_PIX_FMT_BGR24代替PIX_FMT_BGR24，PIX_FMT_BGR是老版本中的宏。

