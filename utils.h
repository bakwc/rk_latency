/*
 * Copyright 2015 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdio.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>

#include "mpp_frame.h"
#include "mpp_common.h"


inline uint64_t millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

inline void writeToFile(const std::string& fname, void* data, size_t size) {
    auto ptr = fopen(fname.c_str(),"wb");  // r for read, b for binary
    fwrite(data, size, 1, ptr);
    fflush(ptr);
    fclose(ptr);
}

inline void readFromFile(const std::string& fname, void* buffer, size_t size) {
    auto ptr = fopen(fname.c_str(),"rb");  // r for read, b for binary
    fread(buffer, size, 1, ptr);
    fflush(ptr);
    fclose(ptr);
}

inline std::string readFromFileFull(const std::string& fname) {
    std::ifstream fin(fname, std::ios::binary);
    std::ostringstream ostrm;
    ostrm << fin.rdbuf();
    return ostrm.str();
}


typedef struct OptionInfo_t {
    const char*     name;
    const char*     argname;
    const char*     help;
} OptionInfo;

typedef struct data_crc_t {
    RK_U32          len;
    RK_U32          sum;
    RK_U32          vor; // value of the xor
} DataCrc;

typedef struct frame_crc_t {
    DataCrc         luma;
    DataCrc         chroma;
} FrmCrc;


#define show_options(opt) \
    do { \
        _show_options(sizeof(opt)/sizeof(OptionInfo), opt); \
    } while (0)

#ifdef __cplusplus
extern "C" {
#endif

void _show_options(int count, OptionInfo *options);
void dump_mpp_frame_to_file(MppFrame frame, FILE *fp);

void calc_data_crc(RK_U8 *dat, RK_U32 len, DataCrc *crc);
void write_data_crc(FILE *fp, DataCrc *crc);
void read_data_crc(FILE *fp, DataCrc *crc);

void calc_frm_crc(MppFrame frame, FrmCrc *crc);
void write_frm_crc(FILE *fp, FrmCrc *crc);
void read_frm_crc(FILE *fp, FrmCrc *crc);

MPP_RET read_yuv_image(RK_U8 *buf, FILE *fp, RK_U32 width, RK_U32 height,
                       RK_U32 hor_stride, RK_U32 ver_stride,
                       MppFrameFormat fmt);
MPP_RET fill_yuv_image(RK_U8 *buf, RK_U32 width, RK_U32 height,
                       RK_U32 hor_stride, RK_U32 ver_stride, MppFrameFormat fmt,
                       RK_U32 frame_count);

typedef struct OpsLine_t {
    RK_U32      index;
    char        cmd[8];
    RK_U64      value1;
    RK_U64      value2;
} OpsLine;

RK_S32 parse_config_line(const char *str, OpsLine *info);

#ifdef __cplusplus
}
#endif

#endif /*__UTILS_H__*/
