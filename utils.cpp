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

#define MODULE_TAG "utils"

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

//#include "mpp_mem.h"
//#include "mpp_log.h"
#include "utils.h"

void _show_options(int count, OptionInfo *options)
{
    int i;
    for (i = 0; i < count; i++) {
        printf("-%s  %-16s\t%s\n",
                options[i].name, options[i].argname, options[i].help);
    }
}

void dump_mpp_frame_to_file(MppFrame frame, FILE *fp)
{
    RK_U32 width    = 0;
    RK_U32 height   = 0;
    RK_U32 h_stride = 0;
    RK_U32 v_stride = 0;
    MppFrameFormat fmt  = MPP_FMT_YUV420SP;
    MppBuffer buffer    = NULL;
    RK_U8 *base = NULL;

    if (NULL == fp || NULL == frame)
        return ;

    width    = mpp_frame_get_width(frame);
    height   = mpp_frame_get_height(frame);
    h_stride = mpp_frame_get_hor_stride(frame);
    v_stride = mpp_frame_get_ver_stride(frame);
    fmt      = mpp_frame_get_fmt(frame);
    buffer   = mpp_frame_get_buffer(frame);

    if (NULL == buffer)
        return ;

    base = (RK_U8 *)mpp_buffer_get_ptr(buffer);

    switch (fmt) {
    case MPP_FMT_YUV422SP : {
        /* YUV422SP -> YUV422P for better display */
        RK_U32 i, j;
        RK_U8 *base_y = base;
        RK_U8 *base_c = base + h_stride * v_stride;
        RK_U8 *tmp = (RK_U8*)malloc(h_stride * height * 2);
        RK_U8 *tmp_u = tmp;
        RK_U8 *tmp_v = tmp + width * height / 2;

        for (i = 0; i < height; i++, base_y += h_stride)
            fwrite(base_y, 1, width, fp);

        for (i = 0; i < height; i++, base_c += h_stride) {
            for (j = 0; j < width / 2; j++) {
                tmp_u[j] = base_c[2 * j + 0];
                tmp_v[j] = base_c[2 * j + 1];
            }
            tmp_u += width / 2;
            tmp_v += width / 2;
        }

        fwrite(tmp, 1, width * height, fp);
        free(tmp);
    } break;
    case MPP_FMT_YUV420SP : {
        RK_U32 i;
        RK_U8 *base_y = base;
        RK_U8 *base_c = base + h_stride * v_stride;

        for (i = 0; i < height; i++, base_y += h_stride) {
            fwrite(base_y, 1, width, fp);
        }
        for (i = 0; i < height / 2; i++, base_c += h_stride) {
            fwrite(base_c, 1, width, fp);
        }
    } break;
    case MPP_FMT_YUV420P : {
        RK_U32 i;
        RK_U8 *base_y = base;
        RK_U8 *base_c = base + h_stride * v_stride;

        for (i = 0; i < height; i++, base_y += h_stride) {
            fwrite(base_y, 1, width, fp);
        }
        for (i = 0; i < height / 2; i++, base_c += h_stride / 2) {
            fwrite(base_c, 1, width / 2, fp);
        }
        for (i = 0; i < height / 2; i++, base_c += h_stride / 2) {
            fwrite(base_c, 1, width / 2, fp);
        }
    } break;
    case MPP_FMT_YUV444SP : {
        /* YUV444SP -> YUV444P for better display */
        RK_U32 i, j;
        RK_U8 *base_y = base;
        RK_U8 *base_c = base + h_stride * v_stride;
        RK_U8 *tmp = (RK_U8*)malloc(h_stride * height * 2);
        RK_U8 *tmp_u = tmp;
        RK_U8 *tmp_v = tmp + width * height;

        for (i = 0; i < height; i++, base_y += h_stride)
            fwrite(base_y, 1, width, fp);

        for (i = 0; i < height; i++, base_c += h_stride * 2) {
            for (j = 0; j < width; j++) {
                tmp_u[j] = base_c[2 * j + 0];
                tmp_v[j] = base_c[2 * j + 1];
            }
            tmp_u += width;
            tmp_v += width;
        }

        fwrite(tmp, 1, width * height * 2, fp);
        free(tmp);
    } break;
//    case MPP_FMT_YUV400: {
//        RK_U32 i;
//        RK_U8 *base_y = base;
//        RK_U8 *tmp = malloc(h_stride * height);
//
//        for (i = 0; i < height; i++, base_y += h_stride)
//            fwrite(base_y, 1, width, fp);
//
//        free(tmp);
//    } break;
    default : {
        printf("not supported format %d\n", fmt);
    } break;
    }
}
