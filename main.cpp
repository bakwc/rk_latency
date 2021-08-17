#if defined(_WIN32)
#include "vld.h"
#endif

#define MODULE_TAG "mpi_dec_test"

#include <string>
#include <iostream>

#include <string.h>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <cassert>
#include <map>
#include <stdlib.h>

#include "rk_mpi.h"

#include "mpp_common.h"

#include "utils.h"

#define MPI_DEC_LOOP_COUNT          4
#define MPI_DEC_STREAM_SIZE         (SZ_4M)
#define MAX_FILE_NAME_LENGTH        256

std::map<int, std::string> framesData;

typedef struct {
    MppCtx          ctx;
    MppApi          *mpi;

    /* end of stream flag when set quit the loop */
    RK_U32          eos;

    /* buffer for stream data reading */
    char            *buf;

    /* input and output */
    MppBufferGroup  frm_grp;
    MppBufferGroup  pkt_grp;
    MppPacket       packet;
    size_t          packet_size;
    MppFrame        frame;

    FILE            *fp_output;
    RK_U32          frame_count;
} MpiDecLoopData;

typedef struct {
    char            file_output[MAX_FILE_NAME_LENGTH];
    MppCodingType   type;
    RK_U32          width;
    RK_U32          height;
    RK_U32          debug;

    RK_U32          have_output;

    RK_S32          timeout;
} MpiDecTestCmd;

static int decode_simple(MpiDecLoopData *data)
{
    RK_U32 pkt_done = 0;
    RK_U32 pkt_eos  = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;
    char   *buf = data->buf;
    MppPacket packet = data->packet;
    MppFrame  frame  = NULL;

    static int frameNum = 0;
    ++frameNum;
    static std::map<int, uint64_t> loadTimes;

    int read_size = 0;
    if (framesData.find(frameNum-1) != framesData.end()) {
        std::string& frameData = framesData[frameNum-1];
        read_size = frameData.size();
        memcpy(buf, &frameData[0], read_size);
    } else {
        data->eos = pkt_eos = 1;
    }


//    std::cout << "Loading frame " << frameNum << ", size: " << read_size << "\n";

    loadTimes[frameNum] = millis();

    // write data to packet
    mpp_packet_write(packet, 0, buf, read_size);
    // reset pos and set valid length
    mpp_packet_set_pos(packet, buf);
    mpp_packet_set_length(packet, read_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);

    do {
        RK_S32 times = 5;
        // send the packet first if packet is not done
        uint64_t t1 = millis();
        if (!pkt_done) {
            ret = mpi->decode_put_packet(ctx, packet);
            if (MPP_OK == ret)
                pkt_done = 1;
        }

        // then get all available frame and release
        do {
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;

            try_again:
            ret = mpi->decode_get_frame(ctx, &frame);
            if (MPP_ERR_TIMEOUT == ret) {
                if (times > 0) {
                    times--;
                    usleep(2000);
                    goto try_again;
                }
                printf("decode_get_frame failed too much time\n");
            }
            if (MPP_OK != ret) {
                printf("decode_get_frame failed ret %d\n", ret);
                break;
            }

            if (frame) {
                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);

                    printf("decode_get_frame get info changed found\n");
                    printf("decoder require buffer w:h [%d:%d] stride [%d:%d]",
                            width, height, hor_stride, ver_stride);

                    ret = mpp_buffer_group_get_internal(&data->frm_grp, MPP_BUFFER_TYPE_ION);
                    if (ret) {
                        printf("get mpp buffer group  failed ret %d\n", ret);
                        break;
                    }
                    mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, data->frm_grp);

                    mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                } else {

                    uint64_t t2 = millis();

                    printf("Decode frame: %d, time: %d\n", data->frame_count+1, (int)(t2-loadTimes[data->frame_count+1]));

                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) {
                        printf("decoder_get_frame get err info:%d discard:%d.\n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
//                    printf("decode_get_frame get frame %d\n", data->frame_count++);
                    data->frame_count++;
                    if (data->fp_output && !err_info)
                        dump_mpp_frame_to_file(frame, data->fp_output);
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);
                frame = NULL;
                get_frm = 1;
            }

            // if last packet is send but last frame is not found continue
            if (pkt_eos && pkt_done && !frm_eos) {
                usleep(10000);
                continue;
            }

            if (frm_eos) {
                printf("found last frame\n");
                break;
            }

            if (get_frm)
                continue;
            break;
        } while (1);

        if (pkt_done)
            break;

        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 3ms
         * * is enough.
         */
        usleep(3000);
    } while (1);

    return ret;
}

int mpi_dec_test_decode(MpiDecTestCmd *cmd)
{
    MPP_RET ret         = MPP_OK;
    size_t file_size    = 0;

    uint64_t t1, t2;

    // base flow context
    MppCtx ctx          = NULL;
    MppApi *mpi         = NULL;

    // input / output
    MppPacket packet    = NULL;
    MppFrame  frame     = NULL;

    MpiCmd mpi_cmd      = MPP_CMD_BASE;
    MppParam param      = NULL;
    RK_U32 need_split   = 1;
    RK_U32 output_block = MPP_POLL_BLOCK;
    RK_S64 block_timeout = cmd->timeout;

    // paramter for resource malloc
    RK_U32 width        = cmd->width;
    RK_U32 height       = cmd->height;
    MppCodingType type  = cmd->type;

    // resources
    char *buf           = NULL;
    size_t packet_size  = MPI_DEC_STREAM_SIZE;
    MppBuffer pkt_buf   = NULL;
    MppBuffer frm_buf   = NULL;

    MpiDecLoopData data;

    printf("mpi_dec_test start\n");
    memset(&data, 0, sizeof(data));

    if (cmd->have_output) {
        data.fp_output = fopen(cmd->file_output, "w+b");
        if (NULL == data.fp_output) {
            printf("failed to open output file %s\n", cmd->file_output);
            goto MPP_TEST_OUT;
        }
    }

    buf = (char*)malloc(packet_size);
    if (NULL == buf) {
        printf("mpi_dec_test malloc input stream buffer failed\n");
        goto MPP_TEST_OUT;
    }

    ret = mpp_packet_init(&packet, buf, packet_size);
    if (ret) {
        printf("mpp_packet_init failed\n");
        goto MPP_TEST_OUT;
    }

    printf("mpi_dec_test decoder test start w %d h %d type %d\n", width, height, type);

    // decoder demo
    ret = mpp_create(&ctx, &mpi);

    if (MPP_OK != ret) {
        printf("mpp_create failed\n");
        goto MPP_TEST_OUT;
    }

    // NOTE: decoder split mode need to be set before init
    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        printf("mpi->control failed\n");
        goto MPP_TEST_OUT;
    }

    mpi_cmd = MPP_DEC_SET_IMMEDIATE_OUT;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        printf("mpi->control failed\n");
        goto MPP_TEST_OUT;
    }

    if (block_timeout) {
        param = &output_block;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_BLOCK, param);
        if (MPP_OK != ret) {
            printf("Failed to set blocking mode on MPI (code = %d).\n", ret);
            goto MPP_TEST_OUT;
        }

        param = &block_timeout;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, param);
        if (MPP_OK != ret) {
            printf("Failed to set blocking mode on MPI (code = %d).\n", ret);
            goto MPP_TEST_OUT;
        }
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, type);
    if (MPP_OK != ret) {
        printf("mpp_init failed\n");
        goto MPP_TEST_OUT;
    }

    data.ctx            = ctx;
    data.mpi            = mpi;
    data.eos            = 0;
    data.buf            = buf;
    data.packet         = packet;
    data.packet_size    = packet_size;
    data.frame          = frame;
    data.frame_count    = 0;

    t1 = millis();

    while (!data.eos) {
        decode_simple(&data);
    }

    t2 = millis();
    std::cout << "\nTotal time: " << (t2 - t1) << "ms\n";

    ret = mpi->reset(ctx);
    if (MPP_OK != ret) {
        printf("mpi->reset failed\n");
        goto MPP_TEST_OUT;
    }

    MPP_TEST_OUT:
    if (packet) {
        mpp_packet_deinit(&packet);
        packet = NULL;
    }

    if (frame) {
        mpp_frame_deinit(&frame);
        frame = NULL;
    }

    if (ctx) {
        mpp_destroy(ctx);
        ctx = NULL;
    }

    if (buf) {
        free(buf);
        buf = NULL;
    }

    if (data.pkt_grp) {
        mpp_buffer_group_put(data.pkt_grp);
        data.pkt_grp = NULL;
    }

    if (data.frm_grp) {
        mpp_buffer_group_put(data.frm_grp);
        data.frm_grp = NULL;
    }

    if (data.fp_output) {
        fclose(data.fp_output);
        data.fp_output = NULL;
    }

    return ret;
}

int main(int argc, char **argv)
{
    for (int i = 0; i < 30; ++i) {
        framesData[i] = readFromFileFull(
                std::string("frame_") + std::to_string(i+1) + ".264"
        );
        if (framesData[i].size() == 0) {
            std::cerr << "Failed to load frame " << i+1 << "\n";
            throw 1;
        }
//        std::cout << "Read frame " << i << ", size: " << framesData[i].size() << "\n";
    }


    RK_S32 ret = 0;
    MpiDecTestCmd  cmd_ctx;
    MpiDecTestCmd* cmd = &cmd_ctx;

    memset((void*)cmd, 0, sizeof(*cmd));

    cmd->type = MPP_VIDEO_CodingAVC;
    cmd->width = 1920;
    cmd->height = 1080;

    // parse the cmd option

    ret = mpi_dec_test_decode(cmd);
    if (MPP_OK == ret)
        printf("test success\n");
    else
        printf("test failed ret %d\n", ret);

    return ret;
}
