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
#include <stdlib.h>

#include "rk_mpi.h"

#include "utils.h"

#define MPI_DEC_STREAM_SIZE         (SZ_4K)
#define MAX_FILE_NAME_LENGTH        256

#define mpp_log printf
#define mpp_err printf
#define msleep sleep
#define mpp_assert assert

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
    RK_S32          frame_count;
    RK_S32          frame_num;
    size_t          max_usage;

    void* jpgData;
    size_t jpgDataSize;
} MpiDecLoopData;

typedef struct {
    char            file_output[MAX_FILE_NAME_LENGTH];
    char            file_config[MAX_FILE_NAME_LENGTH];
    MppCodingType   type;
    MppFrameFormat  format;
    RK_U32          width;
    RK_U32          height;
    RK_U32          debug;

    RK_S32          timeout;
    RK_S32          frame_num;
    size_t          pkt_size;

    void* jpgData;
    size_t jpgDataSize;
} MpiDecTestCmd;


static int decode_advanced(MpiDecLoopData *data)
{
    RK_U32 pkt_eos  = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;
    char   *buf = data->buf;
    MppPacket packet = data->packet;
    MppFrame  frame  = data->frame;
    MppTask task = NULL;

    auto t1 = millis();

    size_t read_size = data->jpgDataSize;
    memcpy(buf, data->jpgData, data->jpgDataSize);

    std::cerr << "jpg cpy: " << (millis() - t1) << "ms\n";

    data->eos = pkt_eos = 1;

    // reset pos
    mpp_packet_set_pos(packet, buf);
    mpp_packet_set_length(packet, read_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);

    ret = mpi->poll(ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
    if (ret) {
        mpp_err("mpp input poll failed\n");
        return ret;
    }

    ret = mpi->dequeue(ctx, MPP_PORT_INPUT, &task);  /* input queue */
    if (ret) {
        mpp_err("mpp task input dequeue failed\n");
        return ret;
    }

    mpp_assert(task);

    mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame (task, KEY_OUTPUT_FRAME,  frame);

    ret = mpi->enqueue(ctx, MPP_PORT_INPUT, task);  /* input queue */
    if (ret) {
        mpp_err("mpp task input enqueue failed\n");
        return ret;
    }

    std::cerr << "jpg enqueue: " << (millis() - t1) << "ms\n";

    /* poll and wait here */
    ret = mpi->poll(ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
    if (ret) {
        mpp_err("mpp output poll failed\n");
        return ret;
    }

    std::cerr << "jpg poll: " << (millis() - t1) << "ms\n";

    ret = mpi->dequeue(ctx, MPP_PORT_OUTPUT, &task); /* output queue */
    if (ret) {
        mpp_err("mpp task output dequeue failed\n");
        return ret;
    }

    std::cerr << "jpg dequeue: " << (millis() - t1) << "ms\n";

    mpp_assert(task);

    if (task) {
        MppFrame frame_out = NULL;
        mpp_task_meta_get_frame(task, KEY_OUTPUT_FRAME, &frame_out);
        //mpp_assert(packet_out == packet);

        std::cerr << "jpg task: " << (millis() - t1) << "ms\n";

        if (frame) {
            /* write frame to file here */

            auto t2 = millis();
            std::cerr << "Decode time: " << (t2 - t1) << "ms\n";

            if (data->fp_output)
                dump_mpp_frame_to_file(frame, data->fp_output);

            std::cerr << "dumped: " << (millis() - t1) << "ms\n";

            data->frame_count++;
            mpp_log("decoded frame %d\n", data->frame_count);

            if (mpp_frame_get_eos(frame_out))
                mpp_log("found eos frame\n");
        }

        /* output queue */
        ret = mpi->enqueue(ctx, MPP_PORT_OUTPUT, task);
        if (ret)
            mpp_err("mpp task output enqueue failed\n");
    }

    return ret;
}

int mpi_dec_test_decode(MpiDecTestCmd *cmd)
{
    MPP_RET ret         = MPP_OK;

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
    size_t packet_size  = cmd->pkt_size;
    MppBuffer pkt_buf   = NULL;
    MppBuffer frm_buf   = NULL;

    MpiDecLoopData data;

    mpp_log("mpi_dec_test start\n");
    memset(&data, 0, sizeof(data));

    data.jpgData = cmd->jpgData;
    data.jpgDataSize = cmd->jpgDataSize;

    {
        data.fp_output = fopen(cmd->file_output, "w+b");
        if (NULL == data.fp_output) {
            mpp_err("failed to open output file %s\n", cmd->file_output);
        }
    }

    RK_U32 hor_stride = MPP_ALIGN(width, 16);
    RK_U32 ver_stride = MPP_ALIGN(height, 16);

    ret = mpp_buffer_group_get_internal(&data.frm_grp, MPP_BUFFER_TYPE_ION);
    if (ret) {
        mpp_err("failed to get buffer group for input frame ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = mpp_buffer_group_get_internal(&data.pkt_grp, MPP_BUFFER_TYPE_ION);
    if (ret) {
        mpp_err("failed to get buffer group for output packet ret %d\n", ret);
    }

    ret = mpp_frame_init(&frame); /* output frame */
    if (MPP_OK != ret) {
        mpp_err("mpp_frame_init failed\n");
    }

    /*
     * NOTE: For jpeg could have YUV420 and YUV422 the buffer should be
     * larger for output. And the buffer dimension should align to 16.
     * YUV420 buffer is 3/2 times of w*h.
     * YUV422 buffer is 2 times of w*h.
     * So create larger buffer with 2 times w*h.
     */
    ret = mpp_buffer_get(data.frm_grp, &frm_buf, hor_stride * ver_stride * 4);
    if (ret) {
        mpp_err("failed to get buffer for input frame ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    packet_size = cmd->jpgDataSize;

    ret = mpp_buffer_get(data.pkt_grp, &pkt_buf, packet_size);
    if (ret) {
        mpp_err("failed to get buffer for input frame ret %d\n", ret);
        goto MPP_TEST_OUT;
    }
    mpp_packet_init_with_buffer(&packet, pkt_buf);
    buf = (char*)mpp_buffer_get_ptr(pkt_buf);

    mpp_frame_set_buffer(frame, frm_buf);

    mpp_log("mpi_dec_test decoder test start w %d h %d type %d\n", width, height, type);

    // decoder demo
    ret = mpp_create(&ctx, &mpi);

    if (MPP_OK != ret) {
        mpp_err("mpp_create failed\n");
        goto MPP_TEST_OUT;
    }

    // NOTE: decoder split mode need to be set before init
    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        mpp_err("mpi->control failed\n");
        goto MPP_TEST_OUT;
    }

    if (block_timeout) {
        param = &output_block;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_BLOCK, param);
        if (MPP_OK != ret) {
            mpp_err("Failed to set blocking mode on MPI (code = %d).\n", ret);
            goto MPP_TEST_OUT;
        }

        param = &block_timeout;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, param);
        if (MPP_OK != ret) {
            mpp_err("Failed to set blocking mode on MPI (code = %d).\n", ret);
            goto MPP_TEST_OUT;
        }
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, type);
    if (MPP_OK != ret) {
        mpp_err("mpp_init failed\n");
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
    data.frame_num      = cmd->frame_num;


    /* NOTE: change output format before jpeg decoding */
    if (cmd->format < MPP_FMT_BUTT)
        ret = mpi->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &cmd->format);

    while (!data.eos) {
        decode_advanced(&data);
    }

    ret = mpi->reset(ctx);
    if (MPP_OK != ret) {
        mpp_err("mpi->reset failed\n");
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

    {
        if (pkt_buf) {
            mpp_buffer_put(pkt_buf);
            pkt_buf = NULL;
        }

        if (frm_buf) {
            mpp_buffer_put(frm_buf);
            frm_buf = NULL;
        }
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

//    if (data.fp_input) {
//        fclose(data.fp_input);
//        data.fp_input = NULL;
//    }

    return ret;
}


int main(int argc, char **argv)
{
//    std::cerr << "azaza: " << (int)MPP_DEC_SET_OUTPUT_FORMAT << "\n";
//    exit(3);

    RK_S32 ret = 0;
    MpiDecTestCmd  cmd_ctx;
    MpiDecTestCmd* cmd = &cmd_ctx;

    memset((void*)cmd, 0, sizeof(*cmd));
    cmd->format = MPP_FMT_BUTT;
    cmd->pkt_size = MPI_DEC_STREAM_SIZE;

    cmd->type = MPP_VIDEO_CodingMJPEG;
    cmd->width = 1920;
    cmd->height = 1080;

    std::string jpgData = readFromFileFull("/home/fippo/med.jpg");
    printf("[info] jpg file loaded, size: %d\n", (int)jpgData.size());

    cmd->jpgData = &jpgData[0];
    cmd->jpgDataSize = jpgData.size();

    std::string outFile = "/home/fippo/med.yuv";
    memcpy(cmd->file_output, outFile.c_str(), outFile.size()+1);

    ret = mpi_dec_test_decode(cmd);
    if (MPP_OK != ret)
        mpp_err("test failed ret %d\n", ret);

//    mpp_env_set_u32("mpi_debug", 0x0);
    return ret;
}
