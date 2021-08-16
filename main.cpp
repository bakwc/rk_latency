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

#include "mpp_common.h"

#include "utils.h"

#define MPI_DEC_LOOP_COUNT          4
#define MPI_DEC_STREAM_SIZE         (SZ_4K)
#define MAX_FILE_NAME_LENGTH        256

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

    FILE            *fp_input;
    FILE            *fp_output;
    RK_U32          frame_count;
} MpiDecLoopData;

typedef struct {
    char            file_input[MAX_FILE_NAME_LENGTH];
    char            file_output[MAX_FILE_NAME_LENGTH];
    MppCodingType   type;
    RK_U32          width;
    RK_U32          height;
    RK_U32          debug;

    RK_U32          have_input;
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
    size_t read_size = fread(buf, 1, data->packet_size, data->fp_input);

    if (read_size != data->packet_size || feof(data->fp_input)) {
        printf("found last packet\n");

        // setup eos flag
        data->eos = pkt_eos = 1;
    }

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
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) {
                        printf("decoder_get_frame get err info:%d discard:%d.\n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
                    printf("decode_get_frame get frame %d\n", data->frame_count++);
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
    size_t read_size = fread(buf, 1, data->packet_size, data->fp_input);

    if (read_size != data->packet_size || feof(data->fp_input)) {
        printf("found last packet\n");

        // setup eos flag
        data->eos = pkt_eos = 1;
    }

    // reset pos
    mpp_packet_set_pos(packet, buf);
    mpp_packet_set_length(packet, read_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);

    ret = mpi->poll(ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
    if (ret) {
        printf("mpp input poll failed\n");
        return ret;
    }

    ret = mpi->dequeue(ctx, MPP_PORT_INPUT, &task);  /* input queue */
    if (ret) {
        printf("mpp task input dequeue failed\n");
        return ret;
    }

    assert(task);

    mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame (task, KEY_OUTPUT_FRAME,  frame);

    ret = mpi->enqueue(ctx, MPP_PORT_INPUT, task);  /* input queue */
    if (ret) {
        printf("mpp task input enqueue failed\n");
        return ret;
    }

    /* poll and wait here */
    ret = mpi->poll(ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
    if (ret) {
        printf("mpp output poll failed\n");
        return ret;
    }

    ret = mpi->dequeue(ctx, MPP_PORT_OUTPUT, &task); /* output queue */
    if (ret) {
        printf("mpp task output dequeue failed\n");
        return ret;
    }

    assert(task);

    if (task) {
        MppFrame frame_out = NULL;
        mpp_task_meta_get_frame(task, KEY_OUTPUT_FRAME, &frame_out);
        //assert(packet_out == packet);

        if (frame) {
            /* write frame to file here */
            MppBuffer buf_out = mpp_frame_get_buffer(frame_out);

            if (buf_out) {
                void *ptr = mpp_buffer_get_ptr(buf_out);
                size_t len  = mpp_buffer_get_size(buf_out);

                if (data->fp_output)
                    fwrite(ptr, 1, len, data->fp_output);

                printf("decoded frame %d size %d\n", data->frame_count, len);
            }

            if (mpp_frame_get_eos(frame_out))
                printf("found eos frame\n");
        }

        /* output queue */
        ret = mpi->enqueue(ctx, MPP_PORT_OUTPUT, task);
        if (ret)
            printf("mpp task output enqueue failed\n");
    }

    return ret;
}

int mpi_dec_test_decode(MpiDecTestCmd *cmd)
{
    MPP_RET ret         = MPP_OK;
    size_t file_size    = 0;

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

    if (cmd->have_input) {
        data.fp_input = fopen(cmd->file_input, "rb");
        if (NULL == data.fp_input) {
            printf("failed to open input file %s\n", cmd->file_input);
            goto MPP_TEST_OUT;
        }

        fseek(data.fp_input, 0L, SEEK_END);
        file_size = ftell(data.fp_input);
        rewind(data.fp_input);
        printf("input file size %ld\n", file_size);
    }

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

    while (!data.eos) {
        decode_simple(&data);
    }

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

    if (data.fp_input) {
        fclose(data.fp_input);
        data.fp_input = NULL;
    }

    return ret;
}

int main(int argc, char **argv)
{
    RK_S32 ret = 0;
    MpiDecTestCmd  cmd_ctx;
    MpiDecTestCmd* cmd = &cmd_ctx;

    memset((void*)cmd, 0, sizeof(*cmd));

    cmd->type = MPP_VIDEO_CodingAVC;
    cmd->width = 1920;
    cmd->height = 1080;

    cmd->have_input = 1;
    cmd->have_output = 0;

    std::string inFileName = "/home/fippo/frames/test.264";
    memcpy(cmd->file_input, inFileName.data(), inFileName.size());

    // parse the cmd option

    ret = mpi_dec_test_decode(cmd);
    if (MPP_OK == ret)
        printf("test success\n");
    else
        printf("test failed ret %d\n", ret);

    return ret;
}
