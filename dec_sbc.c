/*
 *			GPAC - Multimedia Framework C SDK
 *
 *  This file is part of GPAC / SBC decoder filter, based on libsbc from BlueZ.
 *
 *  SBC is the low-complexity subband codec every A2DP Bluetooth device must
 *  implement. A raw .sbc file is a run of self-describing frames: each one
 *  opens with a syncword and carries its own sampling frequency, channel mode,
 *  block count and bitpool, so there is no header to parse and no container -
 *  the whole file is walked frame by frame.
 */

#include <gpac/filters.h>
#include <gpac/constants.h>
#include <string.h>
#include <stdlib.h>

#include <sbc/sbc.h>

typedef struct
{
	GF_FilterPid *ipid, *opid;
	sbc_t sbc;
	Bool started;
	u32 sample_rate, nb_chan;
} GF_SBCDecCtx;

static const u32 sbc_freqs[4] = {16000, 32000, 44100, 48000};

static GF_Err sbcdec_configure_pid(GF_Filter *filter, GF_FilterPid *pid, Bool is_remove)
{
	GF_SBCDecCtx *ctx = (GF_SBCDecCtx *)gf_filter_get_udta(filter);

	if (is_remove)
	{
		if (ctx->opid)
		{
			gf_filter_pid_remove(ctx->opid);
			ctx->opid = NULL;
		}
		ctx->ipid = NULL;
		return GF_OK;
	}
	if (!gf_filter_pid_check_caps(pid))
		return GF_NOT_SUPPORTED;

	ctx->ipid = pid;
	gf_filter_pid_set_framing_mode(pid, GF_TRUE);

	if (!ctx->opid)
		ctx->opid = gf_filter_pid_new(filter);

	/* The real rate and channel count only become known once the first frame
	 * has been decoded; GPAC resolves the graph from the output properties
	 * before that, so the most common A2DP setup goes out now and process()
	 * corrects it. */
	ctx->sample_rate = 44100;
	ctx->nb_chan = 2;

	gf_filter_pid_copy_properties(ctx->opid, pid);
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_STREAM_TYPE, &PROP_UINT(GF_STREAM_AUDIO));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CODECID, &PROP_UINT(GF_CODECID_RAW));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_AUDIO_FORMAT, &PROP_UINT(GF_AUDIO_FMT_S16));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_SAMPLE_RATE, &PROP_UINT(ctx->sample_rate));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_TIMESCALE, &PROP_UINT(ctx->sample_rate));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_NUM_CHANNELS, &PROP_UINT(ctx->nb_chan));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CHANNEL_LAYOUT,
	                           &PROP_LONGUINT(GF_AUDIO_CH_FRONT_LEFT | GF_AUDIO_CH_FRONT_RIGHT));

	return GF_OK;
}

static GF_Err sbcdec_process(GF_Filter *filter)
{
	GF_FilterPacket *pck, *dst_pck;
	u8 *data, *output, *pcm = NULL;
	u32 size;
	size_t pos = 0, total = 0, capacity;
	GF_SBCDecCtx *ctx = (GF_SBCDecCtx *)gf_filter_get_udta(filter);

	pck = gf_filter_pid_get_packet(ctx->ipid);
	if (!pck)
	{
		if (gf_filter_pid_is_eos(ctx->ipid))
		{
			gf_filter_pid_set_eos(ctx->opid);
			return GF_EOS;
		}
		return GF_OK;
	}
	data = (u8 *)gf_filter_pck_get_data(pck, &size);
	if (!data || !size)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OK;
	}

	if (!ctx->started)
	{
		sbc_init(&ctx->sbc, 0L);
		ctx->sbc.endian = SBC_LE;
		ctx->started = GF_TRUE;
	}

	/* SBC compresses roughly fourfold at the usual bitpools; starting at eight
	 * times the input and growing on demand keeps this to a single allocation
	 * for any ordinary file. */
	capacity = (size_t)size * 8;
	pcm = (u8 *)gf_malloc(capacity);
	if (!pcm)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OUT_OF_MEM;
	}

	while (pos < size)
	{
		size_t written = 0;
		ssize_t used;

		if (total + 4096 > capacity)
		{
			u8 *bigger = (u8 *)gf_realloc(pcm, capacity * 2);
			if (!bigger)
			{
				gf_free(pcm);
				gf_filter_pid_drop_packet(ctx->ipid);
				return GF_OUT_OF_MEM;
			}
			pcm = bigger;
			capacity *= 2;
		}

		used = sbc_decode(&ctx->sbc, data + pos, size - pos,
		                  pcm + total, capacity - total, &written);
		/* A negative return means the bytes at pos are not a frame header.
		 * Stopping is the honest response: SBC frames are contiguous, so a bad
		 * one means the rest cannot be trusted either. */
		if (used <= 0)
			break;
		pos += (size_t)used;
		total += written;
	}

	if (!total)
	{
		gf_free(pcm);
		gf_filter_pid_drop_packet(ctx->ipid);
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[SBCDec] No decodable SBC frame found\n"));
		return GF_NON_COMPLIANT_BITSTREAM;
	}

	{
		u32 sr = sbc_freqs[ctx->sbc.frequency & 3];
		u32 ch = (ctx->sbc.mode == SBC_MODE_MONO) ? 1 : 2;
		if ((sr != ctx->sample_rate) || (ch != ctx->nb_chan))
		{
			ctx->sample_rate = sr;
			ctx->nb_chan = ch;
			gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_SAMPLE_RATE, &PROP_UINT(sr));
			gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_TIMESCALE, &PROP_UINT(sr));
			gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_NUM_CHANNELS, &PROP_UINT(ch));
			gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CHANNEL_LAYOUT,
			                           &PROP_LONGUINT((ch == 2)
			                                              ? (GF_AUDIO_CH_FRONT_LEFT | GF_AUDIO_CH_FRONT_RIGHT)
			                                              : GF_AUDIO_CH_FRONT_CENTER));
		}
	}

	dst_pck = gf_filter_pck_new_alloc(ctx->opid, (u32)total, &output);
	if (!dst_pck)
	{
		gf_free(pcm);
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OUT_OF_MEM;
	}
	memcpy(output, pcm, total);
	gf_free(pcm);

	gf_filter_pck_set_cts(dst_pck, 0);
	gf_filter_pck_set_duration(dst_pck, (u32)(total / (2 * ctx->nb_chan)));
	gf_filter_pck_set_sap(dst_pck, GF_FILTER_SAP_1);
	gf_filter_pck_send(dst_pck);

	gf_filter_pid_drop_packet(ctx->ipid);
	gf_filter_pid_set_eos(ctx->opid);
	return GF_EOS;
}

static void sbcdec_finalize(GF_Filter *filter)
{
	GF_SBCDecCtx *ctx = (GF_SBCDecCtx *)gf_filter_get_udta(filter);
	if (ctx->started)
	{
		sbc_finish(&ctx->sbc);
		ctx->started = GF_FALSE;
	}
}

static const GF_FilterCapability SBCDecCaps[] =
	{
		CAP_UINT(GF_CAPS_INPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_FILE),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_FILE_EXT, "sbc|msbc"),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_MIME, "audio/sbc|audio/x-sbc"),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_AUDIO),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_CODECID, GF_CODECID_RAW),
};

GF_FilterRegister SBCDecoderRegister = {
	.name = "sbcdec",
	GF_FS_SET_DESCRIPTION("SBC (Bluetooth subband codec) decoder")
		GF_FS_SET_HELP("This filter decodes raw SBC audio, the low-complexity subband codec of Bluetooth A2DP, using libsbc from BlueZ.")
			.private_size = sizeof(GF_SBCDecCtx),
	SETCAPS(SBCDecCaps),
	.configure_pid = sbcdec_configure_pid,
	.process = sbcdec_process,
	.finalize = sbcdec_finalize,
};

const GF_FilterRegister *EMSCRIPTEN_KEEPALIVE sbcdec_register(GF_FilterSession *session)
{
	return &SBCDecoderRegister;
}

#include "filter_register.h"
__attribute__((constructor))
void register_sbcdec(void) {
    gf_filter_auto_register("sbcdec", sbcdec_register);
}
