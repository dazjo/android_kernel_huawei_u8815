/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ASM__ARCH_OEM_RAPI_CLIENT_H
#define __ASM__ARCH_OEM_RAPI_CLIENT_H

/*
 * OEM RAPI CLIENT Driver header file
 */

#include <linux/types.h>
#include <mach/msm_rpcrouter.h>

enum {
	OEM_RAPI_CLIENT_EVENT_NONE = 0,

	/*
	 * list of oem rapi client events
	 */

	OEM_RAPI_CLIENT_EVENT_MAX

};

struct oem_rapi_client_streaming_func_cb_arg {
	uint32_t  event;
	void      *handle;
	uint32_t  in_len;
	char      *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_cb_ret {
	uint32_t *out_len;
	char *output;
};

struct oem_rapi_client_streaming_func_arg {
	uint32_t event;
	int (*cb_func)(struct oem_rapi_client_streaming_func_cb_arg *,
		       struct oem_rapi_client_streaming_func_cb_ret *);
	void *handle;
	uint32_t in_len;
	char *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_ret {
	uint32_t *out_len;
	char *output;
};

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret);

int oem_rapi_client_close(void);

#ifdef CONFIG_HUAWEI_KERNEL
/*  Returned status codes for requested operation.                         */
  typedef enum {
    NV_DONE_S,          /* Request completed okay */
    NV_BUSY_S,          /* Request is queued */
    NV_BADCMD_S,        /* Unrecognizable command field */
    NV_FULL_S,          /* The NVM is full */
    NV_FAIL_S,          /* Command failed, reason other than NVM was full */
    NV_NOTACTIVE_S,     /* Variable was not active */
    NV_BADPARM_S,       /* Bad parameter in command block */
    NV_READONLY_S,      /* Parameter is write-protected and thus read only */
    NV_BADTG_S,         /* Item not valid for Target */
    NV_NOMEM_S,         /* free memory exhausted */
    NV_NOTALLOC_S,      /* address is not a valid allocation */
    NV_STAT_ENUM_PAD = 0x7FFF,     /* Pad to 16 bits on ARM */
    NV_RPC_ERROR_S   = NV_STAT_ENUM_PAD+1, /* nv rpc call error */
    NV_STAT_ENUM_MAX = 0x7FFFFFFF     /* Pad to 16 bits on ARM */
  } nv_stat_enum_type;

/* usb rpc to replace pcom mechanism for fix reset issue */
/*
 * the oem_rapi_client_streaming write nv function
 * it can be used to all kernel file
 * the caller must ensure the pointer not be NULL.
 */
nv_stat_enum_type oem_rapi_write_nv(u16 nv, void *buf, u8 size);
/*
 * the oem_rapi_client_streaming read nv function
 * it can be used to all kernel file
 * the caller must ensure the pointer not be NULL.
 */
nv_stat_enum_type oem_rapi_read_nv(u16 nv, void *buf, u8 size);
#endif

struct msm_rpc_client *oem_rapi_client_init(void);

#endif
