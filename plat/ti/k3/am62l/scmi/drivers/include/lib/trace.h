/*
 * System Firmware Trace layer
 *
 * Debug Trace layer APIs
 *
 * Copyright (C) 2018-2022, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TRACE_H
#define TRACE_H

#include <lib/bitops.h>
#include <types/short_types.h>

#include <lib/trace_protocol.h>
#include <tisci/tisci_protocol.h>
#include <common/debug.h>
#define TRACE_PRINT_MAX_LENGTH                           255
#define TRACE_NUM_BUFFER_CONVERSION_SIZE                 ((sizeof(uint32_t) * 8U) + 1U)

#define TRACE_LEVEL_ERR                                  0U
#define TRACE_LEVEL_WARN                                 1U
#define TRACE_LEVEL_INFO                                 2U
#define TRACE_LEVEL_DEBUG                                3U

#define TRACE_PM_VAL_PSC_MASK                           0x300000U
#define TRACE_PM_VAL_PD_MASK                            0xFE000U
#define TRACE_PM_VAL_DEVICE_ID_MASK                     0x3FFU

void pm_trace_debug(uint32_t action, uint32_t val);

#define pm_trace(action, val) pm_trace_debug(action, val)

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
#define CONFIG_TRACE
#endif

#endif
