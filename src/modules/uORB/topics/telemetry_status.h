/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file telemetry_status.h
 *
 * Telemetry status topics - radio status outputs
 */

#ifndef TOPIC_TELEMETRY_STATUS_H
#define TOPIC_TELEMETRY_STATUS_H

#include <stdint.h>
#include "../uORB.h"

enum TELEMETRY_STATUS_RADIO_TYPE {
    TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0,
    TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO,
    TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET,
    TELEMETRY_STATUS_RADIO_TYPE_WIRE
};

/**
 * @addtogroup topics
 * @{
 */

struct telemetry_status_s {
	uint64_t timestamp;
    enum TELEMETRY_STATUS_RADIO_TYPE type;  /**< type of the radio hardware     */
	unsigned rssi;              /**< local signal strength                      */
    unsigned remote_rssi;       /**< remote signal strength                     */
    unsigned rxerrors;          /**< receive errors                             */
    unsigned fixed;             /**< count of error corrected packets           */
    uint8_t noise;              /**< background noise level                     */
    uint8_t remote_noise;       /**< remote background noise level              */
    uint8_t txbuf;              /**< how full the tx buffer is as a percentage  */
};

/**
 * @}
 */

ORB_DECLARE(telemetry_status);

#endif /* TOPIC_TELEMETRY_STATUS_H */