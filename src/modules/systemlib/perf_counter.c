/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file perf_counter.c
 *
 * @brief Performance measuring tools.
 */

#include <stdlib.h>
#include <stdio.h>
#include <sys/queue.h>
#include <drivers/drv_hrt.h>

#include "perf_counter.h"

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	sq_entry_t		link;	/**< list linkage */
	enum perf_counter_type	type;	/**< counter type */
	const char		*name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		time_start;
	uint64_t		time_total;
	uint64_t		time_least;
	uint64_t		time_most;
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		time_event;
	uint64_t		time_first;
	uint64_t		time_last;
	uint64_t		time_least;
	uint64_t		time_most;

};

/**
 * List of all known counters.
 */
static sq_queue_t	perf_counters;


perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = NULL;

	switch (type) {
	case PC_COUNT:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_count), 1);
		break;

	case PC_ELAPSED:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_elapsed), 1);
		break;

	case PC_INTERVAL:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_interval), 1);
		break;

	default:
		break;
	}

	if (ctr != NULL) {
		ctr->type = type;
		ctr->name = name;
		sq_addfirst(&ctr->link, &perf_counters);
	}

	return ctr;
}

void
perf_free(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	sq_rem(&handle->link, &perf_counters);
	free(handle);
}

void
perf_count(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;
		break;

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
		hrt_abstime now = hrt_absolute_time();

		switch (pci->event_count) {
		case 0:
			pci->time_first = now;
			break;
		case 1:
			pci->time_least = now - pci->time_last;
			pci->time_most = now - pci->time_last;
			break;
		default: {
			hrt_abstime interval = now - pci->time_last;
			if (interval < pci->time_least)
				pci->time_least = interval;
			if (interval > pci->time_most)
				pci->time_most = interval;
			break;
		}
		}
		pci->time_last = now;
		pci->event_count++;
		break;
	}

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = hrt_absolute_time();
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start != 0) {
				hrt_abstime elapsed = hrt_absolute_time() - pce->time_start;

				pce->event_count++;
				pce->time_total += elapsed;

				if ((pce->time_least > elapsed) || (pce->time_least == 0))
					pce->time_least = elapsed;

				if (pce->time_most < elapsed)
					pce->time_most = elapsed;

				pce->time_start = 0;
			}
		}
		break;

	default:
		break;
	}
}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = 0;
		}
		break;

	default:
		break;
	}
}



void
perf_reset(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count = 0;
		break;

	case PC_ELAPSED: {
		struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
		pce->event_count = 0;
		pce->time_start = 0;
		pce->time_total = 0;
		pce->time_least = 0;
		pce->time_most = 0;
		break;
	}

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
		pci->event_count = 0;
		pci->time_event = 0;
		pci->time_first = 0;
		pci->time_last = 0;
		pci->time_least = 0;
		pci->time_most = 0;
		break;
	}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_COUNT:
		printf("%s: %llu events\n",
		       handle->name,
		       ((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
		struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

		printf("%s: %llu events, %lluus elapsed, %llu avg, min %lluus max %lluus\n",
		       handle->name,
		       pce->event_count,
		       pce->time_total,
		       pce->time_total / pce->event_count,
		       pce->time_least,
		       pce->time_most);
		break;
	}

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;

		printf("%s: %llu events, %llu avg, min %lluus max %lluus\n",
		       handle->name,
		       pci->event_count,
		       (pci->time_last - pci->time_first) / pci->event_count,
		       pci->time_least,
		       pci->time_most);
		break;
	}

	default:
		break;
	}
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == NULL)
		return 0;

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
		struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
		return pce->event_count;
	}

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
		return pci->event_count;
	}

	default:
		break;
	}
	return 0;
}

void
perf_print_all(void)
{
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != NULL) {
		perf_print_counter(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}
}

void
perf_reset_all(void)
{
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != NULL) {
		perf_reset(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}
}
