/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Andrew Tridgell
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
 * @file nshterm.c
 * start a nsh terminal on a given port. This can be useful for error
 * handling in startup scripts to start a nsh shell on /dev/ttyACM0
 * for diagnostics
 */

#include <nuttx/config.h>
#include <termios.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include <systemlib/err.h>

__EXPORT int nshterm_main(int argc, char *argv[]);

int
nshterm_main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("Usage: nshterm <device>\n");
        exit(1);
    }
    uint8_t retries = 0;
    int fd = -1;
    while (retries < 50) {
        /* the retries are to cope with the behaviour of /dev/ttyACM0 */
        /* which may not be ready immediately. */
        fd = open(argv[1], O_RDWR);
        if (fd != -1) {
            break;
        }
        usleep(100000);
        retries++;
    }
    if (fd == -1) {
        perror(argv[1]);
        exit(1);
    }

    /* set up the serial port with output processing */
    
    /* Try to set baud rate */
    struct termios uart_config;
    int termios_state;

    /* Back up the original uart configuration to restore it after exit */
    if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
        warnx("ERROR get termios config %s: %d\n", argv[1], termios_state);
        close(fd);
        return -1;
    }

    /* Set ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag |= (ONLCR | OPOST/* | OCRNL*/);

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERROR setting baudrate / termios config for %s (tcsetattr)\n", argv[1]);
        close(fd);
        return -1;
    }

    /* setup standard file descriptors */
    close(0);
    close(1);
    close(2);
    dup2(fd, 0);
    dup2(fd, 1);
    dup2(fd, 2);

    nsh_consolemain(0, NULL);

    close(fd);

    return OK;
}
