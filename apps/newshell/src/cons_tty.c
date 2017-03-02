/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <inttypes.h>
#include <assert.h>
#include <ctype.h>

#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "uart/uart.h"
#include "bsp/bsp.h"

#include "console.h"

#define CONSOLE_BAUD 115200
#define CONSOLE_FLOW_CONTROL UART_FLOW_CTL_NONE
#define CONSOLE_TX_BUF_SIZE 32
#define CONSOLE_RX_BUF_SIZE 128
#define CONSOLE_ECHO 1
#define CONSOLE_HIST_ENABLE 0

/* Control characters */
#define ESC                0x1b
#define DEL                0x7f

/* ANSI escape sequences */
#define ANSI_ESC           '['
#define ANSI_UP            'A'
#define ANSI_DOWN          'B'
#define ANSI_FORWARD       'C'
#define ANSI_BACKWARD      'D'
#define ANSI_END           'F'
#define ANSI_HOME          'H'
#define ANSI_DEL           '~'

#define ESC_ESC         (1 << 0)
#define ESC_ANSI        (1 << 1)
#define ESC_ANSI_FIRST  (1 << 2)
#define ESC_ANSI_VAL    (1 << 3)
#define ESC_ANSI_VAL_2  (1 << 4)

static int esc_state;
static unsigned int ansi_val, ansi_val_2;

static struct uart_dev *uart_dev;
static uint8_t cur, end;
static struct os_eventq *avail_queue;
static struct os_eventq *lines_queue;
static uint8_t (*completion_cb)(char *line, uint8_t len);

typedef int (*stdout_func_t)(int);
extern void __stdout_hook_install(int (*hook)(int));

static int
console_out(int c)
{
    if ('\n' == c) {
        uart_blocking_tx(uart_dev, '\r');
    }
    uart_blocking_tx(uart_dev, c);

    return c;
}

size_t
console_file_write(void *arg, const char *str, size_t cnt)
{
    int i;

    for (i = 0; i < cnt; i++) {
        console_out(str[i]);
    }
    return cnt;
}

void
console_write(const char *str, int cnt)
{
    console_file_write(NULL, str, cnt);
}

/*
 * Interrupts disabled when console_tx_char/console_rx_char are called.
 * Characters sent only in blocking mode.
 */
static int
console_tx_char(void *arg)
{
    return -1;
}

static inline void
cursor_forward(unsigned int count)
{
    console_printf("\x1b[%uC", count);
}

static inline void
cursor_backward(unsigned int count)
{
    console_printf("\x1b[%uD", count);
}

static inline void
cursor_save(void)
{
    uart_blocking_tx(uart_dev, ESC);
    uart_blocking_tx(uart_dev, '[');
    uart_blocking_tx(uart_dev, 's');
}

static inline void
cursor_restore(void)
{
    uart_blocking_tx(uart_dev, ESC);
    uart_blocking_tx(uart_dev, '[');
    uart_blocking_tx(uart_dev, 'u');
}

static void
insert_char(char *pos, char c, uint8_t end)
{
    char tmp;

    /* Echo back to console */
    uart_blocking_tx(uart_dev, c);

    if (end == 0) {
        *pos = c;
        return;
    }

    tmp = *pos;
    *(pos++) = c;

    cursor_save();

    while (end-- > 0) {
        uart_blocking_tx(uart_dev, tmp);
        c = *pos;
        *(pos++) = tmp;
        tmp = c;
    }

    /* Move cursor back to right place */
    cursor_restore();
}

static void
del_char(char *pos, uint8_t end)
{
    uart_blocking_tx(uart_dev, '\b');

    if (end == 0) {
        uart_blocking_tx(uart_dev, ' ');
        uart_blocking_tx(uart_dev, '\b');
        return;
    }

    cursor_save();

    while (end-- > 0) {
        *pos = *(pos + 1);
        uart_blocking_tx(uart_dev, *(pos++));
    }

    uart_blocking_tx(uart_dev, ' ');

    /* Move cursor back to right place */
    cursor_restore();
}

static void
handle_ansi(uint8_t byte, char *line)
{
    if (esc_state & ESC_ANSI_FIRST) {
        esc_state &= ~ESC_ANSI_FIRST;
        if (!isdigit(byte)) {
            ansi_val = 1;
            goto ansi_cmd;
        }

        esc_state |= ESC_ANSI_VAL;
        ansi_val = byte - '0';
        ansi_val_2 = 0;
        return;
    }

    if (esc_state & ESC_ANSI_VAL) {
        if (isdigit(byte)) {
            if (esc_state & ESC_ANSI_VAL_2) {
                ansi_val_2 *= 10;
                ansi_val_2 += byte - '0';
            } else {
                ansi_val *= 10;
                ansi_val += byte - '0';
            }
            return;
        }

        /* Multi value sequence, e.g. Esc[Line;ColumnH */
        if (byte == ';' && !(esc_state & ESC_ANSI_VAL_2)) {
            esc_state |= ESC_ANSI_VAL_2;
            return;
        }

        esc_state &= ~ESC_ANSI_VAL;
        esc_state &= ~ESC_ANSI_VAL_2;
    }

ansi_cmd:
    switch (byte) {
    case ANSI_BACKWARD:
        if (ansi_val > cur) {
            break;
        }

        end += ansi_val;
        cur -= ansi_val;
        cursor_backward(ansi_val);
        break;
    case ANSI_FORWARD:
        if (ansi_val > end) {
            break;
        }

        end -= ansi_val;
        cur += ansi_val;
        cursor_forward(ansi_val);
        break;
    case ANSI_HOME:
        if (!cur) {
            break;
        }

        cursor_backward(cur);
        end += cur;
        cur = 0;
        break;
    case ANSI_END:
        if (!end) {
            break;
        }

        cursor_forward(end);
        cur += end;
        end = 0;
        break;
    case ANSI_DEL:
        if (!end) {
            break;
        }

        cursor_forward(1);
        del_char(&line[cur], --end);
        break;
    default:
        break;
    }

    esc_state &= ~ESC_ANSI;
}

/*
 * Interrupts disabled when console_tx_char/console_rx_char are called.
 */
static int
console_rx_char(void *arg, uint8_t byte)
{
    static struct os_event *ev;
    static struct console_input *input;

    if (!ev) {
        ev = os_eventq_get_no_wait(avail_queue);
        if (!ev)
            return 0;
        input = ev->ev_arg;
    }

    /* Handle ANSI escape mode */
    if (esc_state & ESC_ANSI) {
        handle_ansi(byte, input->line);
        return 0;
    }

    /* Handle escape mode */
    if (esc_state & ESC_ESC) {
        esc_state &= ~ESC_ESC;
        handle_ansi(byte, input->line);
        switch (byte) {
        case ANSI_ESC:
            esc_state |= ESC_ANSI;
            esc_state |= ESC_ANSI_FIRST;
            break;
        default:
            break;
        }

        return 0;
    }

    /* Handle special control characters */
    if (!isprint(byte)) {
        handle_ansi(byte, input->line);
        switch (byte) {
        case DEL:
            if (cur > 0) {
                del_char(&input->line[--cur], end);
            }
            break;
        case ESC:
            esc_state |= ESC_ESC;
            break;
        case '\r':
            input->line[cur + end] = '\0';
            uart_blocking_tx(uart_dev, '\r');
            uart_blocking_tx(uart_dev, '\n');
            cur = 0;
            end = 0;
            os_eventq_put(lines_queue, ev);
            input = NULL;
            ev = NULL;
            break;
        case '\t':
            if (completion_cb && !end) {
                cur += completion_cb(input->line, cur);
            }
            break;
        default:
            break;
        }

        return 0;
    }

    /* Ignore characters if there's no more buffer space */
    if (cur + end < sizeof(input->line) - 1) {
        insert_char(&input->line[cur++], byte, end);
    }
    return 0;
}

int
console_is_init(void)
{
    return (uart_dev != NULL);
}

int
console_init(struct os_eventq *avail, struct os_eventq *lines,
             uint8_t (*completion)(char *str, uint8_t len))
{
    struct uart_conf uc = {
        .uc_speed = CONSOLE_BAUD,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = CONSOLE_FLOW_CONTROL,
        .uc_tx_char = console_tx_char,
        .uc_rx_char = console_rx_char,
    };

    avail_queue = avail;
    lines_queue = lines;
    completion_cb = completion;
    __stdout_hook_install(console_out);


    if (!uart_dev) {
        uart_dev = (struct uart_dev *)os_dev_open(CONSOLE_UART,
          OS_TIMEOUT_NEVER, &uc);
        if (!uart_dev) {
            return -1;
        }
    }
    return 0;
}
