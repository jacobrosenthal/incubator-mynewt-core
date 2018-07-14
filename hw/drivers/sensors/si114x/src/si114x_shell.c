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

#include "os/mynewt.h"

#if MYNEWT_VAL(SI114X_CLI)
#include <string.h>
#include <errno.h>
#include "console/console.h"
#include "shell/shell.h"
#include "sensor/sensor.h"
#include "sensor/proximity.h"
#include "si114x/si114x.h"
#include "si114x_priv.h"
#include "parse/parse.h"

static int si114x_shell_cmd(int argc, char **argv);

static struct shell_cmd si114x_shell_cmd_struct = {
    .sc_cmd = "si114x",
    .sc_cmd_func = si114x_shell_cmd
};

static int
si114x_shell_err_too_many_args(char *cmd_name)
{
    console_printf("Error: too many arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
si114x_shell_err_too_few_args(char *cmd_name)
{
    console_printf("Error: too few arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
si114x_shell_err_unknown_arg(char *cmd_name)
{
    console_printf("Error: unknown argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
si114x_shell_err_invalid_arg(char *cmd_name)
{
    console_printf("Error: invalid argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
si114x_shell_help(void)
{
    console_printf("%s cmd  [flags...]\n", si114x_shell_cmd_struct.sc_cmd);
    console_printf("cmd:\n");
    console_printf("\tchip_id\n");
    console_printf("\tpeek [reg]\n");
    console_printf("\tpoke [reg value]\n");
    console_printf("\tpeekp [param]\n");
    console_printf("\tpokep [param value]\n");
    console_printf("\tdump_all\n");
    return 0;
}

static int
si114x_shell_cmd_get_chip_id(int argc, char **argv, struct si114x *si114x)
{
    uint8_t id;
    int rc;
    struct sensor_itf *itf;

    if (argc > 3) {
        return si114x_shell_err_too_many_args(argv[1]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    /* Display the chip id */
    if (argc == 2) {
        rc = si114x_get_chip_id(itf, &id);
        if (rc) {
            console_printf("id failed %d\n", rc);
        }else{
            console_printf("id: 0x%02X\n", id);
        }

        rc = si114x_get_chip_revision(itf, &id);
        if (rc) {
            console_printf("revision failed %d\n", rc);
        }else{
            console_printf("revision: 0x%02X\n", id);
        }

        rc = si114x_get_chip_sequence(itf, &id);
        if (rc) {
            console_printf("sequence failed %d\n", rc);
        }else{
            console_printf("sequence: 0x%02X\n", id);
        }
    }

    return 0;
}

static int
si114x_shell_cmd_peek(int argc, char **argv, struct si114x *si114x)
{
    int rc;
    uint8_t value;
    uint8_t reg;
    struct sensor_itf *itf;

    if (argc > 3) {
        return si114x_shell_err_too_many_args(argv[1]);
    } else if (argc < 3) {
        return si114x_shell_err_too_few_args(argv[1]);
    }

    reg = parse_ll_bounds(argv[2], 0, 34, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[2]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    rc = si114x_read8(itf, reg, &value);
    if (rc) {
        console_printf("peek failed %d\n", rc);
    }else{
        console_printf("value: 0x%02X\n", value);
    }

    return 0;
}

static int
si114x_shell_cmd_poke(int argc, char **argv, struct si114x *si114x)
{
    int rc;
    uint8_t reg;
    uint8_t value;
    struct sensor_itf *itf;

    if (argc > 4) {
        return si114x_shell_err_too_many_args(argv[1]);
    } else if (argc < 4) {
        return si114x_shell_err_too_few_args(argv[1]);
    }

    reg = parse_ll_bounds(argv[2], 0, 34, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[2]);
    }

    value = parse_ll_bounds(argv[3], 0, 255, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[3]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    rc = si114x_write8(itf, reg, value);
    if (rc) {
        console_printf("poke failed %d\n", rc);
    }else{
        console_printf("wrote: 0x%02X to 0x%02X\n", value, reg);
    }

    return 0;
}

static int
si114x_shell_cmd_peekp(int argc, char **argv, struct si114x *si114x)
{
    int rc;
    uint8_t value;
    uint8_t reg;
    struct sensor_itf *itf;

    if (argc > 3) {
        return si114x_shell_err_too_many_args(argv[1]);
    } else if (argc < 3) {
        return si114x_shell_err_too_few_args(argv[1]);
    }

    reg = parse_ll_bounds(argv[2], 0, 34, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[2]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    rc = si114x_query_param(itf, reg, &value);
    if (rc) {
        console_printf("peek failed %d\n", rc);
    }else{
        console_printf("value: 0x%02X\n", value);
    }

    return 0;
}

static int
si114x_shell_cmd_pokep(int argc, char **argv, struct si114x *si114x)
{
    int rc;
    uint8_t reg;
    uint8_t value;
    struct sensor_itf *itf;

    if (argc > 4) {
        return si114x_shell_err_too_many_args(argv[1]);
    } else if (argc < 4) {
        return si114x_shell_err_too_few_args(argv[1]);
    }

    reg = parse_ll_bounds(argv[2], 0, 34, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[2]);
    }

    value = parse_ll_bounds(argv[3], 0, 255, &rc);
    if (rc != 0) {
        return si114x_shell_err_invalid_arg(argv[3]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    rc = si114x_write_param(itf, reg, value);
    if (rc) {
        console_printf("poke failed %d\n", rc);
    }else{
        console_printf("wrote: 0x%02X to 0x%02X\n", value, reg);
    }

    return 0;
}

static int
si114x_shell_cmd_dump_all(int argc, char **argv, struct si114x *si114x)
{
    int rc;
    uint8_t value;
    int i;
    struct sensor_itf *itf;

    if (argc > 3) {
        return si114x_shell_err_too_many_args(argv[1]);
    }

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    for (i=0x0; i<=0x3E; i++){
        rc = si114x_read8(itf, i, &value);
        if (rc) {
            console_printf("dump failed %d\n", rc);
            goto err;
        }else{
            console_printf("reg 0x:%02X = 0x%02X\n", i, value);
        }
    }

    for (i=0x00; i<=0x1F; i++){
        rc = si114x_query_param(itf, i, &value);
        if (rc) {
            console_printf("dump failed %d\n", rc);
            goto err;
        }else{
            console_printf("param 0x:%02X = 0x%02X\n", i, value);
        }
    }

    return 0;
err:
    return rc;
}

static int
si114x_shell_cmd(int argc, char **argv)
{
    struct os_dev * dev;
    struct si114x * si114x;

    dev = os_dev_open("si114x_0", OS_TIMEOUT_NEVER, NULL);
    if (dev == NULL) {
        console_printf("failed to open si114x_0 device\n");
        return ENODEV;
    }

    si114x = (struct si114x *)dev;

    if (argc == 1) {
        return si114x_shell_help();
    }

    if (argc > 1 && strcmp(argv[1], "dump_all") == 0) {
        return si114x_shell_cmd_dump_all(argc, argv, si114x);
    }

    if (argc > 1 && strcmp(argv[1], "chip_id") == 0) {
        return si114x_shell_cmd_get_chip_id(argc, argv, si114x);
    }

    if (argc > 1 && strcmp(argv[1], "peek") == 0) {
        return si114x_shell_cmd_peek(argc, argv, si114x);
    }

    if (argc > 1 && strcmp(argv[1], "poke") == 0) {
        return si114x_shell_cmd_poke(argc, argv, si114x);
    }

    if (argc > 1 && strcmp(argv[1], "peek") == 0) {
        return si114x_shell_cmd_peekp(argc, argv, si114x);
    }

    if (argc > 1 && strcmp(argv[1], "poke") == 0) {
        return si114x_shell_cmd_pokep(argc, argv, si114x);
    }

    return si114x_shell_err_unknown_arg(argv[1]);
}

int
si114x_shell_init(void)
{
    int rc;

    rc = shell_cmd_register(&si114x_shell_cmd_struct);
    SYSINIT_PANIC_ASSERT(rc == 0);

    return rc;
}

#endif
