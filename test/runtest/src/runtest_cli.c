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

#if MYNEWT_VAL(RUNTEST_CLI)
#include <inttypes.h>
#include <console/console.h>
#include <shell/shell.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "os/mynewt.h"

#include "runtest/runtest.h"
#include "runtest_priv.h"
#include "testutil/testutil.h"

#ifndef BUILD_TARGET
#define BUILD_TARGET    "ARDUINO_ZERO"
#endif

#ifndef BUILD_ID
#define BUILD_ID "1.2.3.4"
#endif

#define RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN (128)
#define RUNTEST_CLI_BUILDID_SZ 64
char runtest_token[RUNTEST_REQ_SIZE];

int total_tests;
int total_fails;

struct ts_suite *current_ts;

static int runtest_cli_cmd(int argc, char **argv);
struct shell_cmd runtest_cmd_struct = {
    .sc_cmd = "run",
    .sc_cmd_func = runtest_cli_cmd
};


static int
runtest_cli_err_too_many_args(char *cmd_name)
{
    console_printf("Error: too many arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
runtest_cli_err_unknown_arg(char *cmd_name)
{
    console_printf("Error: unknown argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
runtest_cli_cmd_list(int argc, char **argv)
{
    if(argc > 2) {
        return runtest_cli_err_too_many_args(argv[1]);
    }

    struct ts_suite *ts;

	console_printf("tests:\n");
    SLIST_FOREACH(ts, &g_ts_suites, ts_next) {
    	console_printf("%s\n", ts->ts_name);
    }

    return (0);
}

void
runtest_cli_ts_result(char *msg, void *arg, bool passed)
{
    char buf[RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN];
    char *n;
    int n_len;
    char *s;
    int s_len;
    char *m;
    int m_len;

    int len = 35; /* str length of {"k":"","n":"","s":"","m":"","r":1} */
    len += strlen(runtest_token);

    /* How much of the test name can we log? */
    n_len = strlen(tu_case_name);
    if (len + n_len >= RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN) {
        n_len = RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN - len - 1;
    }
    len += n_len;
    n = buf;
    strncpy(n, tu_case_name, n_len + 1);

    /* How much of the suite name can we log? */
    s_len = strlen(current_ts->ts_name);
    if (len + s_len >= RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN) {
        s_len = RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN - len - 1;
    }
    len += s_len;
    s = n + n_len + 2;
    strncpy(s, current_ts->ts_name, s_len + 1);

    /* How much of the message can we log? */
    m_len = strlen(msg);
    if (len + m_len >= RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN) {
        m_len = RUNTEST_CLI_PRINTF_MAX_ENTRY_LEN - len - 1;
    }
    m = s + s_len + 2;
    strncpy(m, msg, m_len + 1);

    total_tests++;
    if (!passed) {
        total_fails++;
    }

    console_printf("{\"k\":\"%s\",\"n\":\"%s\",\"s\":\"%s\",\"m\":\"%s\",\"r\":%d}",
             runtest_token, n, s, m, passed);
}

void
runtest_cli_ts_pass(char *msg, void *arg)
{
    runtest_cli_ts_result(msg, arg, true);
}

void
runtest_cli_ts_fail(char *msg, void *arg)
{
    runtest_cli_ts_result(msg, arg, false);
}

void
runtest_cli_test_init()
{
    total_tests = 0;
    total_fails = 0;

	tu_suite_set_pass_cb(runtest_cli_ts_pass, NULL);
	tu_suite_set_fail_cb(runtest_cli_ts_fail, NULL);

    return;
}

static int
runtest_cli_cmd_test(int argc, char **argv)
{
    int run_all = 0;

    if(argc > 5) {
        return runtest_cli_err_too_many_args(argv[1]);
	}

	if(argc == 2 || (strcmp(argv[2], "all") == 0)) {
		run_all = true;
    }

	if(argc == 4) {
	    strcpy(runtest_token, argv[3]);
    }else{
	    strcpy(runtest_token, "");
    }

    runtest_cli_test_init();

    ts_config.ts_print_results = 0;
    ts_config.ts_system_assert = 0;

    /*
     * go through entire list of registered test suites
     */
    SLIST_FOREACH(current_ts, &g_ts_suites, ts_next) {
        if (run_all || !strcmp(argv[2], current_ts->ts_name)) {
            current_ts->ts_test();
        }
    }

    console_printf("%s Done ", runtest_token);

	char buildID[RUNTEST_CLI_BUILDID_SZ];
    sprintf(buildID, "%s Build %s:", BUILD_TARGET, BUILD_ID);

    console_printf("%s TEST %s - Tests run:%d pass:%d fail:%d %s\n",
             buildID,
             (total_fails ? "FAILED" : "PASSED"),
             total_tests,
             (total_tests-total_fails),
             total_fails,
             runtest_token);

    return tu_any_failed;
}

static int
runtest_cli_help(void)
{
    console_printf("%s cmd  [flags...]\n", runtest_cmd_struct.sc_cmd);
    console_printf("cmd:\n");
	console_printf("\tlist\n");
	/*
     * testname is either a specific test "all".
     * 
     * token is appened to log messages.
     */
    console_printf("\ttest [testname token]\n");
    return 0;
}

static int
runtest_cli_cmd(int argc, char **argv)
{
    if (argc == 1) {
        return runtest_cli_help();
    }

    if (argc > 1 && strcmp(argv[1], "list") == 0) {
        return runtest_cli_cmd_list(argc, argv);
    }

    if (argc > 1 && strcmp(argv[1], "test") == 0) {
        return runtest_cli_cmd_test(argc, argv);
    }

    return runtest_cli_err_unknown_arg(argv[1]);
}

#endif /* MYNEWT_VAL(RUNTEST_CLI) */
