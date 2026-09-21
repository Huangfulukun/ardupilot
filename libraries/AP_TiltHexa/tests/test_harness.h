// test_harness.h -- Minimal assert-based test harness (no gtest dependency)
#pragma once

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static int _thx_test_failures = 0;
static int _thx_test_count = 0;
static const char *_thx_current_test = 0;

#define TEST(name) \
    _thx_current_test = name; \
    printf("  TEST %s ... ", name);

#define CHECK(cond) do { \
    _thx_test_count++; \
    if (!(cond)) { \
        _thx_test_failures++; \
        printf("FAIL at %s:%d\n", __FILE__, __LINE__); \
    } \
} while(0)

#define CHECK_CLOSE(a, b, tol) do { \
    _thx_test_count++; \
    if (fabsf((a)-(b)) > (tol)) { \
        _thx_test_failures++; \
        printf("FAIL at %s:%d: %f != %f (tol=%e)\n", __FILE__, __LINE__, (double)(a), (double)(b), (double)(tol)); \
    } \
} while(0)

#define PASSED() printf("OK\n")

int test_summary() {
    printf("\n=== Test Summary: %d checks, %d failures ===\n", _thx_test_count, _thx_test_failures);
    return _thx_test_failures;
}