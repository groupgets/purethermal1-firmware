/*
 * test.h - minimal test runner.
 *
 * Each TEST() runs in its own forked process: lepton_task.c keeps its state in
 * function-level statics, so a fresh process is the only way to give every
 * test a clean firmware. A test that hangs is killed after TEST_TIMEOUT_S,
 * which catches a firmware loop that never yields.
 */
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdlib.h>

#define TEST_TIMEOUT_S (30)

typedef void (*test_fn)(void);
void test_register(const char *name, test_fn fn);
void test_fail(const char *file, int line, const char *msg);

#define TEST(name)                                                     \
  static void name(void);                                              \
  __attribute__((constructor)) static void register_##name(void)       \
  { test_register(#name, name); }                                      \
  static void name(void)

#define CHECK(cond)                                                    \
  do { if (!(cond)) test_fail(__FILE__, __LINE__, #cond); } while (0)

#define CHECK_CMP(a, op, b)                                            \
  do {                                                                 \
    long long _a = (long long)(a), _b = (long long)(b);                \
    if (!(_a op _b)) {                                                 \
      char _m[256];                                                    \
      snprintf(_m, sizeof _m, "%s %s %s  (%lld vs %lld)",              \
               #a, #op, #b, _a, _b);                                   \
      test_fail(__FILE__, __LINE__, _m);                               \
    }                                                                  \
  } while (0)

#define CHECK_EQ(a, b) CHECK_CMP(a, ==, b)
#define CHECK_NE(a, b) CHECK_CMP(a, !=, b)
#define CHECK_GT(a, b) CHECK_CMP(a, >, b)
#define CHECK_GE(a, b) CHECK_CMP(a, >=, b)
#define CHECK_LT(a, b) CHECK_CMP(a, <, b)
#define CHECK_LE(a, b) CHECK_CMP(a, <=, b)

#endif
