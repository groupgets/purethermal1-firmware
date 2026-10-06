/*
 * test_main.c - runs every registered test in a child process.
 *
 *   ./run_tests              run everything
 *   ./run_tests wedge        run tests whose name contains "wedge"
 *   ./run_tests -l           list test names
 */
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "test.h"

#define MAX_TESTS (256)

static struct { const char *name; test_fn fn; } tests[MAX_TESTS];
static int n_tests;

void test_register(const char *name, test_fn fn)
{
  if (n_tests < MAX_TESTS)
  {
    tests[n_tests].name = name;
    tests[n_tests].fn = fn;
    n_tests++;
  }
}

void test_fail(const char *file, int line, const char *msg)
{
  fflush(stdout);
  fprintf(stderr, "    %s:%d: CHECK failed: %s\n", file, line, msg);
  fflush(stderr);
  _exit(1);
}

static int cmp_name(const void *a, const void *b)
{
  return strcmp(*(const char *const *)a, *(const char *const *)b);
}

int main(int argc, char **argv)
{
  const char *filter = NULL;
  int i, run = 0, failed = 0;

  /* Constructors register in link order; sort so output is stable. */
  qsort(tests, n_tests, sizeof(tests[0]), cmp_name);

  if (argc > 1 && strcmp(argv[1], "-l") == 0)
  {
    for (i = 0; i < n_tests; i++)
      printf("%s\n", tests[i].name);
    return 0;
  }
  if (argc > 1)
    filter = argv[1];

  for (i = 0; i < n_tests; i++)
  {
    int status;
    pid_t pid;

    if (filter && !strstr(tests[i].name, filter))
      continue;
    run++;
    fflush(stdout);

    pid = fork();
    if (pid == 0)
    {
      alarm(TEST_TIMEOUT_S);
      tests[i].fn();
      fflush(stdout);
      _exit(0);
    }
    waitpid(pid, &status, 0);

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
      printf("  ok    %s\n", tests[i].name);
    else
    {
      failed++;
      if (WIFSIGNALED(status) && WTERMSIG(status) == SIGALRM)
        printf("  FAIL  %s  (timed out after %ds - firmware stopped yielding?)\n",
               tests[i].name, TEST_TIMEOUT_S);
      else if (WIFSIGNALED(status))
        printf("  FAIL  %s  (signal %d)\n", tests[i].name, WTERMSIG(status));
      else
        printf("  FAIL  %s\n", tests[i].name);
    }
  }

  printf("\n%d tests, %d passed, %d failed\n", run, run - failed, failed);
  return failed ? 1 : (run ? 0 : 2);
}
