#if !defined(GDB_SEMIHOSTING)
/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
// #include "stm32f10x_usart.h"
#include "stm32f4xx_hal.h"


#ifndef STDOUT_USART
#define STDOUT_USART 2
#endif

#ifndef STDERR_USART
#define STDERR_USART 2
#endif

#ifndef STDIN_USART
#define STDIN_USART 2
#endif

#undef errno
extern int errno;

#define HUSART(A) huart ## A
#define USART(STD) HUSART(STD)

#define STDOUT STDOUT_USART
#define STDERR STDERR_USART
#define STDIN STDIN_USART

extern UART_HandleTypeDef USART(STDOUT);
extern UART_HandleTypeDef USART(STDERR);
extern UART_HandleTypeDef USART(STDIN);

/*
 environ
 A pointer to a list of environment variables and their values. 
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status) {
    _write(1, "exit", 4);
    while (1) {
        ;
    }
}

int _close(int file) {
    return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid() {
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {

    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */


int _read(int file, char *ptr, int len) {
    // TODO: interrupt or DMA transfer would be more efficient
    HAL_UART_Receive(&USART(STDIN), (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {
    // TODO: interrupt or DMA transfer would be more efficient
    switch (file) {
      case STDOUT_FILENO:
        HAL_UART_Transmit(&USART(STDOUT), (uint8_t*)ptr, len, HAL_MAX_DELAY);
        break;
      case STDERR_FILENO:
        HAL_UART_Transmit(&USART(STDERR), (uint8_t*)ptr, len, HAL_MAX_DELAY);
        break;
      default:
        errno = EBADF;
        return -1;
    }
    return len;
}
#endif
