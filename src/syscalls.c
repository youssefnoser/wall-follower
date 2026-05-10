#include "flash_logger.h"
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

int _write(int file, char *ptr, int len) {
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        flash_logger_write(ptr, len);
        return len;
    }
    return -1;
}

extern char _end; /* linker-defined: first byte after BSS */
static char *heap_ptr = &_end;

void *_sbrk(int incr) {
    char *prev = heap_ptr;
    heap_ptr += incr;
    return (void *)prev;
}

void _exit(int status) {
    (void)status;
    while (1); /* spin forever — no OS to return to */
}
