#include <rtthread.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#undef errno
extern int errno;

void *malloc(size_t size)
{
    return rt_malloc((rt_size_t)size);
}

void free(void *ptr)
{
    rt_free(ptr);
}

void *calloc(size_t nmemb, size_t size)
{
    rt_size_t total = (rt_size_t)(nmemb * size);
    void *ptr = rt_malloc(total);

    if (ptr)
    {
        rt_memset(ptr, 0, total);
    }

    return ptr;
}

void *realloc(void *ptr, size_t size)
{
    return rt_realloc(ptr, (rt_size_t)size);
}

void *_sbrk(ptrdiff_t incr)
{
    RT_UNUSED(incr);
    errno = ENOMEM;
    return (void *)-1;
}

int _gettimeofday(struct timeval *tv, void *tz)
{
    return gettimeofday(tv, tz);
}

int _write(int fd, const void *buf, size_t count)
{
    RT_UNUSED(fd);

    if (buf && count > 0)
    {
        rt_kprintf("%.*s", (int)count, (const char *)buf);
    }

    return (int)count;
}

int _read(int fd, void *buf, size_t count)
{
    RT_UNUSED(fd);
    RT_UNUSED(buf);
    RT_UNUSED(count);

    errno = EIO;
    return -1;
}

int _close(int fd)
{
    RT_UNUSED(fd);
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st)
{
    RT_UNUSED(fd);

    if (st)
    {
        st->st_mode = S_IFCHR;
    }

    return 0;
}

int _isatty(int fd)
{
    RT_UNUSED(fd);
    return 1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
    RT_UNUSED(fd);
    RT_UNUSED(offset);
    RT_UNUSED(whence);

    errno = ESPIPE;
    return (off_t)-1;
}

void _exit(int status)
{
    rt_kprintf("_exit(%d)\n", status);
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

int _kill(pid_t pid, int sig)
{
    RT_UNUSED(pid);
    RT_UNUSED(sig);

    errno = EINVAL;
    return -1;
}

pid_t _getpid(void)
{
    return 1;
}
