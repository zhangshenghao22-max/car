#ifndef __CAR4_COMPAT_POSIX_STRING_H__
#define __CAR4_COMPAT_POSIX_STRING_H__

#include <stddef.h>
#include <string.h>

char *strchrnul(const char *s, int c);
char *strtok_r(char *str, const char *delim, char **saveptr);

#endif
