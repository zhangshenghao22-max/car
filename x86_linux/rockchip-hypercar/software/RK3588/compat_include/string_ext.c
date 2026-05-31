#include <string.h>

char *strchrnul(const char *s, int c)
{
    char *found = strchr(s, c);
    return found ? found : (char *)s + strlen(s);
}
