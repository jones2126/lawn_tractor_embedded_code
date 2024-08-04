#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

// Dummy implementation of _write to resolve compile error
int _write(int file, char *ptr, int len) {
    // Write your code to send 'len' bytes from 'ptr' to the desired output
    // You can use UART, USB, etc. for this
    return len;
}
