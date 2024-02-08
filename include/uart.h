#pragma once

#include <stddef.h>

size_t uart_write(const char* buffer, const size_t size);
size_t uart_puts(const char* buffer);

size_t uart_read(char* buffer, const size_t maxsize);
