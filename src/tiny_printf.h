/*
 * tiny_printf.h
 *
 *  Created on: Dec 21, 2018
 *      Author: ladislav
 */

#ifndef TINY_PRINTF_H_
#define TINY_PRINTF_H_

/* Includes */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>



/* External function prototypes (defined in syscalls.c) */
extern int _write(int fd, char *str, int len);

/* Private function prototypes */
void ts_itoa(char **buf, unsigned int d, int base);
int ts_atoi(char *str);
int ts_formatstring(char *buf, const char *fmt, va_list va);
int ts_formatlength(const char *fmt, va_list va);
int printf(const char *fmt, ...);
int sprintf(char* str, const char *fmt, ...);

#endif /* TINY_PRINTF_H_ */
