#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#define ALIGN (sizeof(size_t))
#define ONES ((size_t)-1/UCHAR_MAX)                                                                      
#define HIGHS (ONES * (UCHAR_MAX/2+1))
#define HASZERO(x) ((x)-ONES & ~(x) & HIGHS)

#define SS (sizeof(size_t))

#include <stdarg.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void *memset(void *dest, int c, size_t n)
{
	unsigned char *s = dest;
	c = (unsigned char)c;
	for (; ((uintptr_t)s & ALIGN) && n; n--) *s++ = c;
	if (n) {
		size_t *w, k = ONES * c;
		for (w = (void *)s; n>=SS; n-=SS, w++) *w = k;
		for (s = (void *)w; n; n--, s++) *s = c;
	}
	return dest;
}

void *memcpy(void *dest, const void *src, size_t n)
{
	void *ret = dest;
	
	//Cut rear
	uint8_t *dst8 = dest;
	const uint8_t *src8 = src;
	switch (n % 4) {
		case 3 : *dst8++ = *src8++;
		case 2 : *dst8++ = *src8++;
		case 1 : *dst8++ = *src8++;
		case 0 : ;
	}
	
	//stm32 data bus width
	uint32_t *dst32 = (void *)dst8;
	const uint32_t *src32 = (void *)src8;
	n = n / 4;
	while (n--) {
		*dst32++ = *src32++;
	}
	
	return ret;
}

char *strchr(const char *s, int c)
{
	for (; *s && *s != c; s++);
	return (*s == c) ? (char *)s : NULL;
}

char *strcpy(char *dest, const char *src)
{
	const unsigned char *s = src;
	unsigned char *d = dest;
	while ((*d++ = *s++));
	return dest;
}

char *strncpy(char *dest, const char *src, size_t n)
{
	const unsigned char *s = src;
	unsigned char *d = dest;
	while (n-- && (*d++ = *s++));
	return dest;
}

int strncmp(const char *s1, const char *s2, size_t n)
{
    for ( ; n > 0; s1++, s2++, --n)
	    if (*s1 != *s2)
	        return *(unsigned char *)s1 - *(unsigned char *)s2;
	    else if (*s1 == '\0')
	        return 0;
    return 0;
}

int atoi(const char *str){
        int result = 0;
        while (*str != '\0'){
                result = result * 10;
                result = result + *str - '0';
                str++;
        }
        return result;
}

#define MaxDigit 6
/*
* Main part of itoa and xtoa
* Utilize the concept of long division to implement
*/
void _toa(int in_num, char *out_str, int base, int digit){
	
    int Mdigit = digit;
    int neg = 0;
    out_str[digit--] = '\0';
    
    if(in_num == 0) out_str[digit--] = '0';    
    else if(in_num < 0){
        in_num = -in_num;
        neg = 1;
    }

    while(in_num > 0){

        if(base == 16 && in_num % base >= 10)
            out_str[digit--] = (in_num % base) + 'A' - 10;
        else
            out_str[digit--] = (in_num % base) + '0';
        
        in_num /= base;
    }//End of while(in_num > 0)
    
    if(base == 16){
        out_str[digit--] = 'x';
        out_str[digit--] = '0';
    }    

    if(neg) out_str[digit--] = '-'; //negative number

	digit++;
    //reorder
    int j = 0;
    while(digit < Mdigit + 1){
        out_str[j++] = out_str[digit++];
    } 
}

void xtoa(int in_num, char *out_str){
    
    _toa(in_num, out_str, 16, MaxDigit + 4);//MaxDigit + 4 that can contain address
}


void itoa(int in_num, char *out_str){
   
    _toa(in_num, out_str, 10, MaxDigit);
}


void qprintf(xQueueHandle tx_queue, const char *format, ...){
    va_list ap;
    va_start(ap, format);
    int curr_ch = 0;
    char out_ch[2] = {'\0', '\0'};
    char newLine[3] = {'\n' , '\r', '\0'};
    char percentage[] = "%";
    char *str;
    char str_num[10];
    int out_int;

    /* Block for 1ms. */
     const portTickType xDelay = 0.1; // portTICK_RATE_MS;

    while( format[curr_ch] != '\0' ){
        vTaskDelay( xDelay ); 
        if(format[curr_ch] == '%'){
            if(format[curr_ch + 1] == 's'){
                str = va_arg(ap, char *);
                while (!xQueueSendToBack(tx_queue, str, portMAX_DELAY)); 
                //parameter(...,The address of a string which is put in the queue,...)
            }else if(format[curr_ch + 1] == 'd'){
                itoa(va_arg(ap, int), str_num);
                while (!xQueueSendToBack(tx_queue, str_num, portMAX_DELAY));                
            }else if(format[curr_ch + 1] == 'c'){
                out_ch[0] = (char)va_arg(ap, int);
                while (!xQueueSendToBack(tx_queue, out_ch, portMAX_DELAY));                                   
           }else if(format[curr_ch + 1] == 'x'){
                xtoa(va_arg(ap, int), str_num);
                while (!xQueueSendToBack(tx_queue, str_num, portMAX_DELAY));                                       
            }else if(format[curr_ch + 1] == '%'){
                while (!xQueueSendToBack(tx_queue, percentage, portMAX_DELAY));                                    
            }
            curr_ch++;
        }else if(format[curr_ch] == '\n'){
            while (!xQueueSendToBack(tx_queue, newLine, portMAX_DELAY));
        }else{
            out_ch[0] = format[curr_ch];
            while (!xQueueSendToBack(tx_queue, out_ch, portMAX_DELAY));         
        }
        curr_ch++;
    }//End of while
    va_end(ap);
}


//Ref from zzz0072 -----------------------------------------------
size_t strlen(const char *string)
{
    size_t chars = 0;

    while(*string++) {
        chars++;
    }
    return chars;
}




char *strcat(char *dest, const char *src)
{
    size_t src_len = strlen(src);
    size_t dest_len = strlen(dest);

    if (!dest || !src) {
        return dest;
    }

    memcpy(dest + dest_len, src, src_len + 1);
    return dest;
}

int puts(const char *msg)
{
    if (!msg) {
        return -1;
    }

    return (int)fio_write(1, msg, strlen(msg));
}

static int printf_cb(char *dest, const char *src)
{
    return puts(src);
}

static int sprintf_cb(char *dest, const char *src)
{
    return (int)strcat(dest, src);
}

typedef int (*proc_str_func_t)(char *, const char *);

/* Common body for sprintf and printf */
static int base_printf(proc_str_func_t proc_str, \
                char *dest, const char *fmt_str, va_list param)
{
    char param_chr[] = {0, 0};
    int param_int = 0;
    
    long int param_lint = 0;

    char *str_to_output = 0;
    char itoa_buf[20] = {0};
    int curr_char = 0;

    /* Make sure strlen(dest) is 0
* for first strcat */
    if (dest) {
        dest[0] = 0;
    }

    /* Let's parse */
    while (fmt_str[curr_char]) {
        /* Deal with normal string
* increase index by 1 here */
        if (fmt_str[curr_char++] != '%') {
            param_chr[0] = fmt_str[curr_char - 1];
            str_to_output = param_chr;
        }
        /* % case-> retrive latter params */
        else {
            switch (fmt_str[curr_char]) {
                case 'S':
                case 's':
                    {
                        str_to_output = va_arg(param, char *);
                    }
                    break;

                case 'd':
                case 'D':
                case 'u':
                    {
                       param_int = va_arg(param, int);
                       itoa(param_int, str_to_output);
                    }
                    break;

                case 'X':
                case 'x':
                    {
                       param_int = va_arg(param, int);
                       xtoa(param_int, str_to_output);
                    }
                    break;

                case 'P':
                case 'p':
                    {
                       param_lint = va_arg(param, long int);
                       xtoa(param_int, str_to_output);
                    }
                    break;

                case 'c':
                case 'C':
                    {
                        param_chr[0] = (char) va_arg(param, int);
                        str_to_output = param_chr;
                        break;
                    }

                default:
                    {
                        param_chr[0] = fmt_str[curr_char];
                        str_to_output = param_chr;
                    }
            } /* switch (fmt_str[curr_char]) */
            curr_char++;
        } /* if (fmt_str[curr_char++] == '%') */
        proc_str(dest, str_to_output);
    } /* while (fmt_str[curr_char]) */

    return curr_char;
}

int sprintf(char *str, const char *format, ...)
{
    int rval = 0;
    va_list param = {0};

    va_start(param, format);
    rval = base_printf(sprintf_cb, (char *)str, format, param);
    va_end(param);

    return rval;
}



