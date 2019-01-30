#ifndef PROJECT_BASE_H
#define PROJECT_BASE_H

typedef signed int __int32_t;
typedef __int32_t int32_t;

typedef int32_t esp_err_t;

typedef unsigned int __uint32_t;
typedef __uint32_t uint32_t ;

typedef unsigned long long __uint64_t;
typedef __uint64_t uint64_t ;

#define BIT2     0x00000004
#define BIT1     0x00000002
#define BIT0     0x00000001

#define ESP_ERR_INVALID_ARG         0x102   /*!< Invalid argument */
#define ESP_OK          0       /*!< esp_err_t value indicating success (no error) */

void _esp_error_check_failed(esp_err_t rc, const char *file, int line, const char *function, const char *expression) __attribute__((noreturn));

#ifndef __ASSERT_FUNC
/* This won't happen on IDF, which defines __ASSERT_FUNC in assert.h, but it does happen when building on the host which
   uses /usr/include/assert.h or equivalent.
*/
#ifdef __ASSERT_FUNCTION
#define __ASSERT_FUNC __ASSERT_FUNCTION /* used in glibc assert.h */
#else
#define __ASSERT_FUNC "??"
#endif
#endif

#define ESP_ERROR_CHECK(x) do {                                         \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            _esp_error_check_failed(__err_rc, __FILE__, __LINE__,       \
                                    __ASSERT_FUNC, #x);                 \
        }                                                               \
    } while(0);

#endif //PROJECT_BASE_H
