
#ifndef __ASSERT_H__
#define __ASSERT_H__

int failed_assertion (const char* ex, const char* file, int line);
void panic_mode (const char* msg, const char* file, int line);

#define assert(ex) ((ex) ? 1 : failed_assertion (#ex, __FILE__, __LINE__))
#define panic(msg) panic_mode (msg, __FILE__, __LINE__)

#endif
