
#include <stdio.h>

extern int k_strlen(const char* str);
extern void* k_memcpy(void* dst, const void* src, int len);
extern int k_memcmp(const void* b1, const void* b2, int len);

#define TEST_OK 0

int test_strlen_1();
int test_memcpy_1();
int test_memcpy_2();
int test_memcmp_1();
int test_memcmp_2();

#define RUN_TEST(t) \
{ \
	int result = t();			\
	if (result != TEST_OK) {			\
		printf("test %s failed\n", #t);		\
		return (result);			\
	}						\
}

int main()
{
	RUN_TEST(test_strlen_1);
	RUN_TEST(test_memcpy_1);
	RUN_TEST(test_memcpy_2);
	RUN_TEST(test_memcmp_1);
	RUN_TEST(test_memcmp_2);

	printf("All tests passed!\n");
	return (0);
}


int test_strlen_1()
{
	char str1[] = "";
	char str2[] = "abcdefghij";
	char str3[] = "0XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";

	if (k_strlen(str1) != 0)
		return (1);

	if (k_strlen(str2) != 10)
		return (2);

	if (k_strlen(str3) != 50)
		return (3);

	return (TEST_OK);
}

int test_memcpy_1()
{
	char src[] = { 'a', 'b', 'c', 'd' };
	char dst[] = { 'x', 'x', 'x', 'x' };

	k_memcpy(dst, src, 4);
	if (dst[0] != 'a' || dst[1] != 'b' || dst[2] != 'c' || dst[3] != 'd')
		return (1);
	
	if (src[0] != 'a' || src[1] != 'b' || src[2] != 'c' || src[3] != 'd')
		return (2);

	return (TEST_OK);
}

int test_memcpy_2()
{
	char src[] = { 'a', 'b', 'c', 'd' };
	char dst[] = { 'x', 'x', 'x', 'x' };
	k_memcpy(dst, src, 0);

	if (dst[0] != 'x' || src[0] != 'a')
		return (1);

	return (TEST_OK);
}

int test_memcmp_1()
{
	char a1[] = { 'a', 'b', 'c', 'd' };
	char a2[] = { 'a', 'b', 'c', 'd' };
	int l;

	for (l=0; l<=4; l++) {
		if (k_memcmp(a1, a2, l) != 0)
			return (1);
	}

	return (TEST_OK);
}

int test_memcmp_2()
{
	char a1[] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h' };
	char a2[] = { 'a', 'b', 'c', 'd', 'e', 'f', 'e', 'h' };
	int l;

	for (l=0; l<= 6; l++) {
		if (k_memcmp(a1, a2, l) != 0)
			return (1);
	}

	if (k_memcmp(a1, a2, 8) != ('g' - 'e'))
		return (2);

	if (k_memcmp(a2, a1, 8) != ('e' - 'g'))
		return (3);

	return (TEST_OK);
}

