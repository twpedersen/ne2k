
/*
 * This program can be written in C++ since it is only a helper
 * program and its code will not be linked with the TOS kernel.
 */

#include <iostream>
#include <cmath>

using namespace std;

int main()
{
    cout << "static int table[256] = {" << endl;
    int j = 0;
    for (int i=0; i <= 8; i++) {
	for ( ; j < (int) pow ((float) 2, (float) i); j++) {
	    if (j != 0)
		cout << ", ";
	    cout << i - 1;
	}
    }
    cout << "};" << endl;
    return 0;
}
