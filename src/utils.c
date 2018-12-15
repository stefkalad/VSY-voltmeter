/*
 * utils.c
 *
 *  Created on: Dec 15, 2018
 *      Author: ladislav
 */

// A simple atoi() function
int myAtoi(char *str)
{
    int res = 0; // Initialize result

    // Iterate through all characters of input string and
    // update result
    for (int i = 0; str[i] != '\0'; ++i){
        res = res*10 + str[i] - '0';
        if (0 > str[i] - '0' || str[i] -'0' > 9)
        	return -1;

    }

    return res;
}

