#include <stdlib.h>
#include <stdio.h>
int main(){
    int *ptr;
    ptr = malloc(700 * sizeof(*ptr));
    if (ptr != NULL) {
      *(ptr + 500) = 480; 
      printf("Value of the 500th integer is %d\n",*(ptr + 500));
      *(ptr + 800) = 480; 
      printf("Value of the 800th integer is %d\n",*(ptr + 800));
    }
}
