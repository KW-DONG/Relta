#include <stdio.h>

int max(int x, int y)
{
    int z;
    z = (x>y)?x:y;
    return(z);
}

void main()
{
    int a=1,b=2,c;
    c=max(a,b);
    c = 1;

}