#include <stdio.h>
#include <stdint.h>

#define LEN 10

volatile int32_t content[LEN];
volatile uint8_t head;
volatile uint8_t tail;
volatile uint8_t length;

void init(void)
{
    head = 0;
    tail = 0;
    length = 0;
}

uint8_t write(int32_t value)
{
    if (length>=LEN) return 1;
    content[tail] = value;
    tail = (tail+1)%LEN;
    length ++;
    return 0;
}

uint8_t clear(void)
{
    if (length==0)  return 1;
    length--;
    head = (head+1)%LEN;
}

int main(void)
{
    printf("head:%d; tail:%d; length:%d\n", head, tail, length);

    for (uint8_t i=0;i<20;i++)
    {
        write(2);
        printf("head:%d; tail:%d; length:%d\n", head, tail, length);
    }
    for (uint8_t i=0;i<20;i++)
    {
        clear();
        printf("head:%d; tail:%d; length:%d\n", head, tail, length);
    }
    return 0;
}