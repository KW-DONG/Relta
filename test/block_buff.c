#include <stdio.h>
#include <stdint.h>

typedef struct 
{
    int a[3];
    int b[3];
}block_t;


typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    block_t* content[10];
}block_buff_t;

void Block_Buff_Init(block_buff_t* block)
{
    block->head = 0;
    block->tail = 0;
    block->length = 0;
}

uint8_t Block_Buff_Write(block_t block, block_buff_t* ring_buff)
{
    if(ring_buff->length >= 10) return 0;

    ring_buff->content[ring_buff->tail] = block;
    ring_buff->tail = (ring_buff->tail+1)%10;
    ring_buff->length ++;
    return 1;
}

void Block_Buff_Clear(block_buff_t* ring_buff)
{
    if (ring_buff->length!=0)
    {
        ring_buff->head = (ring_buff->head+1)%10;
        ring_buff->length--;
    }
}

int main()
{
    block_buff_t buffer;
    Block_Buff_Init(&buffer);
    char t;

    for (int i=0;i<11;i++)
    {
        block_t block;
        block.a[0] = 1;
        block.a[1] = 2;
        block.a[2] = 3;
        block.b[0] = 4;
        block.b[1] = 5;
        block.b[2] = 6;

        Block_Buff_Write(&block,&buffer);
    }

    for (int i=0;i<4;i++)
    {
        t = buffer.content[buffer.head]->a[2];
        printf("%d",t);
    }
    
}