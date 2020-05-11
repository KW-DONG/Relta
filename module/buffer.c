#include "buffer.h"
#include "stdlib.h"

void Gcode_Buff_Init(gcode_list_t* gcode_buff)
{
    gcode_buff->head = NULL;
    gcode_buff->tail = gcode_buff->head;
    gcode_buff->length = 0;
}

void Gcode_Buff_Write(gcode_list_t* gcode_buff, gcode_node_t* gcode_node)
{
    gcode_node_t *p = (gcode_node_t*)malloc(sizeof(gcode_node_t));

    p->x = gcode_node->x;
    p->y = gcode_node->y;
    p->z = gcode_node->z;
    p->radius_dwell = gcode_node->radius_dwell;
    p->feedrate = gcode_node->feedrate;
    p->next = NULL;

    if(gcode_buff->length != 0)
    {
        gcode_buff->tail->next = p;
        gcode_buff->tail = p;
    }else
    {
        gcode_buff->head = p;
        gcode_buff->length ++;
    }
}

void Gcode_Buff_Read(gcode_list_t* gcode_buff,gcode_node_t* temp_node)
{
    temp_node->x = gcode_buff->head->x;
    temp_node->y = gcode_buff->head->y;
    temp_node->z = gcode_buff->head->z;
    temp_node->radius_dwell = gcode_buff->head->radius_dwell;
    temp_node->feedrate = gcode_buff->head->feedrate;

    Gcode_Buffer_Remove(gcode_buff);
}

void Gcode_Buff_Remove(gcode_list_t* gcode_buff)
{
    if(gcode_buff->length == 1) Gcode_Buffer_Init(gcode_buff);
    else
    {
        gcode_node_t* new_head = gcode_buff->head->next;
        free(gcode_buff->head);
        gcode_buff->head = new_head;
        gcode_buff->length --;
    }
    
}

void Gcode_Buff_Clear(gcode_list_t* gcode_buff)
{
    uint32_t len = gcode_buff->length;
    for (uint32_t i=0;i=len;i++)
    {
        Gcode_Buff_Remove(gcode_buff);
    }
}

void Block_Buff_Init(block_buff_t* block)
{
    block->head = NULL;
    block->tail = NULL;
    block->length = 0;
}

uint8_t Block_Buff_Write(block_t* block, block_buff_t* ring_buff)
{
    if(ring_buff->length >= RINGBUFF_LEN) return FALSE;

    ring_buff->Block_Buff[ring_buff->tail] = block;
    ring_buff->tail = (ring_buff->tail+1)%RINGBUFF_LEN;
    ring_buff->length ++;
    return TRUE;
}

uint8_t Block_Buff_Read(block_t* block, block_buff_t* ring_buff)
{
    if(ring_buff->length == 0) return FALSE;

    block = ring_buff->Block_Buff[ring_buff->head];
    ring_buff->head = (ring_buff->head+1)%RINGBUFF_LEN;
    ring_buff->length --;
}

void Block_Buff_Clear(block_buff_t* ring_buff)
{
    block_t block;
    while (ring_buff->length!=0)
    {
        Block_Buff_Read(&block, ring_buff);
    }
}

void Uart_Buff_Init(uart_buff_t* uart_buff)
{
    uart_buff->head = NULL;
    uart_buff->tail = NULL;
    uart_buff->length = 0;
}

uint8_t Uart_Buff_Write(uart_buff_t* uart_buff, uint8_t content)
{
    if(uart_buff->length >= RINGBUFF_LEN) return FALSE;

    uart_buff->content[uart_buff->head] = content;
    uart_buff->tail = (uart_buff->tail+1)%RINGBUFF_LEN;
    uart_buff->length ++;
    return TRUE;
}

uint8_t Uart_Buff_Read(uart_buff_t* uart_buff)
{
    if(uart_buff->length == 0) return FALSE;
    else return uart_buff->content[uart_buff->head];
    uart_buff->head = (uart_buff->head+1)%RINGBUFF_LEN;

    uart_buff->length --;
}

void Uart_Buff_Clear(uart_buff_t* uart_buff)
{
    while (uart_buff->length!=0)
    {
        Uart_Buff_Read(uart_buff);
    }
}

float Uart_Buff_Read_Num(uart_buff_t* uart_buff)
{
    uint8_t cha;
    uint8_t sign = 0;
    float result = 0.0f;
    cha = Uart_Buff_Read(uart_buff);
    if (cha=='-')
    {
        sign = 1;
        cha = Uart_Buff_Read(uart_buff);
    }

    for (cha;cha<='0'||cha>='9';cha = Uart_Buff_Read(uart_buff))
    {
        result = result*10.0f+Ascii(cha);
    }
    if (cha = '.')
    {
        cha = Uart_Buff_Read(uart_buff);
        for (float i = -1.0f;cha<='0'||cha>='9';cha = Uart_Buff_Read(uart_buff))
        {
            result = result + Ascii(cha)*E(i);
            i = i - 1.0f;
        }
    }
    if (sign == 1) result = -result;
    return result;
}