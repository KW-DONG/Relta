#include "buffer.h"
#include "type.h"

//gcode_linked_list GCODE_BUFFER;

void Gcode_Buff_Init(linked_list_t* GCODE_BUFFER)
{
    GCODE_BUFFER->head = NULL;
    GCODE_BUFFER->tail = GCODE_BUFFER->head;
    GCODE_BUFFER->length = 0;
}

void Gcode_Buff_Write(linked_list_t* GCODE_BUFFER, gcode_node_t* gcode_node)
{
    gcode_node_t *p = (gcode_node_t*)malloc(sizeof(gcode_node_t));

    p->x = gcode_node->x;
    p->y = gcode_node->y;
    p->z = gcode_node->z;
    p->radius_dwell = gcode_node->radius_dwell;
    p->feedrate = gcode_node->feedrate;
    p->next = NULL;

    if(GCODE_BUFFER->length != 0)
    {
        GCODE_BUFFER->tail->next = p;
        GCODE_BUFFER->tail = p;
    }else
    {
        GCODE_BUFFER->head = p;
        GCODE_BUFFER->length ++;
    }
}

void Gcode_Buff_Read(linked_list_t* GCODE_BUFFER,
                        gcode_node_t* temp_node)
{
    temp_node->x = GCODE_BUFFER->head->x;
    temp_node->y = GCODE_BUFFER->head->y;
    temp_node->z = GCODE_BUFFER->head->z;
    temp_node->radius_dwell = GCODE_BUFFER->head->radius_dwell;
    temp_node->feedrate = GCODE_BUFFER->head->feedrate;

    Gcode_Buffer_Remove(GCODE_BUFFER);
}

void Gcode_Buff_Remove(linked_list_t* GCODE_BUFFER)
{
    if(GCODE_BUFFER->length == 1) Gcode_Buffer_Init(GCODE_BUFFER);
    else
    {
        gcode_node_t* new_head = GCODE_BUFFER->head->next;
        free(GCODE_BUFFER->head);
        GCODE_BUFFER->head = new_head;
        GCODE_BUFFER->length --;
    }
    
}

void Gcode_Buff_Clear(linked_list_t* GCODE_BUFFER)
{
    uint32_t len = GCODE_BUFFER->length;
    for (uint32_t i=0;i=len;i++)
    {
        Gcode_Buff_Remove(GCODE_BUFFER);
    }
}


//ring buffer
void Ring_Buff_Init(ring_buff_t* block)
{
    block->head = NULL;
    block->tail = NULL;
    block->length = 0;
}

uint8_t Ring_Buff_Write(stepper_exe_t block, ring_buff_t* ring_buff)
{
    if(ring_buff->length >= RINGBUFF_LEN) return FALSE;

    ring_buff->Ring_Buff[ring_buff->tail] = block;
    ring_buff->tail = (ring_buff->tail+1)%RINGBUFF_LEN;
    ring_buff->length ++;
    return TRUE;
}

uint8_t Ring_Buff_Read(stepper_exe_t* block, ring_buff_t* ring_buff)
{
    if(ring_buff->length == 0) return FALSE;

    *block = ring_buff->Ring_Buff[ring_buff->head];
    ring_buff->head = (ring_buff->head+1)%RINGBUFF_LEN;
    ring_buff->length --;
}

void Ring_Buff_Clear(ring_buff_t* ring_buff)
{
    stepper_exe_t block;
    while (ring_buff->length!=0)
    {
        Ring_Buff_Read(&block, ring_buff);
    }
}