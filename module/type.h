#ifndef __TYPE_H
#define __TYPE_H
#include <math.h>
#include <stdint.h>

#undef  SQ(x)
#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f

#define RINGBUFF_LEN 200
#define LINKED_LIST_LEN 200

#define TRUE 1
#define FALSE 0

#define FULL    2
#define HALF    4
#define QURT    8
#define EIGH    16


/**
 * radius>0 G2
 * radius<0 G3
 * xyz!=0 && radius=0 && feedrate!=0 G1
 * xyz!=0 && radius=0 && feedrate==0 G0
 * 
 * radius_dwell depend on the type
 */

typedef struct _node
{
    float   x,y,z,
            feedrate,
            radius_dwell;
    uint8_t coordinate_sys;
    uint8_t type;
    struct _node* next;
}gcode_node_t;

typedef struct 
{
    gcode_node_t*       head;
    gcode_node_t*       tail;
    uint32_t            length;
}gcode_list_t;

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    stepper_exe_t Block_Buff[RINGBUFF_LEN];
}block_buff_t;

typedef struct 
{
    uint16_t    head;
    uint16_t    tail;
    uint16_t    length;
    uint8_t     content[100];
}uart_buff_t;


typedef struct 
{
    uint8_t     dir[3];
    int32_t    freq[3];//pps
    uint16_t    step[3];
    uint32_t    step_dwell;//calculated with monitor frequency
}stepper_exe_t;

typedef struct 
{
    float   xyz[3];
    float   abc[3];
    float   xyz_v[3];
    float   feedrate;
    uint8_t state;
}machine_t;

enum machine_state
{
    ON,
    OFF,
    RESET,
    ERROR
};

enum compile_mode
{
    NO,
    YES
};



#endif