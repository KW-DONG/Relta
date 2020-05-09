#include "gcode.h"
#include "type.h"
#include "config.h"
#include <string.h>

float Ascii(uint8_t value)
{
    switch (value)
    {
        case 0x30: return 0.0f;
        case 0x31: return 1.0f;
        case 0x32: return 2.0f;
        case 0x33: return 3.0f;
        case 0x34: return 4.0f;
        case 0x35: return 5.0f;
        case 0x36: return 6.0f;
        case 0x37: return 7.0f;
        case 0x38: return 8.0f;
        case 0x39: return 9.0f;
    }
}

//main function of gcode
void Gcode_Interpret(gcode_list_t* gcode_list, uart_buff_t* uart_buff)
{

    uint8_t type=0;
    uint8_t key;
    gcode_node_t gcode_node;

    key = Uart_Buff_Read(uart_buff);

    for(key;key==0x0d||uart_buff->length==0;key=Uart_Buff_Read(uart_buff))
    {
        if (key=='G')
        {
            key = Uart_Buff_Read(uart_buff);
            if (key=='0')       type = G0;
            else if (key=='1')  type = G1;
            else if (key=='2')
            {
                key = Uart_Buff_Read(uart_buff);
                if (key=='8')   type = G28;
                else            type = G2;
            }else if (key=='3') type = G3;
            else if (key=='4')  type = G4;
        }else if (key=='X') gcode_node.x = Uart_Buff_Read_Num(uart_buff);
        else if (key=='Y')  gcode_node.y = Uart_Buff_Read_Num(uart_buff);
        else if (key=='Z')  gcode_node.z = Uart_Buff_Read_Num(uart_buff);
        else if (key=='R'||key=='P')  gcode_node.radius_dwell = Uart_Buff_Read_Num(uart_buff);
        else if (key=='F')  gcode_node.feedrate = Uart_Buff_Read_Num(uart_buff);
    }
    if (type==G3)   gcode_node.radius_dwell = -gcode_node.radius_dwell;
    Gcode_Buff_Write(&gcode_list,&gcode_node);

}

