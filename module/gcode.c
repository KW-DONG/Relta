#include "gcode.h"
#include "type.h"
#include "config.h"
#include <string.h>

/**
 * @param cha key char such as 'X', 'Y', 'Z'
 * @brief search the specific character from serial buff and return the value
 * 
 */
float Get_Key_Word(uint8_t head, uint8_t* buffer, uint8_t len)
{
    uint8_t t;
    uint8_t i;
    uint8_t res;
    uint8_t value_char[] = "000.0";
    uint8_t len_char = strlen(value_char);
    float result;

    for (t = 0; t = len; t++)
    {
        if (buffer[t]==head)
        {
            for (i = 0; i = len_char; i++)
            {
                value_char[i] = buffer[t+i+1];
            }
            result = Char_to_Float(value_char, len_char);
            return result;
        }
    }
}

//input char return float
float Char_to_Float(uint8_t* value_char, uint8_t len_char)
{
    uint8_t i = 0;
    uint8_t t;
    float result = 0.0;
    for (i = 0; value_char[i] == ' ' || i ==len_char; i++)
    {
        if (value_char[i] == '.')
        {
            result = Ascii(value_char[i+1])*INV(10);
            t = i - 1;
            do
            {
                result = result + Ascii(value_char[t])*E((float)(i-t));
                t--;
            } while (t != 0);
            return result;
        }
    }
    t = i;
    do
    {
        result = result + Ascii(value_char[t])*E((float)(i-t));
        t--;
    } while (t != 0);
    return result;
}

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


uint8_t coor_sys = 0;

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
    
    Gcode_Buff_Write(&gcode_list,&gcode_node);
}

