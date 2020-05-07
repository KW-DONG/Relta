#include "gcode.h"
#include "type.h"
#include "config.h"
#include <string.h>

/**
 * @param cha key char such as 'X', 'Y', 'Z'
 * @brief search the specific character from serial buff and return the value
 * 
 */
float Get_Value(uint8_t head, uint8_t* buffer, uint8_t len)
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
void Gcode_Interpret(uint8_t* buffer, uint8_t len)
{
    uint8_t t;
    uint8_t type = 0;
    uint8_t res;
    gcode_node_t gcode_node;

    len=USART_RX_STA&0x3fff;

    if (USART_RX_BUF[t] == 0x47)//"G"
	{
		uint8_t t1, t2;
        t1 = USART_RX_BUF[t+1];
        t2 = USART_RX_BUF[t+2];

        if (t1 == 0x30 && t2 == 0x20) type = G0;
        if (t1 == 0x31 && t2 == 0x20) type = G1;
        if (t1 == 0x32 && t2 == 0x20) type = G2;
        if (t1 == 0x32 && t2 == 0x38) type = G28;
        if (t1 == 0x33 && t2 == 0x20) type = G3;
        if (t1 == 0x34 && t2 == 0x20) type = G4;
	}
	if (type == 0) Send_Feedback(FAIL);
    else
    {
        Send_Feedback(SUCCESS);
        if (type == G4)
        {
            float dwell = Get_Value('P');
            if (dwell != 0.0) gcode_node.radius_dwell = dwell;
            else gcode_node.radius_dwell = -Get_Value('S');
            gcode_node.type = dwell_t;
        } 
        else if (type == G28)
        {
            gcode_node.type = home_t;

        }
        else{
            gcode_node.x = Get_Value('X', buffer, len);
            gcode_node.y = Get_Value('Y', buffer, len);
            gcode_node.z = Get_Value('Z', buffer, len);
            if (type == G2)
            gcode_node.radius_dwell = Get_Value('R', buffer, len);
            if (type == G3)
            gcode_node.radius_dwell = -Get_Value('R', buffer, len);
            gcode_node.feedrate = Get_Value('F', buffer, len);
        }
    }
    Gcode_Buff_Write(&GCODE_BUFF,&gcode_node);
}

