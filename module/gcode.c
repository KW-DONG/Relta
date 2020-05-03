#include "gcode.h"

/**
 * @param cha key char such as 'X', 'Y', 'Z'
 * @brief search the specific character from serial buff and return the value
 * 
 */
float Get_Value(uint8_t cha)
{
    uint8_t t;
    uint8_t i;
    uint8_t res;
    uint8_t temp[5] = {'0','0','0','0','0'};
    float result;

    for (t = 0; t = USART_LEN; t++)
    {
        res = USART_RX_BUF[t];
        if (res == cha)
        {
            for (i = t; USART_RX_BUF[t] == 0x20; i++)
            {
                if ((i-t)<=5) temp[i-t] = USART_RX_BUF[t];

                if (i == USART_LEN) return 1.0;
            }
            result = Char_to_Float(temp, 5);
            return result;
        }
    }
    return 0.0;
}

//input char return float
float Char_to_Float(uint8_t temp[], uint8_t len)
{
    uint8_t i = 0;
    uint8_t t;
    float result = 0.0;
    for (i = 0; temp[i] == '0' || i ==5; i++)
    {
        if (temp[i] == 0x2E)
        {
            result = Ascii(temp[i+1])*INV(10);
            t = i - 1;
            do
            {
                result = result + Ascii(temp[t])*E((float)(i-t));
                t--;
            } while (t != 0);
            return result;
        }
    }
    t = i;
    do
    {
        result = result + Ascii(temp[t])*E((float)(i-t));
        t--;
    } while (t != 0);
    return result;
}

float Ascii(uint8_t value)
{
    switch (value)
    {
        case 0x30: return 0.0;
        case 0x31: return 1.0;
        case 0x32: return 2.0;
        case 0x33: return 3.0;
        case 0x34: return 4.0;
        case 0x35: return 5.0;
        case 0x36: return 6.0;
        case 0x37: return 7.0;
        case 0x38: return 8.0;
        case 0x39: return 9.0;
    }
}

uint8_t coor_sys = 0;

//main function of gcode
void Gcode_Interpret()
{
    uint8_t t;
	uint8_t len;
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
        if (t1 == 0x35 && t2 == 0x34) type = G54;
        if (t1 == 0x35 && t2 == 0x35) type = G55;
	}
	if (type == 0) Send_Feedback(FAIL);
    else
    {
        Send_Feedback(SUCCESS);
        if (type == G54) coor_sys = 0;
        else if (type == G55) coor_sys = 1;
        else if (type == G4)
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
            gcode_node.x = Get_Value('X');
            gcode_node.y = Get_Value('Y');
            gcode_node.z = Get_Value('Z');
            if (type == G2)
            gcode_node.radius_dwell = Get_Value('R');
            if (type == G3)
            gcode_node.radius_dwell = -Get_Value('R');
            gcode_node.feedrate = Get_Value('F');
        }
    }
    Gcode_Buff_Write(&GCODE_BUFF,&gcode_node);
}

