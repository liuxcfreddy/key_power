#include "Bin.h"

int BIN(bit a,bit b,bit c,bit d,bit e,bit f,bit g,bit h)
{
    unsigned int path=0;
    path+=0x01*h;
    path+=0x02*g;
    path+=0x04*f;
    path+=0x08*e;

    path+=0x10*d;
    path+=0x20*c;
    path+=0x40*b;
    path+=0x80*a

}