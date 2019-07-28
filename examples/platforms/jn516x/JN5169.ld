g_u8HWConf_Flash = 0xf;
g_u8HWConf_Ram = 0x3;
g_u16HWConf_Chip = 0x000b;
g_u16DefaultMinimumHeap = 3350;

MEMORY
{
        flash  : ORIGIN = 0x00080000, LENGTH = 0x80000
        ram    : ORIGIN = 0x0400004c, LENGTH = 0x07fb4
}

INCLUDE JN516x.ld
