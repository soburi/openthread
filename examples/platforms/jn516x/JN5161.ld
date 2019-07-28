g_u8HWConf_Flash = 0x1;
g_u8HWConf_Ram = 0x0;
g_u16HWConf_Chip = 0x0008;
g_u16DefaultMinimumHeap = 1500;

MEMORY
{
        flash  : ORIGIN = 0x00080000, LENGTH = 0x10000
        ram    : ORIGIN = 0x0400004c, LENGTH = 0x01fb4
}

INCLUDE JN516x.ld
