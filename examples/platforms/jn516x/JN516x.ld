exception_vsr_table = 0x00080038;
_exception_vsr_table = 0x00080038;
pSPIflashFncTable = 0x04000000;
restore_state = 0x00000044;
bZSPIWaitWIPClear = 0x0000004a;
u16ZSPIflashRDID = 0x00000050;
u8SelectFlashChipType = 0x00000056;
memcpy = 0x0000005c;
vBootFlashProg = 0x00000066;

EXTERN(AppWarmStart)
EXTERN(AppColdStart)
EXTERN(u32HeapStart);
EXTERN(vException_BusError)
EXTERN(vException_UnalignedAccess)
EXTERN(vException_IllegalInstruction)
EXTERN(vException_SysCall)
EXTERN(vException_Trap)
EXTERN(vException_StackOverflow)
EXTERN(_exception_external_interrupt)
PROVIDE(vException_BusError           = restore_state);
PROVIDE(vException_UnalignedAccess    = restore_state);
PROVIDE(vException_IllegalInstruction = restore_state);
PROVIDE(vException_SysCall            = restore_state);
PROVIDE(vException_Trap               = restore_state);
PROVIDE(vException_StackOverflow      = restore_state);

/* Set minimum heap size unless it is already set from App_Stack_Size.ld */
_minimum_heap_size = DEFINED(_minimum_heap_size) ? _minimum_heap_size : g_u16DefaultMinimumHeap;
__minimum_heap_size = _minimum_heap_size;
_stack_size = DEFINED(_stack_size) ? _stack_size : 2048;
__stack_size = _stack_size;

_SwConfig =  (DEFINED(g_bSWConf_Debug)) | (DEFINED(g_bSWConf_AltDebugPort)<<1);
_HwConfig =  ((g_u8HWConf_Flash<<24) | (g_u8HWConf_Ram << 16) | (g_u16HWConf_Chip));

/* Now building all code at once, so include vectors. MAC address is embedded
   in build for now */
SECTIONS
{
  .version ABSOLUTE(ORIGIN(flash)-4):
  {
    LONG(_HwConfig)
  } > flash 
				       
  .bir ABSOLUTE(ORIGIN(flash)):
  {
    _flash_start = ABSOLUTE(.);
    _flash_beg = ABSOLUTE(.);
    /* Magic number */
    LONG(0x12345678)
    LONG(0x11223344)
    LONG(0x55667788)
        
    /* Configuration A (32K, DRE, 16MHz) */
    BYTE(0x08)

    /* Configuration B (version 1) */
    BYTE(0x01)
    
    /* Application ID */
    SHORT(0x0000)
  } > flash
        
  .flashheader :
    {
    /* Encryption IV */
    LONG(0x00000000)
    LONG(0x00000000)
    LONG(0x00000000)
    SHORT(0x0000)

        /*SW Config*/
        /*
        Bit
        0    Debug En (1 Enable 0 Disable)
        1    0 - JTAG Debug on DIO4-7
             1 - JTAG Debug on DIO12-15
        2-15 Unused
		    */
    SHORT(ABSOLUTE(_SwConfig))

    _enc_start = ABSOLUTE(.);
    /*Flash Image Length*/
    LONG(ABSOLUTE(_image_len))
        
    /* Data start & length */
    LONG(ABSOLUTE(_flash_data_start))
    SHORT((ABSOLUTE(_ram_data_start) / 4) & 0xffff)
    SHORT((ABSOLUTE(_ram_data_len)   / 4) & 0xffff)

    /* BSS start & length */
    SHORT((ABSOLUTE(_bss_start) / 4) & 0xffff)
    SHORT((ABSOLUTE(_bss_len)   / 4) & 0xffff)

    /* Entry points */
    LONG(ABSOLUTE(AppWarmStart))
    LONG(ABSOLUTE(AppColdStart))
  } > flash
  _enc_offset = (_enc_start & 0x0000000F);


  . = ABSOLUTE(_exception_vsr_table);
        
  .vsr_table :
  {
    LONG(ABSOLUTE(vException_BusError))
    LONG(ABSOLUTE(vException_UnalignedAccess))
    LONG(ABSOLUTE(vException_IllegalInstruction))
    LONG(ABSOLUTE(vException_SysCall))
    LONG(ABSOLUTE(vException_Trap))
    LONG(ABSOLUTE(vException_StackOverflow))
  } > flash

  .vsr_handlers : { KEEP(*(.vsr_handlers)) } > flash

  .vectors        : { *(.vectors) } > flash
  .note.gnu.build-id : { *(.note.gnu.build-id) } > flash
  .hash           : { *(.hash) } > flash
  .gnu.hash       : { *(.gnu.hash) } > flash
  .dynsym         : { *(.dynsym) } > flash
  .dynstr         : { *(.dynstr) } > flash
  .gnu.version    : { *(.gnu.version) } > flash
  .gnu.version_d  : { *(.gnu.version_d) } > flash
  .gnu.version_r  : { *(.gnu.version_r) } > flash
  .rel.dyn        :
    {
      *(.rel.init)
      *(.rel.text .rel.text.* .rel.gnu.linkonce.t.*)
      *(.rel.fini)
      *(.rel.rodata .rel.rodata.* .rel.gnu.linkonce.r.*)
      *(.rel.data.rel.ro* .rel.gnu.linkonce.d.rel.ro.*)
      *(.rel.data .rel.data.* .rel.gnu.linkonce.d.*)
      *(.rel.tdata .rel.tdata.* .rel.gnu.linkonce.td.*)
      *(.rel.tbss .rel.tbss.* .rel.gnu.linkonce.tb.*)
      *(.rel.ctors)
      *(.rel.dtors)
      *(.rel.got)
      *(.rel.sdata .rel.sdata.* .rel.gnu.linkonce.s.*)
      *(.rel.sbss .rel.sbss.* .rel.gnu.linkonce.sb.*)
      *(.rel.sdata2 .rel.sdata2.* .rel.gnu.linkonce.s2.*)
      *(.rel.sbss2 .rel.sbss2.* .rel.gnu.linkonce.sb2.*)
      *(.rel.bss .rel.bss.* .rel.gnu.linkonce.b.*)
      PROVIDE_HIDDEN (__rel_iplt_start = .);
      *(.rel.iplt)
      PROVIDE_HIDDEN (__rel_iplt_end = .);
      PROVIDE_HIDDEN (__rela_iplt_start = .);
      PROVIDE_HIDDEN (__rela_iplt_end = .);
    } > flash
  .rela.dyn       :
    {
      *(.rela.init)
      *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*)
      *(.rela.fini)
      *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*)
      *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*)
      *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*)
      *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*)
      *(.rela.ctors)
      *(.rela.dtors)
      *(.rela.got)
      *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*)
      *(.rela.sbss .rela.sbss.* .rela.gnu.linkonce.sb.*)
      *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*)
      *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*)
      *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*)
      PROVIDE_HIDDEN (__rel_iplt_start = .);
      PROVIDE_HIDDEN (__rel_iplt_end = .);
      PROVIDE_HIDDEN (__rela_iplt_start = .);
      *(.rela.iplt)
      PROVIDE_HIDDEN (__rela_iplt_end = .);
    } > flash
  .rel.plt        :
    {
      *(.rel.plt)
    } > flash
  .rela.plt       :
    {
      *(.rela.plt)
    } > flash
  .init           :
  {
    KEEP (*(.init))
  } > flash =0
  .plt            : { *(.plt) } > flash
  .iplt           : { *(.iplt) } > flash
  .fini           :
  {
    KEEP (*(.fini))
  } > flash =0
  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);
  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) } > flash
  .rodata1        : { *(.rodata1) } > flash
  .text           :
  {
    *(.text.unlikely .text.*_unlikely)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(.text .stub .text.* .gnu.linkonce.t.*)
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
  } > flash =0

  .sdata2         :
  {
    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
  } > flash
  .sbss2          : { *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*) } > flash
  .eh_frame_hdr : { *(.eh_frame_hdr) } > flash
/*
  .eh_frame       : ONLY_IF_RO { KEEP (*(.eh_frame)) }
  .gcc_except_table   : ONLY_IF_RO { *(.gcc_except_table
  .gcc_except_table.*) }
*/
  /* These sections are generated by the Sun/Oracle C++ compiler.  */
/*
  .exception_ranges   : ONLY_IF_RO { *(.exception_ranges
  .exception_ranges*) }
*/

  _flash_end = ABSOLUTE(.);

/*
  . = DATA_SEGMENT_RELRO_END (0, .);
*/
  .data           : AT ((LOADADDR(.text) + SIZEOF(.text) + 3) & ~ 3)
  {
    _ram_data_start = ABSOLUTE(.);
    *(.data .data.* .gnu.linkonce.d.*)
    SORT(CONSTRUCTORS)
    . = ALIGN (4);
    _ram_data_end = ABSOLUTE(.);
    . = ALIGN (4);
  } > ram

  /* We want the small data sections together, so single-instruction offsets
     can access them all, and initialized data all before uninitialized, so
     we can shorten the on-disk segment size.  */
  .sdata          : AT ((LOADADDR(.data) + SIZEOF(.data) + 3) & ~ 3)
  {
    *(.sdata .sdata.* .gnu.linkonce.s.*)
  } > ram
  _edata = .; PROVIDE (edata = .);
  /* Adjust the address for the data segment.  We want to adjust up to
     the same address within the page on the next page up.  */
/*
  . = ALIGN (CONSTANT (MAXPAGESIZE)) - ((CONSTANT (MAXPAGESIZE) - .) & (CONSTANT (MAXPAGESIZE) - 1)); . = DATA_SEGMENT_ALIGN (CONSTANT (MAXPAGESIZE), CONSTANT (COMMONPAGESIZE));
*/
  /* Exception handling  */
  .eh_frame       : AT ((LOADADDR(.sdata) + SIZEOF(.sdata) + 3) & ~ 3) { KEEP (*(.eh_frame)) } >flash
  .gcc_except_table   : AT ((LOADADDR(.eh_frame) + SIZEOF(.eh_frame) + 3) & ~ 3) { *(.gcc_except_table .gcc_except_table.*) }
  .exception_ranges   : AT ((LOADADDR(.gcc_except_table) + SIZEOF(.gcc_except_table) + 3) & ~ 3) { *(.exception_ranges .exception_ranges*) }
  /* Thread Local Storage sections  */
  .tdata	  : AT ((LOADADDR(.exception_ranges) + SIZEOF(.exception_ranges) + 3) & ~ 3) { *(.tdata .tdata.* .gnu.linkonce.td.*) }
  .tbss		  : AT ((LOADADDR(.tdata) + SIZEOF(.tdata) + 3) & ~ 3) { *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon) }
  .preinit_array     : AT ((LOADADDR(.tbss) + SIZEOF(.tbss) + 3) & ~ 3)
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > ram
  .init_array     : AT ((LOADADDR(.preinit_array) + SIZEOF(.preinit_array) + 3) & ~ 3)
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > ram
  .fini_array     : AT ((LOADADDR(.init_array) + SIZEOF(.init_array) + 3) & ~ 3)
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > ram

  . = ALIGN(4);

  .ctors          : AT ((LOADADDR(.fini_array) + SIZEOF(.fini_array) + 3) & ~ 3)
  {
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
  } > ram
  .end_ctors          : AT ((LOADADDR(.ctors) + SIZEOF(.ctors) + 3) & ~ 3)
  {
    LONG(0x0) /* TERMINATOR */
    LONG( SIZEOF(.ctors) )
  } > ram
  .dtors          : AT ((LOADADDR(.end_ctors) + SIZEOF(.end_ctors) + 3) & ~ 3)
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } > ram
  .end_dtors          : AT ((LOADADDR(.dtors) + SIZEOF(.dtors) + 3) & ~ 3)
  {
    LONG(0x0) /* TERMINATOR */
    LONG( SIZEOF(.dtors) )
  } > ram
  .jcr            : { KEEP (*(.jcr)) } > ram
  .data.rel.ro : { *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro* .gnu.linkonce.d.rel.ro.*) } > ram
  .dynamic        : { *(.dynamic) } > ram

  . = ALIGN(4);
  __bss_start = .;
  .sbss           :
  {
    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
  } > ram
  .bss            :
  {
   *(.dynbss)
   *(.bss .bss.* .gnu.linkonce.b.*)
   *(COMMON)
   /* Align here to ensure that the .bss section occupies space up to
      _end.  Align after .bss to ensure correct alignment even if the
      .bss section disappears because there are no input sections.
      FIXME: Why do we need it? When there is no .bss section, we don't
      pad the .data section.  */
   . = ALIGN(. != 0 ? 32 / 8 : 1);
  } > ram
  . = ALIGN(32 / 8);
  . = ALIGN(32 / 8);
  _end = .; PROVIDE (end = .);
/*
  . = DATA_SEGMENT_END (.);
*/
  /* reserve minimum heap size */
  .heap ALIGN (0x4) :
  {
    __heap_start = ABSOLUTE(.);
    . += _minimum_heap_size;
  } > ram

  .stack ABSOLUTE (ORIGIN(ram) + LENGTH(ram) - _stack_size - 0x100):
  {
    /* make end of heap the bottom of the stack */
    __heap_end = ABSOLUTE(.);
    _stack_low_water_mark = ABSOLUTE(.);
    . += _stack_size;
  } > ram

  _stack = .;

  /* Test to check if the .data initialisation values in flash has gone
     past the end of the available space; the linker doesn't throw an
     error if that has happened. The following forces a byte to be
     stored after the .data initialisation values, but it is marked as
     NOLOAD so it never gets into the binary */
  .test (((LOADADDR(.text) + SIZEOF(.text) + 3) & ~ 3) + SIZEOF(.data)) (NOLOAD):
  {
    BYTE(0xff)
  } > flash

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}

_bss_start = __bss_start;
_flash_data_start = LOADADDR(.data);
_ram_data_len = (_ram_data_end - _ram_data_start + 3) & ~ 3;
_bss_len      = (_end      - _bss_start      + 3) & ~ 3;
_image_len    = (_flash_end - _flash_start + _ram_data_len + 3) & ~ 3;
_heap_start   = __heap_start;
_heap_end     = __heap_end;
_heap_limit   = __heap_end;
_ram_top      = ABSOLUTE(ORIGIN(ram)) + LENGTH(ram);

ctors_start  = ABSOLUTE(LOADADDR(.ctors) );

__app_ram_start = 0x01001580; /* Page 5, word 8 (16-byte words, 16 words per page) */
PROVIDE(heap_location  = u32HeapStart);
