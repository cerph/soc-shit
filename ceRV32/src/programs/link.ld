SECTIONS
{
    . = 0x01000000;
    .text : {
        *(.text._start)
    }

    . = 0x01100000;
    .trap : {
        *(.text.trap_handler)
    }

    .text.rest : {
        *(.text*)
        *(.rodata*)
    }
    
    .data : {
        *(.data*)
    }

    .bss : {
        *(.bss*)
        *(COMMON)
    }
}