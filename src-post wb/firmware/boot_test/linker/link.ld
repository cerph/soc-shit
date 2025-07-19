MEMORY {
    RAM        (rwxA)   : ORIGIN = 0x01000000, LENGTH = 16M    
    BOOT_ROM   (rxAI)   : ORIGIN = 0x08000000, LENGTH = 1M    
    SPI_FLASH  (rwxAI)  : ORIGIN = 0x08100000, LENGTH = 128M    
}

PROVIDE(__reset_vector = 0x08000000);
PROVIDE(__direct_trap = 0x08001000);
PROVIDE(__vector_trap_base = 0x00000000);

SECTIONS {
    .boot __reset_vector : {
        *(.text._start)
    } > BOOT_ROM

    .trap __direct_trap : {
        *(.text.direct_trap_handler)
    } > BOOT_ROM

    .text : {
        *(.text*)
        *(.rodata*)
    } > SPI_FLASH

    .data : {
        *(.data*)
    } > RAM AT > SPI_FLASH

    .bss : {
        *(.bss*)
        *(COMMON)
    } > RAM
}
