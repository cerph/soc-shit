# Handles UART interrupt, reads a byte, XORs with 0x55, and transmits it back.

.section      .text._start
.global       _start

_start:                     nop
                            nop
                            addi t0, x0, 0xA
                            nop
                            nop
                            addi t1, t0, 0x1
                            nop
                            nop
                            csrw mtvec, t1
                            nop
                            nop
                            ebreak