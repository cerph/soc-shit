# Handles UART interrupt, reads a byte, XORs with 0x55, and transmits it back.

.section      .text._start
.global       _start

_start:                     li          sp, 0x07FFFFFF                   # begin stack

                            li          t0, 0x08001000                 
                            csrw        0x305, t0                        # mtvec[0:1] = 0 --> direct

                            li          t0, (1 << 7)                     
                            csrs        0x304, t0                        # en UART-rx irq 

                            csrsi       0x300, (1 << 3)                  # en mstatus.MIE

                            addi        a0, x0, 0x420
                            call        print
                            ebreak

idle:                       wfi
                            j           idle

.section    .text.direct_trap_handler                                  
.align      6

trap_handler:               li          t0, 0x2000                       # INTC_IRR
                            lbu         t1, 0(t0)
                            xori        t1, t1, 0x7                      # irq id == uart-rx?
                            bnez        t1, trap_exit                    

                            li          t0, 0x3000                       # UART
                            lbu         t1, 8(t0)                        # SR
                            xori        t1, t1, 0x2                      # SR[1]: rx_valid
                            bnez        t1, trap_exit                    

                            lbu         a0, 0(t0)                        # a0 = RBR
                            call        print

                            xori        a0, a0, 0x55                     
                            call        print

trap_exit:                  mret

# print(a0)
print:                      li          t0, 0x3000                       # UART
1:                          lbu         t1, 8(t0)                        # SR
                            xori        t1, t1, 0x1                      # SR[0]: busy
                            bnez        t1, 1b                           # wait if busy
                            sb          a0, 4(t0)                        # THR
                            ret
