# UART test program
# Handles UART interrupt, reads a byte, XORs with 0x55, and transmits it back.

.section      .text._start
.global       _start

_start:                     li          sp, 0x10010000                   # Safe stack

                            li          t0, 0x01100000
                            csrw        0x305, t0                        # mtvec = 0x0110_0000

                            li          t0, (1 << 16)                    # Enable UART interrupt
                            csrw        0x304, t0                        # mie = mie | UART bit

                            csrsi       0x300, (1 << 3)                  # mstatus.MIE = 1

idle:                       wfi
                            j           idle

.section      .text.trap_handler
.align        6

trap_handler:               li          t0, 0x20000000                   # IRR base
                            lbu         t1, 0(t0)
                            andi        t1, t1, 0x7                      # IRR[2: 0]
                            bnez        t1, trap_exit                    # If not UART interrupt, exit

                            li          t0, 0x30000008                   # UART_SR
                            lbu         t1, 0(t0)
                            andi        t1, t1, 0x2                      # UART_SR[1] = rx_valid
                            beqz        t1, trap_exit

                            li          t0, 0x30000000                   # UART_RBR
                            lbu         a0, 0(t0)                        # a0 = rx byte

                            call        print

                            xori        a0, a0, 0x55                     # modify = rx ^ 0x55
                            call        print

trap_exit:                  mret

# print(a0)
print:                      li          t1, 0x30000008                   # UART_SR
1:                          lbu         t2, 0(t1)
                            andi        t2, t2, 0x1                      # UART_SR[0] = uart_busy
                            bnez        t2, 1b                           # wait while busy

                            li          t1, 0x30000004                   # UART_THR
                            sb          a0, 0(t1)                        # transmit
                            ret
