#define UART_BASE      0x30000000
#define UART_RBR       (*(volatile unsigned char *)(UART_BASE + 0x00))
#define UART_THR       (*(volatile unsigned char *)(UART_BASE + 0x04))
#define UART_SR        (*(volatile unsigned char *)(UART_BASE + 0x08))

#define UART_BUSY_MASK 0x01
#define UART_RXVALID   0x02

#define INTC_BASE      0x20000000
#define INTC_IRR       (*(volatile unsigned char *)(INTC_BASE + 0x00))

#define MSTATUS_MIE_BIT (1 << 3)
#define CSR_MSTATUS     0x300
#define CSR_MIE         0x304
#define CSR_MTVEC       0x305

static inline void csr_write(unsigned long reg, unsigned long val) {
    __asm__ volatile ("csrw %0, %1" :: "i"(reg), "r"(val));
}

static inline unsigned long csr_read(unsigned long reg) {
    unsigned long val;
    __asm__ volatile ("csrr %0, %1" : "=r"(val) : "i"(reg));
    return val;
}

void print(unsigned char c) {
    while (UART_SR & UART_BUSY_MASK);  // wait until not busy
    UART_THR = c;
}

// trap handler will be set at 0x0110_0000 by linker
void trap_handler(void) __attribute__((interrupt));
void trap_handler(void) {
    if ((INTC_IRR & 0x7) == 0) {  // UART interrupt ID
        if (UART_SR & UART_RXVALID) {
            unsigned char c = UART_RBR;
            print(c);
            print(c ^ 0x55);  // modify and echo
        }
    }
}

void _start(void) {
    // Setup mtvec to point to trap handler
    csr_write(CSR_MTVEC, 0x01100000);

    // Enable UART interrupt
    csr_write(CSR_MIE, (1 << 16));

    // Set global interrupt enable
    unsigned long mstatus = csr_read(CSR_MSTATUS);
    mstatus |= MSTATUS_MIE_BIT;
    csr_write(CSR_MSTATUS, mstatus);

    // Idle loop
    while (1) {
        __asm__ volatile ("wfi");
    }
}