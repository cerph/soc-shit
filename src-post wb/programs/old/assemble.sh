#!/bin/bash
set -e

AS=riscv32-unknown-elf-as
LD=riscv32-unknown-elf-ld
OBJCOPY=riscv32-unknown-elf-objcopy
GROUP_SCRIPT=/usr/local/bin/group4bytes.py

ASFLAGS="-march=rv32i_zicsr"
LDFLAGS="-T ../link.ld"

$AS $ASFLAGS -o uart_test.o uart_test.s
$LD $LDFLAGS -o uart_test.elf uart_test.o
$OBJCOPY -O verilog uart_test.elf uart_test.hex
python3 $GROUP_SCRIPT uart_test.hex flash.hex
