AS = riscv32-unknown-elf-as
LD = riscv32-unknown-elf-ld
OBJCOPY = riscv32-unknown-elf-objcopy

ASFLAGS = -march=rv32i_zicsr
LDFLAGS = -T linker/link.ld
GROUP_SCRIPT = scripts/regroup_bytes.py
VERILOG_LOADER = scripts/verilog-loader.py

SRC = src/boot_test.s
ELF = build/boot_test.elf
FINAL = flash.memh

all: $(FINAL)

$(FINAL): $(SRC) linker/link.ld $(GROUP_SCRIPT)
	@mkdir -p build
	$(AS) $(ASFLAGS) -o build/tmp.o $(SRC)
	$(LD) $(LDFLAGS) -o $(ELF) build/tmp.o
	$(OBJCOPY) -O verilog $(ELF) build/tmp.hex
	python3 $(GROUP_SCRIPT) build/tmp.hex build/tmp.hex
	python3 $(VERILOG_LOADER) build/tmp.hex $(FINAL)

	rm build/tmp.o build/tmp.hex 

clean:
	rm -rf build

