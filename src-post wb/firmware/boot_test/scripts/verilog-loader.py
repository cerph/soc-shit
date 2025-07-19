FLASH_BASE = 0x08000000

def convert_at_style_hex(input_path, output_path):
    memory = {}
    current_addr = 0

    with open(input_path, "r") as infile:
        for line in infile:
            line = line.strip()
            if line.startswith("@"):
                addr = int(line[1:], 16)
                current_addr = (addr - FLASH_BASE) // 4
            else:
                words = line.split()
                for word in words:
                    memory[current_addr] = word
                    current_addr += 1

    max_addr = max(memory.keys())
    with open(output_path, "w") as outfile:
        for addr in range(max_addr + 1):
            word = memory.get(addr, "00000000")
            outfile.write(f"{word}\n")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python convert_hex.py input.hex output.memh")
        sys.exit(1)
    convert_at_style_hex(sys.argv[1], sys.argv[2])
