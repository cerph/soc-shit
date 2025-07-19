def format_hex_file(input_path, output_path):
    with open(input_path, "r") as infile:
        lines = infile.readlines()

    with open(output_path, "w") as outfile:
        for line in lines:
            if line.startswith("@"):
                outfile.write(line)
                continue
            bytes_clean = line.strip().split()
            # group bytes into words (4 bytes each)
            words = [bytes_clean[i:i+4] for i in range(0, len(bytes_clean), 4)]
            le_words = []
            for w in words:
                # reverse byte order for little endian
                w_le = w[::-1]
                le_words.append("".join(w_le))
            outfile.write(" ".join(le_words) + "\n")


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python script.py input.hex output.hex")
        sys.exit(1)
    format_hex_file(sys.argv[1], sys.argv[2])
