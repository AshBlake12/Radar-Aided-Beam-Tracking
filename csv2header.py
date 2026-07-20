# csv2header.py - frames.csv -> frames_data.h
import sys

if len(sys.argv) < 2:
    print("Error: Missing input file. Usage: python csv2header.py frames.csv")
    sys.exit(1)

# Read the CSV
with open(sys.argv[1], 'r', encoding='utf-8') as f:
    rows = [l.strip().split(',') for l in f if l.strip()]

nc = len(rows[0]) // 2

# Write directly to the file in UTF-8
with open('frames_data.h', 'w', encoding='utf-8') as out:
    out.write(f"#ifndef FRAMES_DATA_H\n#define FRAMES_DATA_H\n\n")
    out.write(f"#define NFRAMES {len(rows)}\n")
    out.write(f"#define NC {nc}\n\n")
    out.write(f"static const double frames[NFRAMES][2*NC]={{\n")
    for r in rows:
        out.write("    {" + ",".join(r) + "},\n")
    out.write("};\n\n#endif // FRAMES_DATA_H\n")

print("Successfully generated frames_data.h (UTF-8 encoding)")