import sys
from pathlib import Path

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, 'r') as f:
    source = f.read()

escaped = source.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n"\n"')

variable = Path(input_file).stem

with open(output_file, 'w') as f:
    f.write(f'const char* {variable} = "{escaped}";\n')
