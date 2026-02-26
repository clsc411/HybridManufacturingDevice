import re
import os

file_path = r"c:\Users\clara\downloads\HybridManufacturingCastingSystem\docs\requirements.md"

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Pattern to match :contentReference[oaicite:N]{index=N}
# Also handling potential leading space
pattern = r'\s*:contentReference\[oaicite:\d+\]\{index=\d+\}'

new_content = re.sub(pattern, '', content)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_content)

print("Cleaned up content references.")
