import subprocess

elf_path = "/home/juanignacio/Documents/PlatformIO/Projects/spiffs_storage/.pio/build/esp32dev/firmware.elf"
addr2line_tool = "/home/juanignacio/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-addr2line"

addresses = [
    "0x400e3586", "0x400e3898", "0x400e3bf5", "0x400e55f1", "0x400e111c",
    "0x400dceae", "0x40086a51", "0x400858e6", "0x40085970", "0x40085d7b",
    "0x400f92af", "0x400f2786", "0x400eb5b5", "0x400d2c56", "0x400d2e8f", "0x400d4fc5"
]

cmd = [addr2line_tool, "-pfiaC", "-e", elf_path] + addresses
result = subprocess.run(" ".join(cmd), shell=True, capture_output=True, text=True)
print(result.stdout)
