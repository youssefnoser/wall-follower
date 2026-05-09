#!/bin/bash
echo "Dumping 4KB of logs from STM32 flash (0x0800F000)..."
openocd -f interface/stlink.cfg -c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "init" -c "halt" -c "dump_image flash_logs.bin 0x0800F000 4096" -c "resume" -c "exit" 2>/dev/null

if [ -f "flash_logs.bin" ]; then
    echo -e "\n=== LOG CONTENTS ==="
    # Use strings to ignore the 0xFFFF unwritten flash memory bytes
    strings flash_logs.bin
    rm flash_logs.bin
else
    echo "Failed to dump logs."
fi
