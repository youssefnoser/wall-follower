#!/bin/bash
echo "Erasing log pages (60 to 63) in STM32 flash..."
openocd -f interface/stlink.cfg -c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "init" -c "halt" -c "flash erase_address 0x0800F000 4096" -c "resume" -c "exit"
echo "Logs erased successfully!"
