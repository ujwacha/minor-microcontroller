#! /bin/bash
# arm-none-eabi-objcopy -O binary -S build/robot_base_2025.elf build/minor_.bin

if (( $(st-info --probe 2>/dev/null | wc -l) == 1 ))
then
    echo NONO
    CubeProgrammer -c port=JLINK -w build/minor_.bin 0x8000000 -c port=JLINK reset=SWrst
else
 
    echo YESYES
    CubeProgrammer -c port=SWD -w build/minor_.bin 0x8000000 -c port=SWD reset=SWrst
fi

