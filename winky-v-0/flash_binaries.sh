if [ -z "$1" ]
then
    BUILD="Debug"
else
    BUILD=$1
fi

arm-none-eabi-objcopy -O binary $BUILD/winky.elf $BUILD/winky.bin
STM32_Programmer.sh -c port=SWD -w winky_bootloader.hex
STM32_Programmer.sh -c port=SWD -w $BUILD/winky.bin 0x08007000
STM32_Programmer.sh -c port=SWD -rst
