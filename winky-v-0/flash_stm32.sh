
STM32_Programmer.sh -c port=SWD -hardRst
STM32_Programmer.sh -c port=SWD -rdu
STM32_Programmer.sh -c port=SWD -w Debug/winky.hex --start