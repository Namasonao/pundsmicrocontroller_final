# pundsmicrocontroller_final
gyroscope mouse

## Running:
git clone this project

make sure folder name matches name of .ioc file

open in STM

open .ioc file

generate C code

run C code

hope it works (if not send messages, maybe I can help)

## Done so far:
Import Gyroscope driver (LSM6DSL)

Configure driver with read/write functions

Print acceleration data


## Next steps:
Understand setup of gyroscope (Several filters are already enabled, but I don't know what they do)

Configure it so it changes acceleration faster (Right now it changes from 1000mg to -1000mg very slowly when you quickly turn around the microcontroller)
