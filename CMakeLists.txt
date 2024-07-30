cmake_minimum_required(VERSION 3.28)
project(empty_LP_MSPM0G3507_nortos_ticlang C)

set(CMAKE_C_STANDARD 11)

include_directories(Debug)
include_directories(lib)
include_directories(C:/ti/mspm0_sdk_2_01_00_03/source/ti/driverlib)

add_executable(empty_LP_MSPM0G3507_nortos_ticlang
        Debug/ti_msp_dl_config.c
        Debug/ti_msp_dl_config.h
        lib/Delay.c
        lib/Delay.h
        lib/Encoder.c
        lib/Encoder.h
        lib/extern.h
        lib/I2C.c
        lib/I2C.h
        lib/Motor.c
        lib/Motor.h
        lib/MPU6050.c
        lib/MPU6050.h
        lib/MPU6050_Reg.h
        lib/OLED.c
        lib/OLED.h
        lib/OLED_Font.h
        lib/PID.c
        lib/PID.h
        lib/Servo.c
        lib/Servo.h
        lib/Sonic.c
        lib/Sonic.h
        lib/Uart.c
        lib/Uart.h
        empty.c
        lib/switch.c
        lib/UI.c
        lib/ssd1306.c
        lib/IR.c
        lib/IR.h
        lib/beep.c
        lib/beep.h
        lib/counter.c
        lib/counter.h
        lib/tasks.c
        lib/tasks.h)
