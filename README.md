LIS3MDL SPI Driver for STM32 microcontrollers.

# Usage 
--------
just add .c and .h file to Src Inc of your project. 
Then configure the following parts based on your code. 
* 5th line of .h file 
*  lines 50-51 for your SPI pins
* use main.c file to see how you should use the library and initiate your LIS3MDL sensors 

* (optional) change SPI_SendRecieveByte if you are using another send/rec mechanism

# Acknowledgement 
-------

This library is inspired by another driver, which I couldn't find at the of pushing.