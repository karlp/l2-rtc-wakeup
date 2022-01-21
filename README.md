## Purpose
laks v2 demo for stm32wb

This project does "nothing" other than setup a framework of a few tools.

* Laks for the c++ startup, peripheral register access.
  This is a _very_ thin layer, focused on functional code, that compiles to optimized code.
  I'd point you to the laks readme, but there isn't one...
  Laks implies the use of SCons.
* FreeRTOS because doing things in big main loops is so 80s.
  This project isn't a FreeRTOS demo, but it does show how to build FreeRTOS for
  a c++ project, with SCons.

## Really?

_THIS_ project works on laks+freertos+ custom tickless idle extensions, aiming
for very low power.  Goal is ~equivalent to st cube demos.

## State
wat-exti is a demo of exti interrupts, because I got that all wrong
wat-rtc-wkup is a demo of rtc wakeup, because it wasn't working first
main doens't do anything yet, and is currently abandonded while I clean up other requirements
