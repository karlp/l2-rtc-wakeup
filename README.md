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
use gdb to ```set opt_really_use_leds = false ``` first, so that you don't
fluctuating numbers.  Currents are measured on JP3, while running the board
from the STLINK usb connection.  (JP1=STLink)

### leds off, 64Mhz, tickless idle==1, 1000Hz, port optim task sel
This gives us ~345uA flat.  Not bad, not great though.  This is what you get
"for free" just from using freertos on a cortex-m.  But we should be able to
get a lot lower...

### leds off, 64Mhz, tickless idle==1, 1000Hz, no port optim task sel
"same"  Apparently the task selection can make a difference if you have many
of them coming active, but no big deal otherwise

### leds off, 64Mhz, tickless idle==0, 1000Hz, port optim task sel
347/348uA, only a little higher, but again, our task mix probably isn't really
giving us a lot here.

### leds off, 32Mhz, tickless idle==1, 1000Hz
159uA.  Better, a lot, even better than the half clock rate would imply.  But
we need more, much more...
