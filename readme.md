# 6 Digit Led Clock

## The source code for my gps based 6 digit LED clock.

The bare clock can be found [here](https://www.banggood.com/DIY-6-Digit-LED-Large-Screen-Two-Color-Digital-Tube-Desktop-Clock-Kit-Touch-Control-p-1150507.html).

The stl files for the case can be found [here](https://www.thingiverse.com/thing:3839005).

For programming the BlackPill Pro Mini you can find STLink V2 clones [here](https://www.ebay.com/sch/i.html?_from=R40&_nkw=stlink+v2&_sacat=0&rt=nc&LH_BIN=1).

You can find a guide on setting up an arm development environment in VSCode [here](http://hbfsrobotics.com/blog/configuring-vs-code-arm-development-stm32cubemx).



The front display on:

![alt text](https://cdn.thingiverse.com/assets/31/30/11/b6/3e/featured_preview_IMG_20190829_154324.png "Display 1")

The front of the case:

![alt text](https://cdn.thingiverse.com/assets/12/04/bc/7e/b0/featured_preview_IMG_20190829_164002.png "Front 1")

The back of the main pcb:

![alt text](https://cdn.thingiverse.com/assets/99/3c/21/44/87/featured_preview_IMG_20190829_164015.png "Back 1")

The output transistor board for driving higher current through LEDs to make them slightly brighter (driving the leds at 5v instead of the stm32's 3.3v supply):

![alt text](https://cdn.thingiverse.com/assets/4a/27/68/29/3c/featured_preview_IMG_20190829_103334.png "Driver Board")

Main clock pcb pinout: (G is pin 1, pin 14 is GND and pin 12 is 5v)

![alt text](https://cdn.thingiverse.com/assets/97/6c/4f/d3/ae/featured_preview_IMG_20190831_225606.png "Main Board")

Blackpill pro mini pcb pinout:

![alt text](https://cdn.thingiverse.com/assets/eb/f7/02/e7/e6/featured_preview_IMG_20190831_225614.png "Blackpill board")
