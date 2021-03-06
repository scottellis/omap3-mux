A small kernel module that lets you look at the current PADCONF register
value for a given GPIO. It also shows the GPIO_OE register status for
the GPIO in question. 

It should work for any OMAP34XX board, but I've only tested it with
Overo's. I wrote it because I keep forgetting to mark the microSD cards 
with the different u-boot muxings I've been playing around with.

The output is similar to the u-boot muxing file board/overo/overo.h

To build it, you need a cross-compile environment and the source tree
for the current kernel that you are running on the device.

If you are using OE, then you can source the appropriate <board>-source-me.txt 
file in the project. There is one for overo and another for the beagleboard.
That's all I've tested.

After that, run make to build.

Here's an example.

	$ git clone git://github.com/scottellis/omap3-mux.git
	$ cd omap3-mux
	$ [optional] export OETMP=/oe1
	$ . overo-source-me.txt 
	$ make
	make -C /oe1/sysroots/overo-angstrom-linux-gnueabi/kernel M=/home/scott/examples/omap3-mux modules 
	make[1]: Entering directory `/oe1/sysroots/overo-angstrom-linux-gnueabi/kernel'
	  CC [M]  /home/scott/examples/omap3-mux/mux.o
	  Building modules, stage 2.
	  MODPOST 1 modules
	  CC      /home/scott/examples/omap3-mux/mux.mod.o
	  LD [M]  /home/scott/examples/omap3-mux/mux.ko
	make[1]: Leaving directory `/oe1/sysroots/overo-angstrom-linux-gnueabi/kernel'


The module will be called mux.ko

Copy it to your device and load it with insmod.

When the module loads a /dev/mux special file will be created. 

Send it a gpio number and it will give you back the PADCONF 
register value corresponding to the GPIO on the console. 

Write is the only I/O operation the module responds to.

A few GPIO appear more then once in table 7.5 of the OMAP3 TRM, for those
pins you will get two lines of output. 

You can find the TRM here [http://www-s.ti.com/sc/techlit/spruf98]
     

Output format

	[padconf_register_address] GPIO_XXX ([16-bit padconf_reg_value]) : [flags]		
	[gpio_oe_register_address] GPIO_OE[bank] : [gpio_oe_reg_value] (bit state) 


The address is at 16-bit resolution as opposed to the TRM Table 7.5's 32-bit level. 

Flags

	IEN - Input Enable
	IDIS - Input Disable
	PTD - Pull type Down
	PTU - Pull type Up
	DIS - Pull type selection is inactive
	EN - Pull type selection is active

If for some reason the value the module is putting out doesn't look right,
verify that I got the gpio to padconf register mapping correct (gp_map) in
mux.c. There was plenty of opportunity for typos there. I've really only
verified the GPIO I'm interested in. 


Sample session

[copy mux.ko to the overo]

	root@overo:~# ls
	mux.ko

	root@overo:~# insmod mux.ko 

	root@overo:~# ls -all /dev/mux
	crw-r--r-- 1 root root 251, 0 Feb  7 12:42 /dev/mux

	root@overo:~# echo 170 > /dev/mux
	0x480021c6  GPIO_170 (0x001c) : IDIS | PTU | EN | M4
	0x49058034 GPIO_OE[5] : 0xFF7FFEEF bit 10 is ON (input)

	root@overo:/sys/class/gpio# echo 170 > export

	root@overo:/sys/class/gpio# echo 170 > /dev/mux
	0x480021c6  GPIO_170 (0x001c) : IDIS | PTU | EN | M4
	0x49058034 GPIO_OE[5] : 0xFF7FFEEF bit 10 is ON (input)

	root@overo:/sys/class/gpio# echo out > gpio170/direction

	root@overo:/sys/class/gpio# echo 170 > /dev/mux
	0x480021c6  GPIO_170 (0x001c) : IDIS | PTU | EN | M4
	0x49058034 GPIO_OE[5] : 0xFF7FFAEF bit 10 is OFF (output)

	root@overo:~# echo 146 > /dev/mux
	0x48002178  GPIO_146 (0x0104) : IEN | PTD | DIS | M4
	0x49056034 GPIO_OE[4] : 0xFFFFFFFF bit 18 is ON (input)

	root@overo:~# rmmod mux


