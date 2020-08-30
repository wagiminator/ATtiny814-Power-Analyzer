Uploading Firmware
==================
Connect a UPDI Programmer to the UPDI Header.

For ATtiny814:
--------------
avrdude -C avrdude.conf -c jtag2updi -P /dev/ttyUSB0 -p t814 -e -U fuse0:w:0x00:m -U fuse1:w:0b00000000:m -U fuse2:w:0x02:m -U fuse4:w:0x00:m -U fuse5:w:0xC7:m -U fuse6:w:0x03:m -U fuse7:w:0x00:m -U fuse8:w:0x00:m -U flash:w:PowerAnalyzer_t814.hex:i

For ATtiny1614:
---------------
avrdude -C avrdude.conf -c jtag2updi -P /dev/ttyUSB0 -p t1614 -e -U fuse0:w:0x00:m -U fuse1:w:0b00000000:m -U fuse2:w:0x02:m -U fuse4:w:0x00:m -U fuse5:w:0xC7:m -U fuse6:w:0x03:m -U fuse7:w:0x00:m -U fuse8:w:0x00:m -U flash:w:PowerAnalyzer_t1614.hex:i

AVRDUDE Options:
----------------
-C  config file
-c  programmer
-P  COM port
-p  part number of target device
-e  perform chip erase
