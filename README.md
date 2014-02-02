CC2538-prog
===========

CC2538-prog is a command line application to communicate with the CC2538 ROM bootloader.

    Copyright (c) 2014, Toby Jaffey <toby@1248.io>

    Permission to use, copy, modify, and/or distribute this software for any
    purpose with or without fee is hereby granted, provided that the above
    copyright notice and this permission notice appear in all copies.

    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
    REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
    FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
    INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
    LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
    OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
    PERFORMANCE OF THIS SOFTWARE.


Instructions
============

To use cc2538-prog, the chip must first be configured to enable the backdoor UART bootloader.
The boards in the CC2538DK are preconfigured with the PER firmware which disable the backdoor UART bootloader. To enable it, we need to rewrite this flash.

http://processors.wiki.ti.com/index.php/CC2538_Bootloader_Backdoor#Testing_Bootloader_Backdoor

The simplest way to do this is to flash the supplied file firmware/cc2538dk-contiki-demo.hex with the Windows Flash Programmer tool or Uniflash for Linux.

To use the backdoor UART bootloader, your firmware must apply the change from the above link to startup_gcc.c (remove `BOOTLOADER_BACKDOOR_DISABLE`). This change is present in cc2538dk-contiki-demo.hex

If successful, you should see a flashing LED.

From now on, you may enable the bootloader at any time by removing the jumper on the SmartRF06 for RF2.6. Removing the jumper and pressing the EM RESET button will start the bootloader. Firmware may then be loaded with cc2538-prog.

Once the firmware is loaded, the bootloader must be disabled before reset, by replacing the RF2.6 jumper.

Hands off method
================

Using the above method, every firmware update requires physically lifting a jumper, resetting, replacing and resetting. This is good as a failsafe, but not for speed of development.

However, if the jumper is always present, the bootloader will still be started if no valid firmware image exists in flash. To provoke the chip into this state, a firmware can commit suicide by zapping the CCA (Customer Configuration Area) using `ROM_PageErase(0x0027FFD4, 44);`

After reset, the board will return to the bootloader automatically where cc2538-prog can be used again.


Usage
=====

Flash

./cc2538-prog -d /dev/ttyUSB1 -f myfile.hex

Console/terminal emulator

./cc2538-prog -d /dev/ttyUSB1 -c



