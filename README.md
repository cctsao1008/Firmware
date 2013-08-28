Firmware
========

Modified PX4 Firmware for support TMR-FC board


Firmware update steps :

# Update bootloader

    sudo dfu-util -a 0 -d 0x0483:0xdf11 --dfuse-address 0x08000000 -D tmrfc_bl.bin

# Update OS image using "dfu-util"

    sudo dfu-util -a 0 -d 0x0483:0xdf11 --dfuse-address 0x08004000 -D tmrfc-v1_default.bin

# Update OS image using "QUpgrade"

Note :  To using "QUpgrade" if you have " tmrfc_bl.bin" been pre-flashed !!! 

    1. Get the tool :  https://pixhawk.ethz.ch/px4/downloads
    2. Open "QUpgrade" tool
    3. Select "Advanced"
    4. Select your "tmrfc_bl.px4"
    5. Click "Flash"
    6. Connect your TMRFC board via USB, OR click "RESET" key on TMRFC board

Enjoy your TMRFC !!
