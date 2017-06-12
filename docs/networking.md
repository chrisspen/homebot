Networking
==========

Both wired and wireless connections are managed by Network Manager.

The wired connection will automatically be enabled when an ethernet cable is connected.

Wireless requires some setup, requiring an initial wired connection. Attached an ethernet cable and then confirm you can SSH in via:

    fab prod shell

Exit the shell, and setup the wireless connection by running:

    fab prod nm.add_wifi_connection:<SSID>,<PASSPHRASE>

If you get an error saying " No network with SSID '<SSID>' found." then list all available wireless networks by running:

    nmcli dev wifi list

If none are listed, then that indicates there might be a problem with the wireless driver.

Ensure all your packages and firmware is up-to-date by following [these instructions](https://raspberrypi.stackexchange.com/questions/44253/raspberry-pi-3-isnt-finding-wifi-networks).

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get dist-upgrade
    sudo rpi-update
    sudo reboot
