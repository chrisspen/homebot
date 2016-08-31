#!/bin/bash
# 2015.7.26 CKS
# Meant to be periodically called by cron.
# If network-manager indicates there's no network connection
# it will be restarted.
# This is to fix cases when the wireless connection will hang
# and not automatically reconnect.

# Works in Ubuntu 14/15 but not Raspbian.
#CONNECTED=`nmcli nm status | sed -n 2p | awk '{print $2}'`

# Works on the ancient version of NM that Raspbian uses...
CONNECTED=`nmcli general | sed -n 2p | awk '{print $1}'`

if [ $CONNECTED = "connected" ]
then
    echo "Connected! No need to do anything."
else
    echo "Disconnected! Restarting Network Manager..."
    service network-manager restart
fi
