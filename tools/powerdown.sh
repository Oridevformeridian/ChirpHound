#!/bin/bash
# Power-sweep step: drop the Heltec beacon to 20 dBm (-10 dB) after the 30 dBm
# segment. Restarts the beacon (seq resets to 00001) so the low-power segment
# is trivially separable in MESHTAST.TXT. Never queries the port while the
# beacon runs -- that contention has crashed it twice.
LOG=/tmp/sweep.log
echo "=== powerdown firing $(date "+%F %T") ===" >> $LOG
systemctl --user stop meshbeacon
sleep 3
~/mesh-venv/bin/meshtastic --port /dev/ttyUSB0 --set lora.tx_power 20 >> $LOG 2>&1
sleep 12                     # let the Heltec config-write reboot settle
systemctl --user reset-failed meshbeacon 2>/dev/null
systemd-run --user --unit=meshbeacon ~/mesh-venv/bin/python ~/.local/share/beacon.py --interval 10 >> $LOG 2>&1
echo "beacon restarted at 20 dBm $(date "+%F %T")" >> $LOG
