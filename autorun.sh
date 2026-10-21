#!/bin/bash
notify-send "~/Documents/my_code/autorun"
sleep 5
mate-terminal -x bash -c "cd /home/er/Documents/my_code;/usr/bin/python mercury_sequence_ui_V10.py; bash"

