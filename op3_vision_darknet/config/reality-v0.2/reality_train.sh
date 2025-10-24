#!/bin/bash -e

cd /media/panther/TORSO_21_dataset/data/reality

# Warning: this file is automatically created/updated by DarkMark v1.10.18-1!
# Created on Thu 2025-10-23 23:38:34 BST by karen@marlyn.

rm -f output.log
#rm -f chart.png

echo "creating new log file" > output.log
date >> output.log

ts1=$(date)
ts2=$(date +%s)
echo "initial ts1: ${ts1}" >> output.log
echo "initial ts2: ${ts2}" >> output.log
echo "cmd: /home/karen/darknet/darknet-HA/build/src-cli/darknet detector -map -dont_show train /media/panther/TORSO_21_dataset/data/reality/reality.data /media/panther/TORSO_21_dataset/data/reality/reality.cfg /media/panther/TORSO_21_dataset/data/reality/reality_best.weights -clear" >> output.log

/usr/bin/time --verbose /home/karen/darknet/darknet-HA/build/src-cli/darknet detector -map -dont_show train /media/panther/TORSO_21_dataset/data/reality/reality.data /media/panther/TORSO_21_dataset/data/reality/reality.cfg /media/panther/TORSO_21_dataset/data/reality/reality_best.weights -clear 2>&1 | tee --append output.log

ts3=$(date)
ts4=$(date +%s)
echo "ts1: ${ts1}" >> output.log
echo "ts2: ${ts2}" >> output.log
echo "ts3: ${ts3}" >> output.log
echo "ts4: ${ts4}" >> output.log

