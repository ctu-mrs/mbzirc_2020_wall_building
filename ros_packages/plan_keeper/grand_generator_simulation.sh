#!/bin/bash

brick_x=0.0
brick_y=-12.5

wall_x=0.0
wall_y=2.5

###################################################

wall_height=1.5
brick_height=1.5

###################################################

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

FILENAME="plan_backup/data.txt"

echo "-1" > $FILENAME

echo "$wall_x $wall_y $wall_height 0.0 3.0" >> $FILENAME
echo "$wall_x $wall_y $wall_height 0.0 3.0" >> $FILENAME
echo "$wall_x $wall_y $wall_height 0.0 3.0" >> $FILENAME
echo "$wall_x $wall_y $wall_height 0.0 3.0" >> $FILENAME

echo "$brick_x $brick_y $brick_height 0.0 8.0" >> $FILENAME
echo "$brick_x $brick_y $brick_height 0.0 8.0" >> $FILENAME
echo "$brick_x $brick_y $brick_height 0.0 8.0" >> $FILENAME
