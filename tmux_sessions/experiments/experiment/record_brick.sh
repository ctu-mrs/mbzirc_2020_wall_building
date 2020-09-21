#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

exclude=(
  '(.*)compressedDepth(.*)'
  '(.*)theora(.*)'
  '(.*)h264(.*)'
# bluefox
'/$(arg UAV_NAME)/bluefox_optflow/(.*)'
# bluefox detect
'/$(arg UAV_NAME)/bluefox_brick/image_raw'
'/$(arg UAV_NAME)/bluefox_brick/image_raw/compressed/(.*)'
# Realsense d435
'(.*)rs_d435(.*)depth_to_infra(.*)'
'(.*)rs_d435(.*)image_rect_raw/compressed(.*)'
'(.*)rs_d435(.*)depth_to_color/image_raw'
'(.*)rs_d435(.*)depth_to_color(.*)compressed'
'(.*)rs_d435(.*)depth_to_color(.*)compressed/(.*)'
'(.*)rs_d435(.*)color(.*)'
'(.*)rs_d435(.*)/infra(.*)'
'(.*)rs_d435(.*)/color/image_rect_color'
)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
