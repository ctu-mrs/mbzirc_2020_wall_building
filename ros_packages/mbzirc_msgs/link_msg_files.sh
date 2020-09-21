#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# delete old links
cd "$MY_PATH"
for file in `find msg/*.msg -type l && find srv/*.srv -type l`; do
  rm "$file"
done

# create new msg links
cd "$MY_PATH/msg"
for file in $(find **/*.msg -type f); do
  ln -sf "$file" $(basename $file)
done

# create new srv links
cd "$MY_PATH/srv"
for file in $(find **/*.srv -type f); do
  ln -sf "$file" $(basename $file)
done
