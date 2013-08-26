#!/bin/bash
cd $1
if [ $# -eq 2 ]
then
	quality=$2
else
	quality=200k
fi

for file in $1*.avi ;
do
	ffmpeg -i $file -b $quality -ab 128k -y $file.mkv
done
