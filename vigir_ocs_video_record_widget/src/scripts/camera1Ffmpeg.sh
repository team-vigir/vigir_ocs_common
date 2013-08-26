#!/bin/bash
cd $1
dvgrab -format dv1 - | tee >(playdv --disable-audio --no-mmap) >(ffmpeg -f dv -i - -b 200k -ab 128k -y $2.mkv)
