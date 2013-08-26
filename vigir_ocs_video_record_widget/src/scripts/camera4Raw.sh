#!/bin/bash
cd $1
plugctl -n 1 oPCR[0].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97d8 - | tee >(playdv --disable-audio --no-mmap -d 1) >(dvgrab -format dv1 -stdin $2_)
