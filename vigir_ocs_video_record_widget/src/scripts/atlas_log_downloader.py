#!/usr/bin/env python
 
import os
import sys
import traceback

class App:
    def __init__(self, ip_entry, dir_entry, secs_entry):
        self.ip_entry = ip_entry
        robot_interface_directory = os.getenv("ATLAS_ROBOT_INTERFACE", ".")
        self.dir_entry = dir_entry
        self.secs_entry = secs_entry

    def log(self):
        try:
            # try to find jef client
            robot_interface_directory = os.getenv("ATLAS_ROBOT_INTERFACE", ".")
            jef_path = os.path.join(robot_interface_directory, "tools", "jef-client")

            if(not os.path.isfile(jef_path)):
                jef_path = "./jef-client"

            if(not os.path.isfile(jef_path)):
                jef_path = "jef-client"

            secs = self.secs_entry
            ip = self.ip_entry
            path = self.dir_entry

            if(not os.path.exists(path)):
                os.mkdir(path)
                
            print("logging {} seconds of data from {} to {}".format(secs, ip, path))
            #os.system("{} -s {} -H {} -D {}".format(jef_path, secs, ip, path))
        except:
            traceback.print_exc()

if __name__ == "__main__":
    ip_entry = "10.66.171.30"
    dir_entry = "."
    secs_entry = "30"
    if len(sys.argv) == 4:
        ip_entry = sys.argv[1]
        dir_entry = sys.argv[2]
        secs_entry = int(sys.argv[3])

    app = App(ip_entry,dir_entry,secs_entry)
    app.log()
