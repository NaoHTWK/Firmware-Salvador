#!/usr/bin/env python3

import argparse
import subprocess
import sys
import threading
import re

class MyThread (threading.Thread):
    def __init__(self, ipPrefix, ip, ssid):
        threading.Thread.__init__(self)
        self.ip = ip
        self.ipPrefix = ipPrefix
        self.ssid = ssid

    def run(self):
        try:
            print_str = "./install.bash --deploy " + self.ipPrefix + str(self.ip)
            cmd = ["./install.bash", "--deploy", self.ipPrefix + str(self.ip)]

            if ssid:
                print_str += " --ssid " + self.ssid
                cmd.extend(["--ssid", self.ssid])

            print(print_str)
            out = subprocess.check_output(cmd)

            print ("Installing finished of " + str(self.ip) + "\n")
        except subprocess.CalledProcessError as arg:
            print ("Installing failed for " + str(self.ip) + " cause: " + str(arg))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--wifi', action='store_true', help='connect via wifi instead of lan')
    parser.add_argument('ips', metavar='int', type=int, choices=range(255),
                        nargs='+', help='nao numbers')
    parser.add_argument('-s', '--ssid', metavar='SSID', type=str, nargs=1, help='set a Wifi after installing')
    args = parser.parse_args()

    if len(args.ips) == 0:
        print ("No Booster ip was given!")
        sys.exit(1)

    team_id = 14

    if args.wifi:
        pre = "192.168." + str(team_id) + "."
    else:
        pre = "10.0." + str(team_id) + "."

    ip = str(subprocess.check_output(["./deploy-helpers/local_ipv4.py", "--no-fail"]))
    ip = re.findall(r'[0-9]+(?:\.[0-9]+){3}', ip)[0]
    if pre not in ip:
        print("Your IP (" + ip + ") isn't in the correct space")
        print("Expected: " + pre + "*")
        ##sys.exit(1)

    ssid = ""
    if args.ssid:
        ssid = args.ssid[0]

    print ("Compiling firmware")
    out = subprocess.check_output(["./install.bash"])
    print ()
    print ()
    print ()

    threads = []

    for ip in args.ips:
        thread = MyThread(pre, ip, ssid)
        thread.start()
        threads.append(thread)

    print ()
    print ()
    print ()
    for t in threads:
        t.join()

