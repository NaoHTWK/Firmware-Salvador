#!/usr/bin/env python3

from __future__ import print_function
import os
import sys
import json
import re
import getpass
import subprocess

def parse_config():
    networks = {}
    wildcards = {}

    with open("/home/booster/etc/network_list.json", mode='r') as f:
        data = json.load(f)

        for key, value in data.items():
            wc = value["Wildcard"]
            wildcards[wc] = key

            for ssid in value["SSID"]:
                networks[ssid] = key

        f.close()

    return networks, wildcards

def update_netplan(ssid, password):
    print("Applying new network config ...")

    os.system("sudo nmcli con modify htwk-wifi ssid '" + ssid + "'")
    os.system("sudo nmcli con modify htwk-wifi wifi-sec.psk '" + password + "'")
    os.system("sudo nmcli con down htwk-wifi")
    os.system("sudo nmcli con up htwk-wifi")

def print_array(array, cols = 3):
    for i in range(0, len(array)):
        print("   [" + '{0: >2}'.format(i + 1) + "] " + '{0: <15}'.format(array[i]), end = "")
        if (i % cols) == (cols - 1):
            print("")

    if (len(array) % cols) != 0:
        print("")

def main():
    if len(sys.argv) > 1:
        if sys.argv[1] in ["-h", "--help"]:
            print("Usage:\t%s [SSID [Password]]" % sys.argv[0])
            sys.exit()
        elif sys.argv[1] in ["-l", "--list"]:
            networks, _ = parse_config()
            print(list(networks.keys()))
            sys.exit()

    ssid = ""
    password = ""

    networks, wildcards = parse_config()

    if len(sys.argv) > 1:
        ssid = sys.argv[1]
    else:
        print("Pleas select a SSID, leave empty to enter another one")
        print_array(list(networks.keys()))
        nr_str = input()
        nr = 0

        try:
            nr = max(0, int(nr_str) - 1)
        except ValueError:
            nr = 0

        if nr == 0:
            ssid = input("Enter the SSID to connect to: ")
        else:
            ssid = list(networks.keys())[nr]

    if len(sys.argv) > 2:
        password = sys.argv[2]
    else:
        if ssid in networks:
            password = networks[ssid]
        else:
            for wc, pw in wildcards.items():
                if wc.lower() in ssid.lower():
                    password = pw

    if password == "":
        print("No Password found for SSID '%s'" % ssid)
        print("Select one of the Wildcards or leave empty to enter it by your own")
        print_array(list(wildcards.keys()))
        nr_str = input()
        nr = 0

        try:
            nr = int(nr_str)
        except ValueError:
            nr = 0

        if nr == 0:
            password = getpass.getpass("Pleas enter a Password: ")
        else:
            wc = wildcards.keys()[int(nr)]
            password = wildcards[wc]
    else:
        print("Found Password for SSID '%s'" % ssid)

    update_netplan(ssid, password)

if __name__ == "__main__":
    main()
