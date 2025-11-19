#!/usr/bin/env python3

import argparse
import subprocess
import sys


def start_clusterssh(ipPrefix, ipList):
    proc = ["clusterssh"]

    try:
        subprocess.call(["clusterssh", "--version"])
    except OSError as e:
        try:
            proc = ["cssh"]
            subprocess.call(["cssh", "--version"])
        except OSError as e:
            try:
                proc = ["csshx"]
                subprocess.call(["csshx", "--version"])
            except OSError as e:
                print("You haven't clusterssh installed! Exit!")
                sys.exit(1)

    for ip in ipList:
        proc.append('booster@' + ipPrefix + str(ip))

    try:
        print('start clusterssh')
        out = subprocess.check_output(proc)
    except subprocess.CalledProcessError as arg:
        print("Error starting clusterssh")


if __name__ == '__main__':
    if sys.version_info[0] == 2:
        range = xrange

    parser = argparse.ArgumentParser()
    parser.add_argument('net', choices=('w', 'wifi', 'l', 'lan'), help="specify that we want connect via wifi or lan")
    parser.add_argument('ips', metavar='<nao ip>', type=int, choices=range(255), nargs='+', help='nao numbers')
    args = parser.parse_args()

    if len(args.ips) == 0:
        print("No Nao ip was given!")
        sys.exit(1)

    if len(args.net) == 0:
        print("no net was given!")
        sys.exit(2)

    team_id = 14

    if args.net in ('w', 'wifi'):
        start_clusterssh("192.168." + str(team_id) + ".", args.ips)
    else:
        start_clusterssh("10.0." + str(team_id) + ".", args.ips)

