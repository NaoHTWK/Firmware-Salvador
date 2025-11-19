#!/usr/bin/env python3

# To install: paramiko, pexpect

import argparse
import os
import paramiko
import pexpect
import stat
import subprocess
import sys

from collections import namedtuple


def fix_ssh_key_file_permissions():
    files_to_fix = [
        "deploy-helpers/your.ssh.key.here",
        "deploy-helpers/your.ssh.key.here.pub",
        "deploy/.ssh",
        "deploy/.ssh/authorized_keys"
    ]

    for path in files_to_fix:
        if not os.path.exists(path):
            continue

        file_mode = os.stat(path).st_mode
        group_perms = (file_mode & 0o070) >> 3
        other_perms = file_mode & 0o007

        if group_perms != 0 or other_perms != 0:
            print(f"Fixing SSH key permissions: {path}")
            os.chmod(path, file_mode & 0o700)


def upload_file(ip, local_file, remote_file):
    remote_dir = os.path.dirname(remote_file)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=ip, port=22, username="booster", password="123456", look_for_keys=False)
    sftp = client.open_sftp()
    try:
        sftp.chdir(remote_dir)
    except IOError:
        sftp.mkdir(remote_dir)
        sftp.chdir(remote_dir)
    sftp.put(local_file, remote_file)
    sftp.close()
    client.close()


def exec_command(ip, cmd):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=ip, port=22, username="booster", password="123456", look_for_keys=False)
    _, stdout, _ = client.exec_command(cmd)
    out = stdout.read().decode().strip()
    client.close()
    return out


def run_command(cmd):
    child = pexpect.spawn("/bin/bash", ["-c", cmd], timeout=10)

    try:
        while True:
            i = child.expect([
                "[Pp]assword",
                pexpect.EOF,
                pexpect.TIMEOUT
            ])
            if i in [0]:
                child.sendline("123456")
            elif i == 1:
                break
            elif i == 2:
                return False
    except Exception:
        return False

    output = child.before.decode(errors="ignore")
    print(output)
    return child.exitstatus == 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Booster Setup Script")
    parser.add_argument("ip", help="IP Address of the robot")
    args = parser.parse_args()

    fix_ssh_key_file_permissions()
    upload_file(args.ip, "./deploy-helpers/.ssh/authorized_keys", "/home/booster/.ssh/authorized_keys")

    # TODO: run it
    # subprocess.run(["./install.bash", args.ip])

    # Your Team ID goes here
    team_id = 14

    booster_id = 10
    booster_name = "Booster"

    eth0_addr = exec_command(args.ip, "cat /sys/class/net/eth0/address")
    eth1_addr = exec_command(args.ip, "cat /sys/class/net/en*/address")
    wlan0_addr = exec_command(args.ip, "cat /sys/class/net/wl*/address")
    match eth0_addr:
        case "YourBoosterMacAddress":
            booster_id = 11
            booster_name = "YourBoosterName"
        case _:
            print(f"Unknown Robot, MAC: {eth0_addr}")
            while True:
                try:
                    booster_id = int(input("Enter a robot id: "))
                    if booster_id < 2 or booster_id > 30:
                        print("Invalid number, please enter a number from 2 to 30")
                        continue
                    break
                except KeyboardInterrupt:
                    sys.exit(1)
                except:
                    print("Invalid input, please enter a number!")
                    pass
            booster_name = input("Enter a robot name: ")

    run_command(f"ssh booster@{args.ip} 'echo 123456 | sudo -S bash -c \"echo {booster_name} > /etc/hostname\"'")
    
    with open("/tmp/10-network-names.rules", "w") as f:
        f.write(f"""
SUBSYSTEM=="net", ACTION=="add", ATTR{{address}}=="{wlan0_addr}", NAME="wlan0"
SUBSYSTEM=="net", ACTION=="add", ATTR{{address}}=="{eth0_addr}", NAME="eth0"
SUBSYSTEM=="net", ACTION=="add", ATTR{{address}}=="{eth1_addr}", NAME="eth1"
""")

    with open("/tmp/wpa_supplicant-wlan0.conf", "w") as f:
        f.write("""
ctrl_interface=/run/wpa_supplicant
update_config=1
country=DE
network={
    ssid="***"
    psk="***"
    priority=10
}
""")

    with open("/tmp/wlan0.network", "w") as f:
        f.write(f"""
[Match]
Name=wlan0

[Network]
Address=192.168.{team_id}.{booster_id}/16
Gateway=192.168.{team_id}.1
DNS=8.8.8.8
""")

    with open("/tmp/eth1.network", "w") as f:
        f.write(f"""
[Match]
Name=eth1

[Network]
Address=192.168.{team_id}.{booster_id}/16
Gateway=192.168.{team_id}.1
DNS=8.8.8.8
""")

    with open("/tmp/eth0.network", "w") as f:
        f.write("""
# Don't change this interface !!!
# It's for booster's internal communication
[Match]
Name=eth0

[Network]
Address=192.168.13.101/24
Gateway=192.168.13.1
DNS=8.8.8.8
""")

    with open("/tmp/nftables.conf", "w") as f:
        f.write("""
table inet filter {
	chain input {
		type filter hook input priority filter; policy accept;
	}

	chain forward {
		type filter hook forward priority filter; policy accept;
	}

	chain output {
		type filter hook output priority filter; policy accept;
		# block booster service that broadcasts on port 9000 and violates league rules
		udp dport 9000 drop
	}
}
""")

    with open("/tmp/setup_robot.bash", "w") as f:
        f.write(f"""
#!/usr/bin/env bash

# This script runs as root

mv /tmp/wpa_supplicant-wlan0.conf /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
mv /tmp/wlan0.network /etc/systemd/network/wlan0.network
mv /tmp/eth0.network /etc/systemd/network/eth0.network
mv /tmp/eth1.network /etc/systemd/network/eth1.network
mv /tmp/10-network-names.rules /etc/udev/rules.d/10-network-names.rules
mv /tmp/nftables.conf /etc/nftables.conf

#echo 'GRUB_CMDLINE_LINUX="net.ifnames=0 biosdevname=0"' | tee /etc/default/grub.d/99-disable-ifnames.cfg

#grub-mkconfig

systemctl stop NetworkManager
systemctl disable NetworkManager

systemctl enable systemd-networkd
systemctl start systemd-networkd

systemctl enable --now nftables.service

systemctl enable wpa_supplicant@wlan0
systemctl start wpa_supplicant@wlan0

systemctl stop booster-daemon-perception.service
systemctl disable booster-daemon-perception.service

reboot
""")

    upload_file(args.ip, "./deploy-helpers/.bash_aliases", "/home/booster/.bash_aliases")
    upload_file(args.ip, "/tmp/wpa_supplicant-wlan0.conf", "/tmp/wpa_supplicant-wlan0.conf")
    upload_file(args.ip, "/tmp/wlan0.network", "/tmp/wlan0.network")
    upload_file(args.ip, "/tmp/eth0.network", "/tmp/eth0.network")
    upload_file(args.ip, "/tmp/eth1.network", "/tmp/eth1.network")
    upload_file(args.ip, "/tmp/10-network-names.rules", "/tmp/10-network-names.rules")
    upload_file(args.ip, "/tmp/nftables.conf", "/tmp/nftables.conf")
    upload_file(args.ip, "/tmp/setup_robot.bash", "/tmp/setup_robot.bash")

    exec_command(args.ip, "chmod +x /tmp/setup_robot.bash")
    run_command(f"ssh booster@{args.ip} 'echo 123456 | sudo -S bash -c /tmp/setup_robot.bash&'")
