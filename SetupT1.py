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
    parser.add_argument("-b", "--booster", default="192.168.10.102", help="IP Address of the robots orin board")
    #parser.add_argument("-m", "--master", default="192.168.10.101", help="IP Address of the robots intel board")
    args = parser.parse_args()

    fix_ssh_key_file_permissions()
    upload_file(args.booster, "./deploy-helpers/.ssh/authorized_keys", "/home/booster/.ssh/authorized_keys")

    # TODO: run it
    # subprocess.run(["./install.bash", args.ip])

    # Your Team ID goes here
    team_id = 13

    booster_id = 20
    booster_name = "Booster"

    eth0_addr = exec_command(args.booster, "cat /sys/class/net/eth0/address")
    match eth0_addr:
        case "YourBoostersMacAddress":
            booster_id = 21
            booster_name = "YourBoosterName"
        case _:
            print(f"Unknown Robot, MAC: {eth0_addr}")
            while True:
                try:
                    booster_id = int(input("Enter a robot id: "))
#                    if booster_id < 2 or booster_id > 30:
#                        print("Invalid number, please enter a number from 2 to 30")
#                        continue
#                    break
                except KeyboardInterrupt:
                    sys.exit(1)
                except:
                    print("Invalid input, please enter a number!")
                    pass
            booster_name = input("Enter a robot name: ")

    run_command(f"ssh booster@{args.booster} 'echo 123456 | sudo -S bash -c \"echo {booster_name} > /etc/hostname\"'")
    #run_command(f"ssh master@{args.master} 'echo 123456 | sudo -S bash -c \"echo {booster_name} > /etc/hostname\"'")

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

    with open("/tmp/default-wifi-powersave-off.conf", "w") as f:
        f.write("""
[connection]
wifi.powersave = 2
""")

    with open("/tmp/htwk-bootup.service", "w") as f:
        f.write("""
[Unit]
Description=Switch LAN Profile after boot
After=NetworkManager-wait-online.service
Wants=NetworkManager-wait-online.service

[Service]
Type=oneshot
ExecStart=/usr/bin/bash etc/systemd/htwk-boot-script.bash
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
""")

    with open("/tmp/htwk-boot-script.bash", "w") as f:
        f.write("""
#!/usr/bin/env bash

killall realsense2_camera_node
killall booster-video-stream

/usr/bin/nmcli con up htwk-lan
# For T1 without Intel board only
/usr/bin/nmcli con up booster-lan
#iw dev "$(ls /sys/class/net/ | grep -E '^wl')" set power_save off
""")

    with open("/tmp/setup_robot.bash", "w") as f:
        f.write(f"""
#!/usr/bin/env bash

# This script runs as root

echo "-1" > /home/booster/jerseynumber.txt
chown booster:booster /home/booster/jerseynumber.txt
chmod 644 /home/booster/jerseynumber.txt

tee /etc/udev/rules.d/99-realsense-d455.rules > /dev/null << 'EOF'
# Intel RealSense D455
SUBSYSTEM=="usb", ATTR{{idVendor}}=="8086", ATTR{{idProduct}}=="0b5c", MODE="0666", GROUP="video"
SUBSYSTEM=="usb", ATTR{{idVendor}}=="8086", ATTR{{idProduct}}=="0b5c", ATTR{{authorized}}="1"
# Additional D455 variants (if any)
SUBSYSTEM=="usb", ATTR{{idVendor}}=="8086", ATTR{{idProduct}}=="0b5d", MODE="0666", GROUP="video"
SUBSYSTEM=="usb", ATTR{{idVendor}}=="8086", ATTR{{idProduct}}=="0b5d", ATTR{{authorized}}="1"
EOF

usermod -a -G video $USER
usermod -a -G plugdev $USER
udevadm control --reload-rules
udevadm trigger

nmcli -t -f NAME con show | while read -r name; do
    nmcli con delete "$name"
done

nmcli con add type ethernet ifname eth0 con-name htwk-lan
nmcli con modify htwk-lan ipv4.addresses 192.168.10.102/24,10.0.{team_id}.{booster_id}/16 ipv4.gateway 192.168.10.1 ipv4.dns "8.8.8.8 1.1.1.1" ipv4.method manual

# For T1 without Intel board only
nmcli con add type ethernet ifname "$(ls /sys/class/net/ | grep -E '^en')" con-name booster-lan
nmcli con modify booster-lan ipv4.addresses 192.168.13.101/24 ipv4.gateway 192.168.13.1 ipv4.dns "8.8.8.8 1.1.1.1" ipv4.method manual

nmcli con add type wifi con-name htwk-wifi ifname "$(ls /sys/class/net/ | grep -E '^wl')" ssid your_ssid_here
nmcli con modify htwk-wifi wifi-sec.key-mgmt wpa-psk
nmcli con modify htwk-wifi wifi-sec.psk your_password_here
nmcli con modify htwk-wifi ipv4.addresses 192.168.{team_id}.{booster_id}/16 ipv4.gateway 192.168.{team_id}.1 ipv4.dns "8.8.8.8 1.1.1.1" ipv4.method manual
nmcli con modify htwk-wifi connection.autoconnect yes

mv /tmp/nftables.conf /etc/nftables.conf
mv /tmp/default-wifi-powersave-off.conf /etc/NetworkManager/conf.d/default-wifi-powersave-off.conf
rm /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
mv /tmp/htwk-bootup.service /etc/systemd/system/htwk-bootup.service
mv /tmp/htwk-boot-script.bash /etc/systemd/htwk-boot-script.bash
chmod +x /etc/systemd/htwk-boot-script.bash

systemctl daemon-reload

systemctl enable nftables
systemctl enable htwk-bootup

reboot
""")

    upload_file(args.booster, "./deploy-helpers/.bash_aliases", "/home/booster/.bash_aliases")
    upload_file(args.booster, "/tmp/nftables.conf", "/tmp/nftables.conf")
    upload_file(args.booster, "/tmp/default-wifi-powersave-off.conf", "/tmp/default-wifi-powersave-off.conf")
    upload_file(args.booster, "/tmp/htwk-bootup.service", "/tmp/htwk-bootup.service")
    upload_file(args.booster, "/tmp/htwk-boot-script.bash", "/tmp/htwk-boot-script.bash")
    upload_file(args.booster, "/tmp/setup_robot.bash", "/tmp/setup_robot.bash")

    exec_command(args.booster, "chmod +x /tmp/setup_robot.bash")
    run_command(f"ssh booster@{args.booster} 'echo 123456 | sudo -S bash -c /tmp/setup_robot.bash&'")
    #run_command(f"ssh master@{args.master} 'echo 123456 | sudo -S bash -c reboot'")

# Recovery:
# - ssh booster@192.168.10.102
# - cd ~/Documents/recovery
# - ./v1.2.1.2-release-perception-aarch64.run
