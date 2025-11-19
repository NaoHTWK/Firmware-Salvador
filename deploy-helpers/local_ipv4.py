#!/usr/bin/env python3

"""Print the local IPv4 address of the developer machine so it can be used to recieve debug network packets from robots."""

from typing import Tuple, List, Optional

import socket
import fcntl
import struct
import array
import sys
from collections import namedtuple
from ctypes import *


# On Mac OSX we use the getifaddrs() to find the all IP addresses of all local interfaces.
#
# We would like to also use it on Linux as getifaddrs() is the more modern API,
# but there we recieve undefined address families  in the return values.
# Thus we use the older SIOCGIFCONF ioctl API on Linux.
if sys.platform.startswith('darwin'):

    # https://kbyanc.blogspot.com/2010/11/python-enumerating-ip-addresses-on.html
    #
    # Copyright (c) 2007, Kelly Yancey
    #
    # Permission is hereby granted, free of charge, to any person obtaining a copy
    # of this software and associated documentation files (the "Software"), to deal
    # in the Software without restriction, including without limitation the rights
    # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    # copies of the Software, and to permit persons to whom the Software is
    # furnished to do so, subject to the following conditions:
    #
    # The above copyright notice and this permission notice shall be included in
    # all copies or substantial portions of the Software.
    #
    # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    # THE SOFTWARE.


    class sockaddr_in(Structure):
        _fields_ = [
            ('sin_len',     c_uint8),
            ('sin_family',  c_uint8),
            ('sin_port',    c_uint16),
            ('sin_addr',    c_uint8 * 4),
            ('sin_zero',    c_uint8 * 8)
        ]

        def __str__(self) -> str:
            assert self.sin_len >= sizeof(sockaddr_in)
            return socket.inet_ntoa(bytes(self.sin_addr))

        def address_bytes(self) -> bytes:
            assert self.sin_len >= sizeof(sockaddr_in)
            return bytes(self.sin_addr)


    class sockaddr_in6(Structure):
        _fields_ = [
            ('sin6_len',        c_uint8),
            ('sin6_family',     c_uint8),
            ('sin6_port',       c_uint16),
            ('sin6_flowinfo',   c_uint32),
            ('sin6_addr',       c_uint8 * 16),
            ('sin6_scope_id',   c_uint32)
        ]

        def __str__(self) -> str:
            assert self.sin6_len >= sizeof(sockaddr_in6)
            return socket.inet_ntop(socket.AF_INET6, bytes(self.sin6_addr))

        def address_bytes(self) -> bytes:
            assert self.sin6_len >= sizeof(sockaddr_in6)
            return bytes(self.sin6_addr)


    class sockaddr_dl(Structure):
        _fields_ = [
            ('sdl_len',         c_uint8),
            ('sdl_family',      c_uint8),
            ('sdl_index',       c_short),
            ('sdl_type',        c_uint8),
            ('sdl_nlen',        c_uint8),
            ('sdl_alen',        c_uint8),
            ('sdl_slen',        c_uint8),
            ('sdl_data',        c_uint8 * 12)
        ]

        def __str__(self) -> str:
            assert self.sdl_len >= sizeof(sockaddr_dl)
            addrdata = self.sdl_data[self.sdl_nlen:self.sdl_nlen + self.sdl_alen]
            return ':'.join('{0:02x}'.format(x) for x in addrdata)

        def address_bytes(self) -> bytes:
            return b''  # we don't care for AF_ASH/AF_LINK so we don't implement it


    class sockaddr_storage(Structure):
        _fields_ = [
            ('sa_len',      c_uint8),
            ('sa_family',   c_uint8),
            ('sa_data',     c_uint8 * 254)
        ]


    class sockaddr(Union):
        _anonymous_ = ('sa_storage', )
        _fields_ = [
            ('sa_storage', sockaddr_storage),
            ('sa_sin', sockaddr_in),
            ('sa_sin6', sockaddr_in6),
            ('sa_sdl', sockaddr_dl),
        ]

        def family(self) -> int:
            return self.sa_storage.sa_family

        def __str__(self) -> str:
            family = self.family()
            if family == socket.AF_INET:
                return str(self.sa_sin)
            if family == socket.AF_INET6:
                return str(self.sa_sin6)
            if family == 18:  # AF_LINK
                return str(self.sa_sdl)
            raise NotImplementedError(f"address family {family} not supported")

        def address_bytes(self) -> bytes:
            family = self.family()
            if family == socket.AF_INET:
                return self.sa_sin.address_bytes()
            if family == socket.AF_INET6:
                return self.sa_sin6.address_bytes()
            if family == 18:  # AF_LINK
                return self.sa_sdl.address_bytes()
            raise NotImplementedError(f"address family {family} not supported")


    class ifaddrs(Structure):
        pass
    ifaddrs._fields_ = [
        ('ifa_next',        POINTER(ifaddrs)),
        ('ifa_name',        c_char_p),
        ('ifa_flags',       c_uint),
        ('ifa_addr',        POINTER(sockaddr)),
        ('ifa_netmask',     POINTER(sockaddr)),
        ('ifa_dstaddr',     POINTER(sockaddr)),
        ('ifa_data',        c_void_p)
    ]


    # Define constants for the most useful interface flags (from if.h).
    IFF_UP            = 0x0001
    IFF_BROADCAST     = 0x0002
    IFF_LOOPBACK      = 0x0008
    IFF_POINTTOPOINT  = 0x0010
    IFF_RUNNING       = 0x0040
    if sys.platform.startswith('darwin') or 'bsd' in sys.platform:
        IFF_MULTICAST = 0x8000
    elif sys.platform.startswith('linux'):
        IFF_MULTICAST = 0x1000
    else:
        raise NotImplementedError()

    # Load library implementing getifaddrs and freeifaddrs.
    if sys.platform.startswith('darwin'):
        libc = cdll.LoadLibrary('libc.dylib')
    elif sys.platform.startswith('linux'):
        libc = cdll.LoadLibrary('libc.so.6')
    elif 'bsd' in sys.platform:
        libc = cdll.LoadLibrary('libc.so')
    else:
        raise NotImplementedError()

    # Tell ctypes the argument and return types for the getifaddrs and
    # freeifaddrs functions so it can do marshalling for us.
    libc.getifaddrs.argtypes = [POINTER(POINTER(ifaddrs))]
    libc.getifaddrs.restype = c_int
    libc.freeifaddrs.argtypes = [POINTER(ifaddrs)]

    IfAddr = namedtuple('IfAddr', 'name flags family address netmask address_bytes')


    def getifaddrs():
        """
        Get local interface addresses.

        Returns generator of tuples consisting of interface name, interface flags,
        address family (e.g. socket.AF_INET, socket.AF_INET6), address, and netmask.
        The tuple members can also be accessed via the names 'name', 'flags',
        'family', 'address', and 'netmask', respectively.
        """
        # Get address information for each interface.
        addrlist = POINTER(ifaddrs)()
        if libc.getifaddrs(pointer(addrlist)) < 0:
            raise OSError

        # Iterate through the address information.
        ifaddr = addrlist
        while ifaddr and ifaddr.contents:
            # The following is a hack to workaround a bug in FreeBSD
            # (PR kern/152036) and MacOSX wherein the netmask's sockaddr may be
            # truncated.  Specifically, AF_INET netmasks may have their sin_addr
            # member truncated to the minimum number of bytes necessary to
            # represent the netmask.  For example, a sockaddr_in with the netmask
            # 255.255.254.0 may be truncated to 7 bytes (rather than the normal
            # 16) such that the sin_addr field only contains 0xff, 0xff, 0xfe.
            # All bytes beyond sa_len bytes are assumed to be zero.  Here we work
            # around this truncation by copying the netmask's sockaddr into a
            # zero-filled buffer.
            if ifaddr.contents.ifa_netmask:
                netmask = sockaddr()
                memmove(byref(netmask), ifaddr.contents.ifa_netmask,
                        ifaddr.contents.ifa_netmask.contents.sa_len)
                if netmask.sa_family == socket.AF_INET and netmask.sa_len < sizeof(sockaddr_in):
                    netmask.sa_len = sizeof(sockaddr_in)
            else:
                netmask = None

            try:
                yield IfAddr(
                    ifaddr.contents.ifa_name,
                    ifaddr.contents.ifa_flags,
                    ifaddr.contents.ifa_addr.contents.family(),
                    str(ifaddr.contents.ifa_addr.contents),
                    str(netmask) if netmask else None,
                    ifaddr.contents.ifa_addr.contents.address_bytes()
                )
            except NotImplementedError:
                # Unsupported address family.
                yield IfAddr(
                    ifaddr.contents.ifa_name,
                    ifaddr.contents.ifa_flags,
                    None,
                    None,
                    None,
                    None
                )
            ifaddr = ifaddr.contents.ifa_next

        # When we are done with the address list, ask libc to free whatever memory
        # it allocated for the list.
        libc.freeifaddrs(addrlist)


    def list_ipv4_interface_addrs() -> List[Tuple[str, bytes]]:
        def is_valid_ipv4(a: IfAddr):
            return (a.family is not None) and (a.name is not None) and (a.address_bytes is not None) and (a.family == socket.AF_INET)
        return [(_.name.decode("ascii"), _.address_bytes) for _ in getifaddrs() if is_valid_ipv4(_)]


elif sys.platform.startswith('linux'):


    def list_ipv4_interface_addrs() -> List[Tuple[str, bytes]]:
        # return [
        #     ["lo",bytes.fromhex("7f000001")],  # 127.0.0.1
        #     ["wg0",bytes.fromhex("ac101003")],  # 172.16.16.3
        #     ["docker0",bytes.fromhex("ac110001")],  # 172.17.0.1
        #     ["enx000ec6d6e25a",bytes.fromhex("0a000dbc")],  # 10.0.13.188
        # ]

        MAX_POSSIBLE = 128  # arbitrary. raise if needed.
        SIZE_IN_BYTES = MAX_POSSIBLE * 32
        SIOCGIFCONF = 0x8912

        if_info_bytes = array.array('B', b'\0' * SIZE_IN_BYTES)
        if_info_bytes_addr = if_info_bytes.buffer_info()[0]

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:

            num_out_bytes = struct.unpack('iL', fcntl.ioctl(
                s.fileno(),
                SIOCGIFCONF,
                struct.pack('iL', SIZE_IN_BYTES, if_info_bytes_addr),
            ))[0]

        lst = []
        for i in range(0, num_out_bytes, 40):
            name = if_info_bytes[i:i + 16].tobytes().split(b'\0', 1)[0].decode('utf8')
            ip = if_info_bytes[i + 20:i + 24].tobytes()
            lst.append((name, ip))

        return lst


else:
    raise NotImplementedError("Need implementation for {0}".format(sys.platform))


def find_local_htwk_sepcific_ip() -> Optional[bytes]:
    best_guess = None
    second_best_guess = None
    any_result = None

    HTWK_TEAM_NO = 44
    PREFIX_192_168 = b'\xc0\xa8'
    PREFIX_10_0 = b'\x0a\x00'
    PREFIX_LOOPBACK = 127

    for (ifname, ipv4) in list_ipv4_interface_addrs():
        if (ifname == "lo") or (ifname.startswith("docker")) or (ipv4[0] == PREFIX_LOOPBACK):
            continue
        if ipv4[2] == HTWK_TEAM_NO:
            if best_guess is None:
                prefix = ipv4[:2]
                if prefix in (PREFIX_192_168, PREFIX_10_0):
                    best_guess = ipv4
            if second_best_guess is None:
                second_best_guess = ipv4
        if any_result is None:
            any_result = ipv4

    if best_guess is not None:
        return best_guess
    if second_best_guess is not None:
        return second_best_guess
    return any_result


def print_local_htwk_sepcific_ip(fail_on_missing_addr: bool) -> None:
    if (ip := find_local_htwk_sepcific_ip()) is not None:
        print(socket.inet_ntoa(ip))
        sys.exit(0)

    if fail_on_missing_addr:
        print("Error: could not find local IPv4 address as destination for HTWK Robots network debugging. Exiting.", file=sys.stderr)
        sys.exit(1)
    else:
        print("No Address found.")
        sys.exit(0)


def main() -> None:
    fail_on_missing_addr = True
    if len(sys.argv) > 1:
        if sys.argv[1] == "--no-fail":
            fail_on_missing_addr = False
        else:
            print("Error: unrecognized args: {0}. Exiting.".format(repr(sys.argv)), file=sys.stderr)
            sys.exit(2)
    print_local_htwk_sepcific_ip(fail_on_missing_addr)


if __name__ == "__main__":
    main()
