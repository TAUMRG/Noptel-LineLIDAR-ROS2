#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Print LineLidar device information

Requires:
  - Python3
  - Python3 LineLidar class

Tested on:
  - Linux
  - Windows

Authors:
  PCo

Releases:
  Version 1.0.0 - 08/03/2023 - Initial release
"""

## Parameters
#

ll_default_addr = "192.168.0.2"



## Modules
#

import sys
import argparse

sys.path.append(".")
sys.path.append("..")
from linelidarclass.linelidar import discover, LineLidar, LLchr, \
					_ll_default_port, \
					_ll_default_ssh_python_path



## Main routine
#

if __name__ == "__main__":

  # Parse the command line arguments
  argparser = argparse.ArgumentParser()

  argparser.add_argument(
	  "-a", "--address",
	  help = "IP address of the LineLidar. Default: {}".
			format(ll_default_addr),
	  type = str,
	  default = ll_default_addr
	)

  argparser.add_argument(
	  "-p", "--port",
	  help = "UDP port of the LineLidar. Default: {}".
			format(_ll_default_port),
	  type = int,
	  default = _ll_default_port
	)

  argparser.add_argument(
	  "-s", "--sshcmd",
	  help = 'Non-interactive SSH command to log into a remote shell and '
			'use the host as a relay to communicate with the '
			'LineLidar. E.g. "ssh user@sshserver" in Linux or '
			'Windows >= 10, "plink -batch puttyprofile" in '
			'Windows < 10',
	  type = str
	)

  argparser.add_argument(
	  "-py", "--ssh-python-path",
	  help = "Path of the Python executable on the SSH relay host. "
			"Default: {}".format(_ll_default_ssh_python_path),
	  type = str,
	  default = _ll_default_ssh_python_path,
	)

  argparser.add_argument(
	  "-d", "--discover",
	  help = "Discover devices on a network and display each device's "
			"information. Network syntax should be xx.xx.xx.xx/mm. "
			"If the network is omitted, discover devices on all "
			"available network interfaces.",
	  type = str,
          const = "",
	  nargs = "?"
	)

  args = argparser.parse_args()

  # Should we discover devices?
  if args.discover is not None:

    # Discover devices
    with LineLidar() as ll:
      addresses = discover(network = args.discover if args.discover else None,
				port = args.port,
				sshcmd = args.sshcmd,
				sshpypath = args.ssh_python_path)

    # List the IPs we discovered, if any
    if not addresses:
      print("No LineLidar device discovered")
    else:
      print("LineLidar devices discovered at addresses:")
      for a in addresses:
        print("  {}".format(a))
      print()

  else:
    addresses = [args.address]

  # Iterate over the addresses to probe
  for a in addresses:

    # Open communication with the device
    with LineLidar(addr = a, port = args.port,
			sshcmd = args.sshcmd,
			sshpypath = args.ssh_python_path,
			set_clean_state = False) as ll:

      # Get the device's information
      serial = ll.read_chr(LLchr.SERIAL_NUMBER).value
      fw = ll.read_chr(LLchr.FW_VERSION)
      mac = ll.read_chr(LLchr.MAC).mac

      # Print the device's information
      print("LineLidar device information at address {}:".format(a))
      print("  Serial number: {}".format(serial))
      print("  Firmware:      {}.{}.{}".format(fw.major, fw.minor, fw.bugfix))
      print("  MAC address:   {}".format(mac))
