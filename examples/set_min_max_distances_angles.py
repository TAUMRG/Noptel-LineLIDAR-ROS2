#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Set minimum, maximum ranging distances and/or angles, and optionally save them

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
from linelidarclass.linelidar import LineLidar, LLchr, LLsrv, \
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
	  "-md", "--min-dist",
	  help = "Minimum ranging distance",
	  type = float,
	  required = False
	)

  argparser.add_argument(
	  "-MD", "--max-dist",
	  help = "Maximum ranging distance",
	  type = float,
	  required = False
	)

  argparser.add_argument(
	  "-ma", "--min-angle",
	  help = "Minimum viewing angle",
	  type = float,
	  required = False
	)

  argparser.add_argument(
	  "-MA", "--max-angle",
	  help = "Maximum viewing angle",
	  type = float,
	  required = False
	)

  argparser.add_argument(
	  "-S", "--save",
	  help = "Save in non-volatile memory",
	  action = "store_true"
	)

  args = argparser.parse_args()

  # Open communication with the device
  with LineLidar(addr = args.address, port = args.port,
			sshcmd = args.sshcmd,
			sshpypath = args.ssh_python_path,
			set_clean_state = False) as ll:

    # Get the current minimum and maximum ranging distances
    min_range = ll.read_chr(LLchr.MIN_DISTANCE).distance
    max_range = ll.read_chr(LLchr.MAX_DISTANCE).distance

    # Get the current minimum and maximum viewing angles
    min_view_angle = ll.read_chr(LLchr.MIN_ANGLE).angle
    max_view_angle = ll.read_chr(LLchr.MAX_ANGLE).angle

    # Print the current minimum and maximum ranging distances and viewing angles
    print("Current settings:")
    print("  Minimum ranging distance: {:0.2f} m".format(min_range))
    print("  Maximum ranging distance: {:0.2f} m".format(max_range))
    print("  Minimum viewing angle:    {:0.2f} deg".format(min_view_angle))
    print("  Maximum viewing angle:    {:0.2f} deg".format(max_view_angle))

    settings_modified = False

    # If we were asked to set different minimum or maximum ranging distances,
    # do so
    if args.min_dist is not None and args.min_dist != min_range:
      ll.write_chr(LLchr.MIN_DISTANCE, distance = args.min_dist)
      settings_modified = True

    if args.max_dist is not None and args.max_dist != max_range:
      ll.write_chr(LLchr.MAX_DISTANCE, distance = args.max_dist)
      settings_modified = True

    # If we were asked to set different minimum or maximum viewing angles do so
    if args.min_angle is not None and args.min_angle != min_view_angle:
      ll.write_chr(LLchr.MIN_ANGLE, angle = args.min_angle)
      settings_modified = True

    if args.max_angle is not None and args.max_angle != max_view_angle:
      ll.write_chr(LLchr.MAX_ANGLE, angle = args.max_angle)
      settings_modified = True

    # Were any of the settings modified?
    if settings_modified:

      # Get the new minimum and maximum ranging distances
      min_range = ll.read_chr(LLchr.MIN_DISTANCE).distance
      max_range = ll.read_chr(LLchr.MAX_DISTANCE).distance

      # Get the new minimum and maximum viewing angles
      min_view_angle = ll.read_chr(LLchr.MIN_ANGLE).angle
      max_view_angle = ll.read_chr(LLchr.MAX_ANGLE).angle

      # Print the new minimum and maximum ranging distances and viewing angles
      print()
      print("New settings:")
      print("  Minimum ranging distance: {:0.2f} m".format(min_range))
      print("  Maximum ranging distance: {:0.2f} m".format(max_range))
      print("  Minimum viewing angle:    {:0.2f} deg".format(min_view_angle))
      print("  Maximum viewing angle:    {:0.2f} deg".format(max_view_angle))

      # Were we asked to save the settings in non-volatile memory?
      if args.save:

        # Save the DEVICE_CONFIG service
        ll.save_srv(LLsrv.DEVICE_CONFIG)

        print()
        print("Saved")
