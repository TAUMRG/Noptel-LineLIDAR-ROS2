#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Simulate a simple rangefinder with a LineLidar: print all targets dead ahead

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
from linelidarclass.linelidar import LineLidar, LLchr, \
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

  args = argparser.parse_args()

  # Open communication with the device
  with LineLidar(addr = args.address, port = args.port,
			sshcmd = args.sshcmd,
			sshpypath = args.ssh_python_path) as ll:

    dead_ahead_target_ranges = []
    last_measurementid = None

    # Enable zero results reporting, enable notification on RANGE and start
    # active ranging at 10 Hz
    ll.report_zero_results(True)
    ll.enable_notification(LLchr.RANGE)
    ll.set_sampling_rate(10)

    while True:

      # Get one RANGE notification
      notif = ll.get_notification(chrmask = [LLchr.RANGE])

      # Has the measurement ID changed?
      if last_measurementid is not None and \
		notif.measurementid != last_measurementid:

        # The device is sending us a new measurement: display the targets from
        # the last measurement
        for i, dist in enumerate((dead_ahead_target_ranges + [0, 0, 0])[:3]):
          dist = "/" if dist == 0 else "{:0.2f} m".format(dist)
          print("{:<20}".format("Target #{}: {}".format(i + 1, dist)),
			end = "")
        print("\r", end = "")

        # Clear the list of targets
        dead_ahead_target_ranges = []

      # Find the distance reported by the SPAD cell with the lowest angle -
      #  i.e. the one looking dead ahead
      d = sorted([(abs(t[1]), t[0]) for t in notif.targets])[0][1]

      # Record the range reported by that cell if it isn't 0
      if d != 0:
        dead_ahead_target_ranges.append(d)

      last_measurementid = notif.measurementid
