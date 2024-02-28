#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Display LineLidar targets by distance and angle on a simple terminal readout

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
ll_default_frequency = 10 #Hz
update_targets_every = 1 / 25 #s



## Modules
#

import os
import sys
import colorama
import argparse
from time import time
from math import radians, cos, sin

sys.path.append(".")
sys.path.append("..")
from linelidarclass.linelidar import LineLidar, LLchr, \
					_ll_default_port, \
					_ll_default_ssh_python_path



## Constants
#

unicode_block_elements = {
  (0,0,
   0,0): " ",		# Space

  (1,1,
   0,0): "\u2580",	# Upper half block

  (0,0,
   1,1): "\u2584",	# Lower half block

  (1,1,
   1,1): "\u2588",	# Full block

  (1,0,
   1,0): "\u258c",	# Left half block

  (0,1,
   0,1): "\u2590",	# Right half block

  (0,0,
   1,0): "\u2596",	# Quadrant lower left

  (0,0,
   0,1): "\u2597",	# Quadrant lower right

  (1,0,
   0,0): "\u2598",	# Quadrant upper left

  (1,0,
   1,1): "\u2599",	# Quadrant upper left and lower left and lower right

  (1,0,
   0,1): "\u259a",	# Quadrant upper left and lower right

  (1,1,
   1,0): "\u259b",	# Quadrant upper left and upper right and lower left

  (1,1,
   0,1): "\u259c",	# Quadrant upper left and upper right and lower right

  (0,1,
   0,0): "\u259d",	# Quadrant upper right

  (0,1,
   1,0): "\u259e",	# Quadrant upper right and lower left

  (0,1,
   1,1): "\u259f",	# Quadrant upper right and lower left and lower right
}

unicode_block_elements_aspect_ratio = 1 / 2



## Routines
#

def display_targets(fullscreen, min_range, max_range, min_angle, max_angle,
			calibrated_angles, targets):
  """Display targets as text semigraphics on the console
  """

  # determine the maximum horizontal span of any target
  min_target_x = sin(radians(min_angle)) * max_range
  max_target_x = sin(radians(max_angle)) * max_range
  max_x_span = max_target_x - min_target_x

  # Get the size of the terminal
  cols, lines = os.get_terminal_size()
  cols -= 1	# Don't use the last column to avoid linewrap issues in Windows

  # Vertical distance label format
  v_dist_lbl_fmt = "{:0.1f}|"

  # Horizontal distance label format
  h_dist_lbl_fmt = " {:0.0f} "

  # Determine the length of the maximum vertical distance label
  v_dist_lbl_max_len = len(v_dist_lbl_fmt.format(max_range))

  # Determine the length of the maximum horizontal distance label
  max_x = max(abs(min_target_x), abs(max_target_x))
  h_dist_lbl_max_len = len(h_dist_lbl_fmt.format(max_x))

  # Make sure we have enough terminal real-estate
  if cols <= v_dist_lbl_max_len or lines < 2:
    return

  # Calculate the size of the semigraphics canvas, leaving enough columns on
  # the left for vertical distance labels and one line at the bottom for
  # horizontal ones
  canvas_width = (cols - v_dist_lbl_max_len) * 2
  canvas_height = (lines - 1) * 2

  # Calculate the distance (meters) -> pixels conversion functions for the
  # horizontal and vertical axes to optimize the use of the canvas space
  if fullscreen:
    usable_x_pixels = canvas_width
    usable_y_pixels = canvas_height
  
  else:
    pixel_ratio_x = pixel_ratio_y = canvas_width / max_x_span
    pixel_ratio_y *= unicode_block_elements_aspect_ratio
    usable_x_pixels = canvas_width
    usable_y_pixels = pixel_ratio_y * max_range

    if usable_y_pixels > canvas_height:
      pixel_ratio_x = pixel_ratio_y = canvas_height / max_range
      pixel_ratio_x /= unicode_block_elements_aspect_ratio
      usable_x_pixels = pixel_ratio_x * max_x_span
      usable_y_pixels = canvas_height

  m2px_x = lambda m: round(m * (usable_x_pixels - 1) / max_x_span)
  m2px_y = lambda m: round(m * (usable_y_pixels - 1) / max_range)

  # Add markers for the range limits to the targets
  for a in calibrated_angles:
    targets.append((min_range, a, None))
    targets.append((max_range, a, None))

  # Add markers for the aperture limits to the targets
  d = min_range
  s = max_range / usable_y_pixels
  while d < max_range:
    targets.append((d, min_angle, None))
    targets.append((d, max_angle, None))
    d += s

  # Create a blank canvas
  canvas = [[0] * canvas_width for _ in range(canvas_height)]

  # Plot the targets and markers onto the canvas
  for t in targets:
    dist, angle, _ = t
    m = canvas_height - 1 - m2px_y(dist * cos(radians(angle)))
    canvas[canvas_height - 1 - m2px_y(dist * cos(radians(angle)))] \
		[m2px_x(dist * sin(radians(angle)) - min_target_x)] = 1

  # Create the horizontal ruler
  hruler_step_meters = 1
  while m2px_x(hruler_step_meters) / 2 < h_dist_lbl_max_len:
    hruler_step_meters += 1

  h_lbl_col_vals = {int(m2px_x(x -min_target_x) / 2): x \
			for x in range(-100 * hruler_step_meters,
					100 * hruler_step_meters,
					hruler_step_meters)}
  h_lbl_col_vals = {c + v_dist_lbl_max_len: abs(h_lbl_col_vals[c]) \
			for c in h_lbl_col_vals \
			if 0 <= c < cols - v_dist_lbl_max_len}

  h_ruler = "{{:>{}}}".format(v_dist_lbl_max_len).format("[m]")
  for c in h_lbl_col_vals:
    h_ruler += " " * (c - len(h_ruler)) + \
		"{{:<{}}}".format(h_dist_lbl_max_len).format(h_lbl_col_vals[c])
  h_ruler_width = int(v_dist_lbl_max_len + usable_x_pixels / 2)
  h_ruler = (h_ruler[:h_ruler_width] + (" " * cols))[:cols]

  # Create the text lines to display
  textlines = []
  for j in range(0, canvas_height, 2):
    v_dist = (canvas_height - j - 2) / usable_y_pixels * max_range
    v_lbl = "{{:>{}}}".format(v_dist_lbl_max_len).format(
					v_dist_lbl_fmt.format(v_dist) \
						if v_dist <= max_range else "")
    plottxt = "".join([unicode_block_elements[tuple(
			canvas[j][i : i + 2] + canvas[j + 1][i : i + 2])] \
			for i in range(0, canvas_width, 2)])
    textlines.append(v_lbl + plottxt)

  # Home the cursor
  print(colorama.Cursor.POS(1, 1), end = "")

  # Display the text lines
  for l in textlines:
    print(l)
  print(h_ruler, end = "\r")



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
	  "-f", "--frequency",
	  help = "Sampling rate",
	  type = int,
	  default = ll_default_frequency
	)

  argparser.add_argument(
	  "-F", "--fullscreen",
	  help = "Disregard the aspect ratio and maximize the display",
	  action = "store_true"
	)

  args = argparser.parse_args()

  # Initialize colorama, so the Windows console understands ANSI codes
  colorama.init()

  # Open communication with the device
  with LineLidar(addr = args.address, port = args.port,
			sshcmd = args.sshcmd,
			sshpypath = args.ssh_python_path) as ll:

    # Get the current minimum and maximum ranging distances
    min_range = ll.read_chr(LLchr.MIN_DISTANCE).distance
    max_range = ll.read_chr(LLchr.MAX_DISTANCE).distance

    # Get the calibrated angles
    calibrated_angles = ll.read_chr(LLchr.CALIBRATED_ANGLES).angles
    min_angle = min(calibrated_angles)
    max_angle = max(calibrated_angles)

    targets = []
    last_measurementid = None
    last_display_update = 0

    # Disable zero results reporting, enable notification on RANGE and start
    # active ranging at the required frequency
    ll.report_zero_results(False)
    ll.enable_notification(LLchr.RANGE)
    ll.set_sampling_rate(args.frequency)

    while True:

      # Get one RANGE notification
      notif = ll.get_notification(chrmask = [LLchr.RANGE])

      # Has the measurement ID changed?
      if last_measurementid is not None and \
		notif.measurementid != last_measurementid:

        # The device is sending us a new measurement: display the targets from
        # the last measurement
        now = time()
        if now - last_display_update > update_targets_every:
          display_targets(args.fullscreen, min_range, max_range,
				min_angle, max_angle, calibrated_angles,
				targets)
          last_display_update = now

        # Clear the list of targets
        targets = []

      # Record the targets
      targets.extend(notif.targets)

      last_measurementid = notif.measurementid
