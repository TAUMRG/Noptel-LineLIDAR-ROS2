#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Display LineLidar targets by distance and angle on a simple graphical readout

Requires:
  - Python3
  - Python3 Tkinter
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
minimum_window_size = 800 #px
update_targets_every = 1 / 25 #s



## Modules
#

import sys
import queue
import argparse
import threading
import numpy as np
import tkinter as tk
from time import time
import multiprocessing
from math import floor, ceil, radians, tan, sin, cos

sys.path.append(".")
sys.path.append("..")
from linelidarclass.linelidar import LineLidar, LLchr, \
					_ll_default_port, \
					_ll_default_ssh_python_path, \
					_ll_default_udp_timeout



## Classes
#

class shared_data:
  pass



class target:
  """Description of a target
  """

  def __init__(self, angle, dist):
    self.angle = angle
    self.dist = dist



## Routines
#

def on_canvas_resize(event, sd):
  """ Update the new canvas size
  """

  # Has the canvas size changed?
  if event.width != sd.canvas_width or event.height != sd.canvas_height:

    # Grab the new canvas size
    sd.canvas_width = event.width
    sd.canvas_height = event.height

    # Recalculate the distance (meters) -> pixels conversion factor
    sd.meters_to_pixels = sd.canvas_height / (sd.max_range + sd.vert_margins)

    # Recalculate the location of the device on the canvas in pixels
    sd.dev_x = round(sd.canvas_width * sd.device_x_loc)
    sd.dev_y = sd.canvas_height - round(sd.bottom_margin * sd.meters_to_pixels)

    # Rebuild / redraw the canvas
    sd.rebuild_canvas = True
    redraw_canvas(sd)

    # Update the display
    sd.root.update_idletasks()



def notifications_getter_thread(ll, notif_queue, expected_notifs, timeout):
  """Thread dedicated to getting notifications from the device
  """

  t = threading.current_thread()

  while getattr(t, "do_run", True):

    # Get one notification
    try:
      notif = ll.get_notification(expected_notifs, timeout = timeout)

    except Exception as e:
      notif_queue.put((None, e))

    # Send the notification down the notifications queue for processing
    notif_queue.put((notif, None))

  # Send the sentinel to indicate we're done
  notif_queue.put(None)



def process_device_notifications(sd):
  """Get notifications from the device, process them and update the display
  """

  start_tstamp = time()

  # Process the notifications queue until it's empty, so we don't let it fill
  # up if the machine is too slow.
  while True:

    # If the machine really is too slow, we can't catch up with the queue and
    # we haven't let the GUI run for too long, flush the queue, let the GUI
    # know it needs to display a warning and stop processing the notifications
    if time() - start_tstamp > 0.5:	# the GUI hasn't run for 0.5 s

      try:
        while True:
          sd.notif_queue.get_nowait()

      except queue.Empty:
        if not sd.notif_queue_overrun:
          sd.notif_queue_overrun_status_updated = True
        sd.notif_queue_overrun = True

        # Redraw the canvas
        redraw_canvas(sd)

        break

    # Get one notification from the notifications queue
    try:
      notif, sd.exc = sd.notif_queue.get_nowait()

    # Let the GUI run if the queue is empty
    except queue.Empty:
      if sd.notif_queue_overrun:
        sd.notif_queue_overrun_status_updated = True
      sd.notif_queue_overrun = False
      break

    except Exception as e:
      sd.exc = e

    # If the notifications getter thread sent us an exception or we got one
    # ourselves, stop the application
    if sd.exc is not None:
      sd.stop = True

    else:
      # Process the notification
      sd.last_notif_tstamp = time()

      # Did we get a RANGE notification?
      if notif.char == LLchr.RANGE:

        # If the current measurement ID exists and it's different from the
        # measurement ID in the notification, it means the device is sending us
        # a new list of targets for a new measurement cycle
        if sd.nlog_measid and sd.nlog_measid[-1] != notif.measurementid:

          sd.min_i_log_keep = sd.nlog_depth

          # Periodically update / recalculate the targets to display
          update_targets_from_notifications_log(sd)

          # Prune the notifications log
          sd.nlog_dists = sd.nlog_dists[sd.min_i_log_keep:]

          sd.nlog_measid = sd.nlog_measid[sd.min_i_log_keep:]
          sd.nlog_tstamps = sd.nlog_tstamps[sd.min_i_log_keep:]

          sd.nlog_depth -= sd.min_i_log_keep

          # Redraw the canvas
          redraw_canvas(sd)

        # Add the new notification to the notifications log
        dists = np.full((1, sd.nb_cal_angles), np.nan)

        for dist, angle, _ in notif.targets:
          dists[0][sd.cal_angles_i[angle]] = dist

        sd.nlog_dists = np.append(sd.nlog_dists, dists, 0)

        sd.nlog_measid.append(notif.measurementid)
        sd.nlog_tstamps.append(notif.timestamp.total_seconds())

        sd.nlog_depth += 1

  # Should we stop?
  if sd.stop:

    # Stop the notifications getter thread
    setattr(sd.notif_getter, "do_run", False)
    while sd.notif_queue.get() is not None:
      pass
    sd.notif_getter.join()

    # Redraw the canvas one last time
    redraw_canvas(sd)

    # Quit the GUI and close the window
    sd.root.quit()
    sd.root.destroy()

  else:

    # Update the display and reschedule the next call to ourselves
    sd.root.after(1, process_device_notifications, sd)



def update_targets_from_notifications_log(sd):
  """Periodically update / recalculate the list of targets to display from the
  notifications log
  """

  # Should we update the targets?
  if sd.nlog_tstamps[-1] >= sd.next_targets_update_tstamp:

    # Only use the head of the notifications log to determine the targets
    sd.targets = []
    i = sd.nlog_depth - 1
    while i >= 0 and sd.nlog_measid[i] == sd.nlog_measid[-1]:
      for j, dist in enumerate(sd.nlog_dists[i]):
        if not np.isnan(dist):
          sd.targets.append(target(sd.cal_angles[j], dist))
      i -= 1

    # Schedule the next targets update
    sd.next_targets_update_tstamp = sd.nlog_tstamps[-1] + \
						sd.update_targets_every

    # Signal that the flat list of targets to display has been updated
    sd.targets_updated = True



def redraw_canvas(sd):
  """Draw or redraw the background elements and the current targets in the
  canvas
  """

  # Fonction to convert meters to pixels
  m2px = lambda m: round(m * sd.meters_to_pixels)

  # Should we rebuild the entire canvas?
  if sd.rebuild_canvas:

    # Clear the entire canvas
    sd.canvas.delete("all")

    # Draw a measurement grid centered on the device
    for i in range(-floor(sd.max_range / sd.gds),
			ceil(sd.max_range / sd.gds), 1):
      i *= sd.gds

      sd.canvas.create_line(sd.dev_x + m2px(i), 0,
				sd.dev_x + m2px(i), sd.canvas_height - 1,
				width = 1, fill = "lightgrey")

      sd.canvas.create_text(sd.dev_x + m2px(i), sd.canvas_height - 1,
				font = (sd.default_font, m2px(0.35 * sd.gds)),
				fill = "grey", angle = 90, anchor = "sw",
				text = "{} m".format(abs(i)))

    for i in range(ceil(sd.min_range / sd.gds),
			ceil(sd.max_range / sd.gds) + 1, 1):
      i *= sd.gds

      sd.canvas.create_line(0, sd.dev_y - m2px(i),
				sd.canvas_width - 1, sd.dev_y - m2px(i),
				width = 1, fill = "lightgrey")

      sd.canvas.create_text(1, sd.dev_y - m2px(i),
				font = (sd.default_font, m2px(0.35 * sd.gds)),
				fill = "grey", anchor = "sw",
				text = "{} m".format(i))

    # Draw the limits of the viewing aperture as defined by the minimum and
    # maximum viewing angles, if they're within the calibrated angles aperture
    if sd.min_cal_angle < sd.min_view_angle < sd.max_cal_angle:
      sd.canvas.create_line(
			sd.dev_x + m2px(cos(radians(90 - sd.min_view_angle)) * \
					sd.min_range),
			sd.dev_y - m2px(sin(radians(90 - sd.min_view_angle)) * \
					sd.min_range),
			sd.dev_x + m2px(cos(radians(90 - sd.min_view_angle)) * \
					sd.max_range),
			sd.dev_y - m2px(sin(radians(90 - sd.min_view_angle)) * \
					sd.max_range),
			fill = "red", width = 2)

    if sd.min_cal_angle < sd.max_view_angle < sd.max_cal_angle:
      sd.canvas.create_line(
			sd.dev_x + m2px(cos(radians(90 - sd.max_view_angle)) * \
					sd.min_range),
			sd.dev_y - m2px(sin(radians(90 - sd.max_view_angle)) * \
					sd.min_range),
			sd.dev_x + m2px(cos(radians(90 - sd.max_view_angle)) * \
					sd.max_range),
			sd.dev_y - m2px(sin(radians(90 - sd.max_view_angle)) * \
					sd.max_range),
			fill = "red", width = 2)

    # Draw the limits of the viewing aperture as defined by the minimum and
    # maximum calibrated angles
    sd.canvas.create_line(sd.dev_x, sd.dev_y,
			sd.dev_x + m2px(cos(radians(90 - sd.min_cal_angle)) * \
					sd.max_range),
			sd.dev_y - m2px(sin(radians(90 - sd.min_cal_angle)) * \
					sd.max_range),
			fill = "black", width = 2)

    sd.canvas.create_line(sd.dev_x, sd.dev_y,
			sd.dev_x + m2px(cos(radians(90 - sd.max_cal_angle)) * \
					sd.max_range),
			sd.dev_y - m2px(sin(radians(90 - sd.max_cal_angle)) * \
					sd.max_range),
			fill = "black", width = 2)

    # Draw a grid of measurement arcs centered on the device
    for i in range(ceil(sd.min_range / sd.gds),
			floor(sd.max_range / sd.gds) + 1, 1):
      i *= sd.gds

      sd.canvas.create_arc(sd.dev_x - m2px(i), sd.dev_y - m2px(i),
				sd.dev_x + m2px(i), sd.dev_y + m2px(i),
				style = "arc",
				start = 90 - sd.max_cal_angle,
				extent = sd.max_cal_angle - sd.min_cal_angle,
				outline = "pink")

      sd.canvas.create_text(sd.dev_x + m2px(cos(radians(
						90 - sd.max_cal_angle)) * i),
				sd.dev_y - m2px(sin(radians(
						90 - sd.max_cal_angle)) * i),
				font = (sd.default_font, m2px(0.35 * sd.gds)),
				fill = "pink",
				angle = -sd.max_cal_angle,
				anchor = "w",
				text = " {} m".format(i))

    # Draw arcs to show the minimum and maximum range distances
    sd.canvas.create_arc(sd.dev_x - m2px(sd.min_range),
				sd.dev_y - m2px(sd.min_range),
				sd.dev_x + m2px(sd.min_range),
				sd.dev_y + m2px(sd.min_range),
				style = "arc",
				start = 90 - sd.max_cal_angle,
				extent = sd.max_cal_angle - sd.min_cal_angle,
				outline = "red")

    sd.canvas.create_arc(sd.dev_x - m2px(sd.max_range),
				sd.dev_y - m2px(sd.max_range),
				sd.dev_x + m2px(sd.max_range),
				sd.dev_y + m2px(sd.max_range),
				style = "arc",
				start = 90 - sd.max_cal_angle,
				extent = sd.max_cal_angle - sd.min_cal_angle,
				outline = "red")

    # Force the instructional lines to be updated
    sd.update_instructions = True

  # Update the instructional lines if needed
  if sd.update_instructions or sd.rebuild_canvas:

    sd.canvas.delete("instructions")

    lines = ("Q to quit",)

    # Print the lines
    for i, l in enumerate(lines):
      sd.canvas.create_text(sd.canvas_width - m2px(0.5 * sd.gds),
				sd.dev_y - m2px((1 - i) * sd.gds),
				font = (sd.default_font, m2px(0.5 * sd.gds)),
				fill = "black", anchor = "se",
				text = l, tags = "instructions")

    sd.update_instructions = False

  # Draw the targets as a points cloud
  if (sd.targets_updated or sd.rebuild_canvas) and sd.targets is not None:

    sd.canvas.delete("targets")

    ptradius = m2px(0.02 * sd.gds)

    for t in sd.targets:

      ra = radians(90 - t.angle)

      dist_x = cos(ra) * t.dist
      dist_y = sin(ra) * t.dist

      target_x = sd.dev_x + m2px(dist_x)
      target_y = sd.dev_y - m2px(dist_y)

      # Pick a color for the target
      color = "#0000FF"

      # Draw the target as a dot
      sd.canvas.create_oval(target_x - ptradius, target_y - ptradius,
				target_x + ptradius, target_y + ptradius,
				outline = color, fill = color,
				tags = "targets")

    sd.targets_updated = False

  # Update the notifications queue overrun status if needed
  if sd.notif_queue_overrun_status_updated or sd.rebuild_canvas:

    sd.canvas.delete("overrun")

    if sd.notif_queue_overrun:
      sd.canvas.create_text(sd.canvas_width - m2px(0.5 * sd.gds),
				0,
				font = (sd.default_font, m2px(0.5 * sd.gds)),
				fill = "red", anchor = "ne",
				text = "Buffer overrun!",
				tags = "overrun")

  sd.rebuild_canvas = False



def set_stop_flag(sd):
  """Callback to set the stop flag to trigger the shutdown of the application
  """

  sd.stop = True



## Main program
#

def show_targets(linelidar, freq):
  """Main application entry point: create the display then run the display's
  main loop which will periodially call the function to process the device's
  messages.
  """

  # Initialize shared data
  sd = shared_data()

  # Flag to stop the application, and exception if it got stopped abnormally
  sd.stop = False
  sd.exc = None

  # Copy the parameters that the callbacks need into the shared data
  sd.ll = linelidar
  sd.freq = freq

  # Get the minimum and maximum ranging distances
  sd.min_range = sd.ll.read_chr(LLchr.MIN_DISTANCE).distance
  sd.max_range = sd.ll.read_chr(LLchr.MAX_DISTANCE).distance

  # Get the minimum and maximum viewing angles
  sd.min_view_angle = sd.ll.read_chr(LLchr.MIN_ANGLE).angle
  sd.max_view_angle = sd.ll.read_chr(LLchr.MAX_ANGLE).angle

  # Calculate the size of a grid division so we have a number of grid lines
  # that's appropriate for the maximum distance, and so the text elements are
  # scaled properly
  sd.gds = 16				# Start with 16 meters per division
  while sd.max_range / sd.gds < 15:	# As long as we have < 15 grid lines,
    sd.gds /= 2				# halve the grid division

  # Get the calibrated angles
  sd.cal_angles = tuple(sorted(set(
			sd.ll.read_chr(LLchr.CALIBRATED_ANGLES).angles)))
  sd.cal_angles_i = {a: i for i, a in enumerate(sd.cal_angles)}
  sd.nb_cal_angles = len(sd.cal_angles)

  # The minimum and maximum calibrated angles are the device's aperture limits
  sd.min_cal_angle = min(sd.cal_angles)
  sd.max_cal_angle = max(sd.cal_angles)

  # Calculate the margins around the canvas
  sd.top_margin = 0.2 * sd.gds
  sd.bottom_margin = 1.5 * sd.gds
  sd.vert_margins = sd.top_margin + sd.bottom_margin
  side_margins = 3 * sd.gds

  # Work out the window's aspect ratio, and where the device is located on the
  # bottom edge of the canvas
  ar_x = sd.max_range * (tan(radians(sd.max_cal_angle)) - \
				tan(radians(sd.min_cal_angle)))
  sd.device_x_loc = sd.max_range * -tan(radians(sd.min_cal_angle)) / ar_x

  ar_x = round(ar_x + side_margins * 2)
  ar_y = round(sd.max_range + sd.top_margin + sd.bottom_margin)

  # Create the window
  sd.root = tk.Tk()
  sd.root.title("LineLidar targets")

  sd.root.aspect(ar_x, ar_y, ar_x, ar_y)
  sd.root.minsize(minimum_window_size, round(minimum_window_size * ar_y / ar_x))

  sd.canvas = tk.Canvas(sd.root, bg = "white")
  sd.canvas.pack(fill = tk.BOTH, expand = tk.YES)

  sd.canvas_width = sd.canvas.winfo_reqwidth()
  sd.canvas_height = sd.canvas.winfo_reqheight()

  # Get the default font name
  sd.default_font = tk.Label(sd.root)["font"]

  # Bind the Configure event to catch resizing events
  sd.root.bind("<Configure>", lambda event: on_canvas_resize(event, sd))

  # Bind the window manager "delete window" event
  sd.root.protocol("WM_DELETE_WINDOW", lambda arg = sd: set_stop_flag(arg))

  # Bind keys
  sd.root.bind("q", lambda event: set_stop_flag(sd))
  sd.root.bind("Q", lambda event: set_stop_flag(sd))

  # calculate the initial distance (meters) -> pixels conversion factor
  sd.meters_to_pixels = sd.canvas_height / (sd.max_range + sd.vert_margins)

  # Work out the initial location of the device on the canvas in pixels
  sd.dev_x = round(sd.canvas_width * sd.device_x_loc)
  sd.dev_y = sd.canvas_height - round(sd.bottom_margin * sd.meters_to_pixels)

  # If the frequency isn't specified, use the default frequency
  if sd.freq is None:
    sd.freq = ll_default_frequency

  sd.expected_notifications = []

  # Disable zero results reporting, enable notification on RANGE and start
  # active ranging at the desired rate
  sd.ll.report_zero_results(False)
  sd.ll.enable_notification(LLchr.RANGE)
  sd.expected_notifications.append(LLchr.RANGE)
  sd.ll.set_sampling_rate(sd.freq)

  sd.notif_timeout = _ll_default_udp_timeout

  # Spawn a thread dedicated to getting notifications from the device and
  # sending them to the processing routine, so we don't miss UDP packets if the
  # processing routine runs slow
  sd.notif_queue = multiprocessing.Queue()
  sd.notif_getter = threading.Thread(target = notifications_getter_thread,
					args = (sd.ll,sd.notif_queue,
						sd.expected_notifications,
						max(sd.notif_timeout,
							1.5 / sd.freq)))
  sd.notif_getter.start()

  # Notifications log
  sd.nlog_dists = np.empty(shape = (0, sd.nb_cal_angles))
  sd.nlog_measid = []
  sd.nlog_tstamps = []
  sd.nlog_depth = 0

  sd.next_targets_update_tstamp = 0

  sd.update_targets_every = update_targets_every

  sd.targets = None
  sd.targets_updated = False

  sd.notif_queue_overrun = False
  sd.notif_queue_overrun_status_updated = False

  sd.rebuild_canvas = True

  # Run the display's main loop, getting messages from the device when it's idle
  sd.root.after(0, process_device_notifications, sd)
  sd.root.mainloop()

  # Stop any active ranging
  try:
    sd.ll.stop_sampling()
  except:
    pass

  # Disable notification on all characteristics
  try:
    sd.ll.disable_all_notifications()
  except:
    pass

  # If the application stopped with an exception, re-raise it
  if sd.exc is not None:
    raise sd.exc



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
	  type = int
	)

  args = argparser.parse_args()

  # Create a LineLidar class instance
  ll = LineLidar()

  # Open communication with the device
  ll.open(addr = args.address, port = args.port,
		sshcmd = args.sshcmd, sshpypath = args.ssh_python_path)

  try:
    # Run the display application
    show_targets(ll, args.frequency)

  finally:
    # Close communication with the device
    ll.close()
