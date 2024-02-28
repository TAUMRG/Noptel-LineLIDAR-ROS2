#!/usr/bin/python3
"""LineLidar communication class
Copyright 2023 (c) Noptel Oy, Oulu Finland

Sample application:
  Example of a multithreaded LineLidar application whereby a dedicated thread
  takes care of the communication with the LineLidar while the main thread is
  free to do other things

Requires:
  - Python3
  - Python3 LineLidar class

Tested on:
  - Linux
  - Windows

Authors:
  PCo

Releases:
  Version 1.0.0 - 23/01/2024 - Initial release
"""

## Modules
#

import sys
import argparse
from time import sleep

import queue
import threading
import multiprocessing

sys.path.append(".")
sys.path.append("..")
from linelidarclass.linelidar import LineLidar, LLchr



## Routines
#

def linelidar_comm_thread(msg_queue, address, frequency):
  """ Handle communication with the LineLidar and send the targets to the main
  thread through the message queue
  """

  t = threading.current_thread()

  try:

    # Open communication with the device
    with LineLidar(addr = address) as ll:

      targets = []
      last_measurementid = None

      # Disable zero results reporting, enable notification on RANGE and start
      # active ranging
      ll.report_zero_results(False)
      ll.enable_notification(LLchr.RANGE)
      ll.set_sampling_rate(frequency)

      # Get notifications as long as we're not told to stop
      while getattr(t, "do_run", True):

        # Get one RANGE notification
        notif = ll.get_notification()

        # Has the measurement ID changed?
        if last_measurementid is not None and \
		notif.measurementid != last_measurementid:

          # Send the last measurement's ID and targets to the main process
          msg_queue.put((last_measurementid, targets))

          # Clear the list of targets
          targets = []

        # Add the notification's target to this measurement's targets
        targets.extend(notif.targets)

        # Save the notification's measurement ID
        last_measurementid  = notif.measurementid

      # Stop sampling
      ll.stop_sampling()

  except Exception as e:

    # The communication with the LineLidar resulted in an error: notify the main
    # process by sending it the exception
    msg_queue.put(e)



## Main program
#

if __name__ == "__main__":

  # Parse the command line arguments
  argparser = argparse.ArgumentParser()

  argparser.add_argument(
	  "-a", "--address",
	  help = "IP address of the LineLidar",
	  type = str,
	  required = True
	)

  argparser.add_argument(
	  "-f", "--frequency",
	  help = "Sampling rate. Default: 10 Hz",
	  type = int,
	  default = 10
	)

  args = argparser.parse_args()

  # Spawn the thread to communicate with the device
  q = multiprocessing.Queue()
  t = threading.Thread(target = linelidar_comm_thread,
			args = (q, args.address, args.frequency))
  t.start()

  try:

    # Do something forever and display the number of targets detected by the
    # LineLidar when we get some
    while True:

      # Do something that takes some time
      print("Let's count to 5 in the main thread:")
      for counter in range(1, 6):
        print(counter, " ", end = "")
        sys.stdout.flush()
        sleep(0.1)
      print()

      # Process messages from the communication thread
      while True:

        # Try to get one message out of the queue
        try:
          msg = q.get_nowait()

        except queue.Empty:
          break

        # If we got an exception from the communication thread, print the
        # exception and raise an exception in the main thread to stop it
        if isinstance(msg, Exception):
          retcode = -1
          print("{} exception in communication thread{}".
		format(type(msg).__name__,
			(": {}".format(msg) if str(msg) else "")))
          raise RuntimeError("COMM_THREAD_EXC")

        # Display how many targets we got
        print("Received measurement ID #{} from the communication thread: "
		"{} targets".format(msg[0], len(msg[1])))

  except KeyboardInterrupt:

    # The user interrupted the program
    retcode = 0

    print()
    print("Interrupted")

  except Exception as e:

    # Either the main thread or the communication thread encountered an error
    retcode = -1

    # Did the main thread encounter an error?
    if not (isinstance(e, RuntimeError) and str(e) == "COMM_THREAD_EXC"):

      # Print the exception
      print("{} exception in main thread{}".
		format(type(e).__name__,
			(": {}".format(e) if str(e) else "")))

  finally:

    # Tell the communication thread to stop if it hasn't stopped by itself
    # already
    setattr(t, "do_run", False)

    # Join the thread
    t.join()

    # Exit with the return code
    sys.exit(retcode)
