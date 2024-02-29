"""! %LineLidar Low-level communication class

@file linelidarclass.py

@mainpage Python3 %LineLidar class

@section main_page_description Description
Low-level class to communicate with a %LineLidar device in Python

LineLidar communication class submodule

@section main_page_notes Notes
Copyright 2023 (c) Noptel Oy, Oulu Finland

Tested on:
  - Linux
  - Windows

Authors:
  PCo
"""

from __future__ import annotations



## Modules
#

import re
import sys
import shlex
import pickle
import select
import socket
import psutil						# type: ignore
import subprocess
from time import time, sleep
from struct import pack, unpack
from types import TracebackType
from base64 import b64encode, b64decode
from datetime import datetime, timedelta
from typing import List, Tuple, Dict, Type, Union, Optional, Callable, IO

## Platform-dependent
if sys.platform[0:3] == "win":
  import queue
  import threading
  import multiprocessing

  try:
    multiprocessing.set_start_method("spawn")
  except RuntimeError as e:
    if str(e) == "context has already been set":
      pass
    else:
      raise



## Default parameters
# @defgroup DefaultParameters Default parameters

from .default import _ll_default_port, \
			_ll_default_udp_timeout, \
			_ll_default_ssh_timeout, \
			_ll_default_reboot_timeout, \
			_ll_default_retries, \
			_ll_default_sent_cmds_log_depth, \
			_ll_default_ssh_python_path



## Base classes
# @defgroup BaseClasses Base classes

from . import ipaddress, macaddress, _LLcmd, _LLsrv, _LLsta, _LLchr



## Enums
# @defgroup Enums Enums

from . import LLcmd, LLsrv, LLchr, LLsta



## Type definitions
#

## @cond # Excluded from Doxygen

from . import _SERVICE_INFO_TYPE, _CHR_DECODED_ENTRY_TYPE, _CHR_ARGS_TYPE, \
		_ENC_DEC_FCTS_VAL_TYPE, _CHR_INFO_TYPE

## ICD descriptors
#

from . import _LLSTA_CMDS, _CHR_FTYPE, _SRV_DESC, _CHR_DESC

##\endcond # /Excluded from Doxygen



# ## Remote SSH relay stub
# Note: this stub is coded for brevity rather than clarity, to fit as
#       tightly as possible as an argument on the command line after
#       shortening / encoding

ssh_relay_exec: Tuple[str, ...] = (
	r"## Modules",
	r"#",

	r"import threading as T, sys, socket as S, pickle as P, base64 as B",

	r"## Main program",
	r"#",

	r"# Open communication with the device (N.B.: the default family for",
	r"# socket() is AF_INET, so we can omit it) and initialize a few",
	r"# other variables here to save a few bytes",
	r"s, e, l = S.socket(type = S.SOCK_DGRAM), [], 1",

	r"# Optional line to setup the socket for broadcasting.",
	r"# This line is stripped out when preparing the relay stub to talk",
	r"# to a single host",
	r"s.setsockopt(S.SOL_SOCKET, S.SO_BROADCAST, 1)",

	r"# Thread to relay LineLidar packets back to the client",
	r"#",

	r"def t():",

	r"  # The first line to send back to the client is the ready marker",
	r"  L = '\nLLRDY'",

	r"  # Set the socket timeout so recvfrom() times out every second, so",
	r"  # we have an opportunity to regularly check whether the thread",
	r"  # should stop",
	r"  s.settimeout(1)",

	r"  # Run as long as we last read a line from stdin - i.e. the SSH",
	r"  # connection is still up - or we have cleanup packets to send to",
	r"  # the LineLidar before quitting",
	r"  while l or e:",

	r"    # Send the latest line to stdout. If we run for the first time",
	r"    # this will send the ready marker to the client",
	r"    if L: print(L); L = sys.stdout.flush()",

	r"    # Try to get a UDP packet from the LineLidar for 1 second, then",
	r"    # pickle, base64-encode it and send it to the client",
	r"    try: L = B.b64encode(P.dumps(s.recvfrom(4096)))",
	r"    except: pass",

	r"# Start the thread",
	r"T.Thread(target = t).start()",

	r"# Run as long as we last read a line from stdin - i.e. the SSH",
	r"# connection is still up - or we have cleanup packets to send to",
	r"# the LineLidar before quitting",
	r"while l or e:",

	r"  # Read a line from stdin",
	r"  l = sys.stdin.readline()",

	r"  # If we got a line, evaluate it back into a list of commands",
	r"  if l: e = P.loads(B.b64decode(l))",

	r"  # Relay the last command in the list if to the LineLidar and pop",
	r"  # it from the list if the list of commands isn't empty",
	r"  if e: s.sendto(e.pop(), ('{addr}', {port}))")



## Main classes
# @defgroup MainClasses Main classes

from . import LLresponse



class LineLidar:
  """! Main %LineLidar class

  @ingroup MainClasses
  """

  # ## Variables
  ## Address and port of the device or network
  __addrport: Tuple[str, int]

  ## Netmask to filter incoming UDP replies by IP - Default: /32 (single host)
  _netmask: int = 0xffffffff

  ## UDP socket to communicate with the device
  __socket: Optional[socket.socket] = None
  __socket = None

  ## SSH client process
  __sshproc: Optional[subprocess.Popen]
  __sshproc = None

  ## Windows-only queue to read the SSH client's stdout in non-blocking mode
  __sshproc_win_stdout: Optional[multiprocessing.queues.Queue]
  __sshproc_win_stdout = None

  ## Windows-only thread to read the SSH client's stdout in non-blocking mode
  __win_reader_thread: Optional[threading.Thread]
  __win_reader_thread = None

  ## SSH receive buffer
  __ssh_recvbuf: bytes

  ## Default communication timeout in seconds
  __default_timeout: float

  ## Current communication timeout in seconds
  __timeout: float

  ## Whether to automatically stop ranging upon closing the device
  __autostop_ranging: bool

  ## Whether the device is currently ranging
  __is_ranging: bool

  ## last sent message ID
  __msgid: int

  ## Sent commands log depth
  _sent_cmds_log_depth: int

  ## Sent commands log
  __sent_cmds_log: Tuple[List[Tuple[int, _LLcmd]], List[Union[_LLsrv, _LLchr]]]

  ## How many times a failed command should be retried for fault-tolerance
  retries: Optional[int] = None

  ## Notifications stack
  notifications: List

  ## Debugging messages toggle
  _debug: bool

  ## Enum-by-value mappings
  _val_to__LLcmd: Dict[int, _LLcmd] = \
			{getattr(LLcmd, a).value: getattr(LLcmd, a) \
				for a in dir(LLcmd) \
				if isinstance(getattr(LLcmd, a), _LLcmd)}
  _val_to__LLsrv: Dict[int, _LLsrv] = \
			{getattr(LLsrv, a).value: getattr(LLsrv, a) \
				for a in dir(LLsrv) \
				if isinstance(getattr(LLsrv, a), _LLsrv)}
  _val_to__LLsta: Dict[int, _LLsta] = \
			{getattr(LLsta, a).value: getattr(LLsta, a) \
				for a in dir(LLsta) \
				if isinstance(getattr(LLsta, a), _LLsta)}
  _val_to__LLchr: Dict[Tuple[_LLsrv, int], _LLchr] = \
			{getattr(LLchr, a).value: getattr(LLchr, a) \
				for a in dir(LLchr) \
				if isinstance(getattr(LLchr, a), _LLchr)}



  # ## Methods
  def __init__(self: LineLidar,
		addr: Optional[str] = None,
		port: Optional[int] = None,
		sshcmd: Optional[str] = None,
		sshpypath: Optional[str] = None,
		sent_cmds_log_depth: Optional[int] = None,
		retries: int = None,
		set_clean_state: bool = True,
		autostop_ranging: bool = True,
		debug: bool = False,
		timeout: Optional[float] = None) \
		-> None:

    """! %__init__ method

    @param addr IP address of the device. If unspecified, the device is not
		automatically opened.
    @param port Port of the device (if the device is automatically opened)
    @param sshcmd If specified, ssh command to log into a shell account to use
                  the host as a relay to talk to the LineLidar(if the device is
                  automatically opened). Works with a Linux or Windows host
                  with sshd and Python 2 or 3 installed.
    @param sshpypath Path of the Python executable on the SSH relay host (if
                     the device is automatically opened)
    @param sent_cmds_log_depth Number of sent commands tracked, to ignore
                               out-of-order responses that have already
                               generated a timeout (if the device is
                               automatically opened). Set to 1 to disable
                               filtering out out-of-order UDP packets.
    @param retries How many times a failed command should be retried (if the
                   device is automatically opened)
    @param set_clean_state Set device in a known, stopped state after opening
                           (if the device is automatically opened)
    @param autostop_ranging: Enable or disable automatically stopping ranging
                             upon closing the device (if the device is
                             automatically opened)
    @param debug Enable or disable debugging messages
    @param timeout Communication timeout in seconds (if the device is
                   automatically opened)
    """

    ## Debugging messages toggle
    self._debug = debug

    # If the address is specified, open communication with the device rightaway
    if addr is not None:
      self.open(addr,
		port,
		sshcmd,
		sshpypath,
		sent_cmds_log_depth = sent_cmds_log_depth,
		retries = retries,
		set_clean_state = set_clean_state,
		autostop_ranging = autostop_ranging,
		timeout = timeout)



  def __enter__(self: LineLidar) \
		-> LineLidar:
    """! %__enter__ method
    """

    return self



  def __exit__(self: LineLidar,
		exc_type: Optional[Type[BaseException]],
		exc_value: Optional[BaseException],
		exc_traceback: Optional[TracebackType]) \
		-> None:
    """! %__exit__ method
    """

    self.close()



  def __del__(self: LineLidar) \
		-> None:
    """! %__del__ method
    """

    self.close()



  def __highlighted_bytes(self: LineLidar,
				b: bytes,
				start: Optional[int] = None,
				end: Optional[int] = None) \
				-> str:
    """! Generate a printable byte sequence in hex with a slice of it
    highlighted

    @param b Byte sequence
    @param start Start of the byte sequence slice to highlight
    @param end End of the byte sequence slice to highlight

    @return Printable hex byte sequence
    """

    s: str = " ".join(["{:02x}".format(v) for v in b])

    if start is None or end is None or start < 0 or end <= start:
      return s

    return (s[0 : start * 3] + ">>>" + s[start * 3: end * 3 - 1] + "<<< " + \
		s[end * 3:]).rstrip()



  def open(self: LineLidar,
		addr: str,
		port: Optional[int] = None,
		sshcmd: Optional[str] = None,
		sshpypath: Optional[str] = None,
		sent_cmds_log_depth: Optional[int] = None,
		retries: int = None,
		set_clean_state: bool = True,
		autostop_ranging: bool = True,
		timeout: Optional[float] = None) \
		-> Union[socket.socket,Tuple[subprocess.Popen, Optional[
						multiprocessing.queues.Queue]]]:
    """! Open communication with a %LineLidar

    @param addr IP address of the device
    @param port Port of the device
    @param sshcmd If specified, ssh command to log into a shell account to use
                  the host as a relay to talk to the LineLidar. Works with a
                  Linux or Windows host with sshd and Python 2 or 3 installed.
    @param sshpypath Path of the Python executable on the SSH relay host
    @param sent_cmds_log_depth Number of sent commands tracked, to ignore
                               out-of-order responses that have already
                               generated a timeout. Set to 1 to disable
                               filtering out out-of-order UDP packets.
    @param retries How many times a failed command should be retried
    @param set_clean_state Set device in a known, stopped state after opening
    @param autostop_ranging: Enable or disable automatically stopping ranging
                             upon closing the device
    @param timeout Communication timeout in seconds

    @return Open UDP socket if the communication with the device is direct, or
            the tuple (open SSH client process, Windows-only stdout queue) if
            the communication with the device goes through an SSH relay host.
            On Linux machines, the Windows-only stdout queue is always None.
            On Windows machine, it is a multiprocessing.Queue() object from
            which the SSH process' stdout can be read in non-blocking mode
            instead of the process' stdout handle, which is always blocking.
    """

    assert sent_cmds_log_depth is None or  sent_cmds_log_depth >= 1
    assert retries is None or retries >= 0

    # Make sure neither a communication socket already exists nor a SSH client
    # has already been spawned
    if self.__socket is not None or self.__sshproc is not None:
      raise RuntimeError("Communication with the LineLidar is already open")

    # If no port was specified, use the default port
    if port is None:
      port = _ll_default_port

    ## Default communication timeout in seconds
    self.__default_timeout = (_ll_default_udp_timeout if sshcmd is None else \
				_ll_default_ssh_timeout) if timeout is None \
				else timeout

    ## Current communication timeout in seconds
    self.__timeout = self.__default_timeout

    # Resolve the address
    addr, port = socket.getaddrinfo(addr, port)[0][4]
    assert isinstance(port, int)

    ## Whether to automatically stop ranging upon closing the device
    self.__autostop_ranging = autostop_ranging

    ## Whether the device is currently ranging
    self.__is_ranging = False

    # Open communication with the device directly
    if sshcmd is None:
      self.__socket = socket.socket(family = socket.AF_INET,
					type = socket.SOCK_DGRAM)

      # Setup the socket for broadcasting if the netmask isn't /32 (single host)
      if self._netmask != 0xffffffff:
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Open communication with the device through a SSH relay host
    else:

      ## SSH receive buffer
      self.__ssh_recvbuf = b""

      # If no path for the Python executable on the SSH host was specified,
      # use the default path
      if sshpypath is None:
        sshpypath = _ll_default_ssh_python_path

      # Shorten / encode the remote SSH relay stub to be executable on the
      # command line with python -c
      l: str
      cls: List[str] = []

      for l in ssh_relay_exec:

        # Strip out the line to setup the socket for broadcasting if we should
        # leave the socket setup to talk to a single host (netmask /32)
        if self._netmask == 0xffffffff and "BROADCAST" in l:
          continue

        l = re.sub("\s\s", " ", l).rstrip(" \t\r\n")	# Zap unnecessary spaces
        l = re.sub(r", ", r",", l)			# Zap unnecessary spaces
        l = re.sub(r" =", r"=", l)			# Zap unnecessary spaces
        l = re.sub(r"= ", r"=", l)			# Zap unnecessary spaces
        l = re.sub(r" !", r"!", l)			# Zap unnecessary spaces
        l = re.sub(r": ", r":", l)			# Zap unnecessary spaces
        l = re.sub(r"; ", r";", l)			# Zap unnecessary spaces
        l = re.sub(r" \* ", r"*", l)			# Zap unnecessary spaces
        l = re.sub(r" \+ ", r"+", l)			# Zap unnecessary spaces
        l = re.sub(r"\\n", r"\\\\\\n", l)

        if not l or re.match("^\s*#.*$", l):
          continue

        cls.append(l)

      relaycmd: str = 'unset HISTFILE\n{} -c "exec(\\"{}\\")"\n'.\
			format(sshpypath,
				r"\n".join(cls).format(addr = addr,
							port = port))

      # Split the supplied non-interactive SSH command into individual arguments
      # the way the shell would
      split_sshcmd: List[str] = shlex.split(sshcmd)

      # If the shell command is directly "ssh" and we don't have any "-t" or
      # "-tt" in the command line, append the "-T" argument to suppress the
      # warning on stderr about not allocating a pseudo terminal because stdin
      # is not a terminal
      if split_sshcmd[0] == "ssh" and \
		not [a for a in split_sshcmd if a[0:2] == "-t"]:
        split_sshcmd = split_sshcmd[0:1] + ["-T"] + split_sshcmd[1:]

      # Open the SSH session and start the remote relay command
      try:

        # Spawn the non-interactive SSH command process
        self.__sshproc = subprocess.Popen(split_sshcmd, bufsize = 0,
						stdin = subprocess.PIPE,
						stdout = subprocess.PIPE,
						stderr = None if self._debug \
							else subprocess.DEVNULL)

        # If we run in Windows, spawn a thread to read the data from the
        # client's blocking stdout and send it to a queue that can be read
        # with a timeout
        if sys.platform[0:3] == "win":

          # Create a queue for the reader thread to send the SSH process' stdout
          # back to us
          self.__sshproc_win_stdout = multiprocessing.Queue()

          # Spawn the reader thread
          self.__win_reader_thread = threading.Thread(
					target = self.__windows_reader_thread,
					args = (self.__sshproc.stdout,
					self.__sshproc_win_stdout))
          self.__win_reader_thread.start()

        # Send the relay command to execute remotely
        self.__sshproc.stdin.write(relaycmd.encode("ascii"))

        # Wait until we get the ready marker from the SSH relay stub
        while self.__recv_ssh_line() != "LLRDY":
          pass

      # In case of failure, close everything and re-raise the exception
      except:
        self.close()
        raise

    ## Address and port of the device or network
    self.__addrport = (addr, port)

    ## Sent commands log depth
    self._sent_cmds_log_depth = _ll_default_sent_cmds_log_depth \
					if sent_cmds_log_depth is None else \
				sent_cmds_log_depth

    ## Sent commands log
    self.__sent_cmds_log = ([], [])

    ## last sent message ID
    self.__msgid = 0

    ## How many times a failed command should be retried for fault-tolerance
    self.retries = _ll_default_retries if retries is None else retries

    ## Notifications stack
    self.notifications = []

    # Should we leave the device in a known, stopped state after opening?
    if set_clean_state:

      # Try to issue the commands needed to set the device in a clean state
      try:
        self.set_clean_state()

      # In case of failure, close everything and re-raise the exception
      except:
        self.close()
        raise

    if self.__socket is not None:
      return self.__socket

    assert self.__sshproc is not None
    return (self.__sshproc, self.__sshproc_win_stdout)



  def close(self: LineLidar) \
		-> None:
    """! Close communication with the %LineLidar
    """

    # If needed, stop the device ranging by writing a frequency of 0 Hz to
    # MEASURING_RATE blindly several times
    if (self.__socket is not None or self.__sshproc is not None) and \
		self.__autostop_ranging and self.__is_ranging:
      for _ in range(3):
        self._send_cmd(LLcmd.WRITE, LLchr.MEASURING_RATE, frequency = 0)

    # If we have an open UDP socket, close it
    if self.__socket is not None:
      self.__socket.close()
      self.__socket = None

    # If we have a Windows reader thread, tell it to stop after the pipe to
    # the SSH client is broken (next, when we kill the SSH client)
    if self.__win_reader_thread is not None and \
		self.__sshproc_win_stdout is not None:
      setattr(self.__win_reader_thread, "do_run", False)

    # If we have a running SSH client, kill it
    if self.__sshproc is not None:
      self.__sshproc.kill()
      self.__sshproc.wait()
      self.__sshproc = None

    retries = None

    # If we have a Windows reader thread, wait for it to indicate it's stopped
    # then join() it
    exc: Optional[Exception] = None
    if self.__win_reader_thread is not None and \
		self.__sshproc_win_stdout is not None:

      while exc is None:
        _, exc = self.__sshproc_win_stdout.get()

      self.__win_reader_thread.join()
      self.__win_reader_thread = None

      # Stop and delete the queue
      self.__sshproc_win_stdout.close()
      del(self.__sshproc_win_stdout)
      self.__sshproc_win_stdout = None

      # If the Windows reader thread sent us an exception other than the
      # sentinel we expected, re-raise it
      if exc is not None and exc != RuntimeWarning:
        raise exc



  def _send_data(self: LineLidar,
			data: Union[bytes, List[bytes]]) \
			-> None:
    """! Send data to the %LineLidar

    @param data Data to send in the form of a byte array or a list of
                byte arrays

    If the data is sent to the LineLidar directly, the data in the form of a
    byte array is sent.

    If the data is sent to the LineLidar through a SSH relay, if the data is
    in the form of a byte array, it is first encapsulated in a list. Then the
    list of byte array(s) is pickled, base-64 encoded and sent to the SSH
    relay stub.
    """

    # Make sure either a communication socket exists or a SSH client has
    # been spawned
    if self.__socket is None and self.__sshproc is None:
      raise RuntimeError("Communication with the LineLidar is not open")

    # Send the data to the LineLidar directly...
    if self.__socket is not None:
      assert isinstance(data, bytes)
      self.__socket.sendto(data, self.__addrport)

    # ...or send the command to the LineLidar through a SSH relay
    elif self.__sshproc is not None:

      # If the data is in the form of a byte array, encapsulate it into a list
      if isinstance(data, bytes):
        data = [data]

      # Pickle, base64-encode and send the list to the SSH relay stub
      self.__sshproc.stdin.write(b64encode(pickle.dumps(data, protocol = 2)) \
					+ b"\n")
      self.__sshproc.stdin.flush()



  def __encode_cmd(self: LineLidar,
			msgid: int,
			cmd: _LLcmd,
			chr_or_srv: Union[_LLsrv, _LLchr],
			*args: bool,
			**kwargs: _CHR_ARGS_TYPE) \
			-> Tuple[bytes, Optional[str]]:
    """! Encode a %LineLidar command.

    @param msgid Message ID
    @param cmd Command to encode
    @param chr_or_srv Characteristic or service
    @param args Positional arguments (here, only True or False allowed -
		see below)
    @param kwargs Keyworded arguments (see below)

    If the command is SAVE_SERVICE or RESTORE_SERVICE, pass a service (_LLsrv)
    in chr_or_srv. Otherwise pass a characteristic (_LLchr)

    If the command is SET_NOTIFICATION, pass a single argument True or False to
    enable or disable notification - e.g.
    _encode_cmd(LLcmd.SET_NOTIFICATION, LLchr.TEMPERATURE, True)

    If the command is WRITE, pass the relevant parameters as keyworded
    arguments - e.g.
    _encode_cmd(LLcmd.WRITE, LLchr.MIN_DISTANCE, distance = 3.5)

    @return (Encoded command, optional debug message)
    """

    debugmsg: Optional[str]
    if self._debug:
      debugmsg = ">LL: cmd: {}, msgid: {}".format(cmd.name, msgid)
    else:
      debugmsg = None

    uint_pack: Callable = lambda i: pack("<I", i)

    # Compose the start of the command bytes
    cmddata: bytes = uint_pack(cmd.value) + uint_pack(msgid)

    # Does the command concern a characteristic?
    if cmd in (LLcmd.READ, LLcmd.WRITE, LLcmd.SET_NOTIFICATION):

      # Make sure chr_or_srv is a characteristic
      if not isinstance(chr_or_srv, _LLchr):
        raise ValueError("Target type {} incorrect - should be {}"
				.format(type(chr_or_srv), _LLchr))

      # Make sure the characteristic supports the command
      if cmd not in _CHR_DESC[chr_or_srv]["supported_cmds"]:
        raise ValueError("{} command not supported for characteristic {}".
				format(cmd.name, chr_or_srv.name))

      # Add the service and characteristic to the command bytes
      cmddata += uint_pack(chr_or_srv.value[0].value)
      cmddata += uint_pack(chr_or_srv.value[1])

      if debugmsg is not None:
        debugmsg += ", srv: {}, chr: {}".format(chr_or_srv.value[0].name,
						chr_or_srv.name)

      # If the command is SET_NOTIFICATION, add the enabled flag to the command
      # bytes
      if cmd == LLcmd.SET_NOTIFICATION:

        if not args in ((True,), (False,)) or kwargs != {}:
          raise RuntimeError("Sole positional argument True or False required "
				"for SET_NOTIFICATION command "
				"in characteristic {}".format(chr_or_srv.name))

        cmddata += uint_pack(1 if args[0] else 0)

        if debugmsg is not None:
          debugmsg += ", notification {}abled".format("en" if args[0] else \
							"dis")

      # If the command is READ, make sure no parameters were passed
      elif cmd == LLcmd.READ:

        if args or kwargs:
          raise RuntimeError("No arguments required for READ command")

      # If the command is WRITE, add the relevant parameters to the command
      # bytes
      elif cmd == LLcmd.WRITE:

        if args:
          raise RuntimeError("Positional arguments invalid for WRITE command")

        # Encode the parameters listed in the characteristic's description
        f: Dict[str, _ENC_DEC_FCTS_VAL_TYPE]
        for f in _CHR_DESC[chr_or_srv]["fields"] + ({"": None},):

          pname: str = list(f)[0]
          pname_expr: Optional[str] = None

          # Have we reached the terminator?
          if not pname:
            break

          # Get the accepted argument types and the encoding function for this
          # parameter
          acc_arg_types: Tuple[_CHR_DECODED_ENTRY_TYPE, ...] = f[pname][0]
          encoding_fct: Callable = f[pname][1]

          # Is there an expression between [...] at the end of the parameter
          # name to specify the number of parameter elements to encode?
          if pname[-1] == "]":

            # Split the parameter name and expression
            j: int = pname.find("[")
            pname_expr = pname[j + 1 : -1]
            pname = pname[0 : j]

            # If the parameter is "_" (the "don't care" parameter) and it wasn't
            # passed in kwargs, make one up.
            # Note: the only type of variable-length "don't care" parameters
            #       found in any characteristics is "character", so we don't
            #       need to make bogus "_" parameters of any other type.
            if pname == "_" and pname not in kwargs:
              kwargs[pname] = ""

            # Make sure the parameter was passed in kwargs
            if pname not in kwargs:
              raise RuntimeError("Argument \"{}\" required for WRITE "
					"command in characteristic {}".
					format(pname, chr_or_srv.name))

            # Make sure the parameter is a list, tuple or str
            ___kwargs_pname = kwargs[pname]
            if not isinstance(___kwargs_pname, list) and \
		not isinstance(___kwargs_pname, tuple) and \
		not isinstance(___kwargs_pname, str):
              raise RuntimeError("Argument \"{}\" must be a list, tuple or " \
					"str for WRITE command in " \
					"characteristic {}".
					format(pname, chr_or_srv.name))
            kw: Union[List[Tuple[float, float, int]], List[int],
			Tuple[int, ...], str] = ___kwargs_pname

            # Evaluate the expression to determine the number of elements
            # to encode
            nbelements: int = eval(pname_expr, kwargs)

            # Make sure the number of elements is positive
            if nbelements < 0:
              raise RuntimeError("Negative size {n} for argument "
					"\"{a}[{n}]\" for WRITE command in " \
					"characteristic {c}".
					format(n = nbelements, a = pname,
						c = chr_or_srv.name))

            # If the parameter is "_" (the "don't care" parameter) and it wasn't
            # passed in kwargs, make one up to be encoded as a filler
            # Note: the only type of variable-length "don't care" parameters
            #       found in any characteristics is "character", so we don't
            #       need to make bogus "_" parameters of any other type.
            if pname == "_" and not kw:
              kw = "\x00" * nbelements

            # Make sure the parameter passed in kwargs is the right length
            if len(kw) != nbelements:
              raise RuntimeError("Argument \"{}\" required for WRITE command " \
					"in characteristic {} must contain " \
					"{} elements, not {}".
					format(pname, chr_or_srv.name,
						nbelements, len(kw)))

            # Make sure all the elements in a list or tuple are the correct type
            if type(kw) in (list, tuple) and \
		not all([type(v) in acc_arg_types for v in kw]):
              raise RuntimeError("Argument \"{}\" required for WRITE command " \
					"in characteristic {} contains " \
					"elements other than {}".
					format(pname, chr_or_srv.name,
						" or ".join(["{}".format(t) \
						for t in acc_arg_types])))

            # Encode the elements and add them to the command bytes
            if nbelements:
              cmddata += encoding_fct(kw, nbelements)

          # There is no expression between [...] at the end of the parameter
          # name: the parameter to encode is a scalar
          else:

            # If the parameter is "_" (the "don't care" parameter) and it wasn't
            # passed in kwargs, make one up.
            # Note: the only types of scalar "don't care" parameters
            #       found in any characteristics are integers, so we don't
            #       need to make bogus "_" parameters of any other type.
            if pname == "_" and pname not in kwargs:
              kwargs[pname] = 0

            # Make sure the parameter was passed in kwargs
            if pname not in kwargs:
              raise RuntimeError("Argument \"{}\" required for WRITE "
					"command in characteristic {}".
					format(pname, chr_or_srv.name))

            # Make sure the parameter is the correct type
            if type(kwargs[pname]) not in acc_arg_types:
              raise RuntimeError("Argument \"{}\" required for WRITE command " \
					"in characteristic {} should be " \
					"{}, not {}".
					format(pname, chr_or_srv.name,
						" or ".join(["{}".format(t) \
						for t in acc_arg_types]),
						type(kwargs[pname])))

            # Encode the elements and add them to the command bytes
            cmddata += encoding_fct((kwargs[pname],), 1)

    # Does the command concern a service?
    elif cmd in (LLcmd.SAVE_SERVICE, LLcmd.RESTORE_SERVICE):

      # Make sure chr_or_srv is a service
      if not isinstance(chr_or_srv, _LLsrv):
        raise ValueError("Target type {} incorrect - should be {}"
				.format(type(chr_or_srv), _LLsrv))

      # Make sure the service supports the command
      if cmd not in _SRV_DESC[chr_or_srv]["supported_cmds"]:
        raise ValueError("{} command not supported for service {}".
				format(cmd.name, chr_or_srv.name))

      # Add the service to the command bytes
      cmddata += uint_pack(chr_or_srv.value)

      if debugmsg is not None:
        debugmsg += ", srv: {}".format(chr_or_srv.name)

    # Not a command (i.e. NOTIFICATION)
    else:
      raise ValueError("Invalid command {}".format(cmd.name))

    if debugmsg is not None:
      debugmsg += " [{}]".format(self.__highlighted_bytes(cmddata))

    return (cmddata, debugmsg)



  def _send_cmd(self: LineLidar,
		cmd: _LLcmd,
		chr_or_srv: Union[_LLsrv, _LLchr],
                *args: bool,
		**kwargs: _CHR_ARGS_TYPE) \
		-> int:
    """! Send a command to the device.

    @param cmd Command to send
    @param chr_or_srv Characteristic or service
    @param args Positional arguments (here, only True or False allowed -
		see below)
    @param kwargs Keyworded arguments (see below)

    If the command is SAVE_SERVICE or RESTORE_SERVICE, pass a service (_LLsrv)
    in chr_or_srv. Otherwise pass a characteristic (_LLchr)

    If the command is SET_NOTIFICATION, pass a single argument True or False to
    enable or disable notification - e.g.
    _send_cmd(LLcmd.SET_NOTIFICATION, LLchr.TEMPERATURE, True)

    If the command is WRITE, pass the relevant parameters as keyworded
    arguments - e.g.
    _send_cmd(LLcmd.WRITE, LLchr.MIN_DISTANCE, distance = 3.5)

    @return Sent message ID
    """

    # Encode the command
    msgid = self.__msgid
    debugmsg: Optional[str]
    cmddata: Union[bytes, List[bytes]]
    cmddata, debugmsg = self.__encode_cmd(msgid, cmd, chr_or_srv,
						*args, **kwargs)

    # If we're writing to the MEASURING_RATE characteristic, track whether
    # the device is currently ranging
    if cmd == LLcmd.WRITE and chr_or_srv == LLchr.MEASURING_RATE:
      self.__is_ranging = kwargs["frequency"] != 0

    # Do we send the command to the LineLidar through a SSH relay?
    if self.__sshproc is not None:

      # Encapsulate the command in a list
      cmddata = [cmddata]

      # If the device is ranging and we should automatically stop it upon
      # closing, encode several extra commands to write frequency 0 Hz to the
      # MEASURING_RATE characteristic for the SSH relay stub to send to the
      # device all by itself before quitting, in case we abruptly lose the
      # SSH connection before we can send the commands ourselves, and add the
      # extra commands to the list
      if self.__autostop_ranging and self.__is_ranging:
        future_msgid: int
        for future_msgid in range(msgid + 1, msgid + 4):
          cmddata = [self.__encode_cmd(future_msgid % 0xffffffff,
					LLcmd.WRITE, LLchr.MEASURING_RATE,
					frequency = 0)[0]] + cmddata

    # Send the command or commands to the LineLidar
    self._send_data(cmddata)

    # Trim the sent commands log - i.e. remove the oldest entries - 1 to make
    # room for this command
    while len(self.__sent_cmds_log[0]) >= self._sent_cmds_log_depth:
      self.__sent_cmds_log[0].pop(0)
      self.__sent_cmds_log[1].pop(0)

    # Add this command to the sent commands log
    self.__sent_cmds_log[0].append((self.__msgid, cmd))
    self.__sent_cmds_log[1].append(chr_or_srv)

    # Increment the message identification number
    self.__msgid = (msgid + 1) % 0xffffffff

    if debugmsg is not None:
      print(debugmsg)
      sys.stdout.flush()

    return msgid



  def __windows_reader_thread(self: LineLidar,
				fd: IO,
				queue: multiprocessing.queues.Queue) \
				-> None:
    """! Windows-only blocking file reader thread

    This thread reads data from a blocking file and transfer the data to a
    queue, which in turn can be read with a timeout.

    This workaround is required because Windows doesn't provide select() on
    file descriptors.
    """

    e: Exception

    t: threading.Thread = threading.current_thread()

    while True:

      # Get data from the blocking file
      try:
        b: bytes = fd.read(4096)

      except Exception as e:
        queue.put((None, e))
        return

      # If we got no data, either the pipe is broken unintentionally, or we were
      # instructed to stop
      if not b:

        if getattr(t, "do_run", True):
          queue.put((None, BrokenPipeError))
          return

        else:
          break

      # Relay the data
      queue.put((b, None))

    # Send a sentinel to indicate we've stopped
    queue.put((None, RuntimeWarning))




  def __recv_ssh_line(self: LineLidar,
			timeout: Optional[float] = None) \
			-> str:
    """! Get a line of text from the SSH client

    @param timeout Communication timeout in seconds

    @return Received text line
    """

    # If we already have a line to return in the receive buffer, return it
    # rightaway
    try:
      i: int = self.__ssh_recvbuf.index(b"\n")
      l: str = self.__ssh_recvbuf[:i].decode("ascii").strip("\r")
      self.__ssh_recvbuf = self.__ssh_recvbuf[i + 1:]
      return l

    except:
      pass

    if timeout is None:
      timeout = self.__default_timeout

    # Make sure the SSH process has been spawned
    if self.__sshproc is None:
      raise RuntimeError("Communication with the SSH server is not open")

    if timeout != self.__timeout:
      self.__timeout = timeout

    timeout_tstamp: float = time() + self.__timeout

    while True:

      # If we waited too long to get a line from the SSH client, time out
      now: float = time()
      if now >= timeout_tstamp:
        raise TimeoutError

      # Wait for data from the SSH client
      data: bytes = b""
      exc: Optional[Exception] = None
      if self.__win_reader_thread is not None and \
		self.__sshproc_win_stdout is not None:
        try:
          data, exc = self.__sshproc_win_stdout.get(timeout = \
							timeout_tstamp - now)

        except queue.Empty:
          pass

        # The Windows reader thread sent us an exception, meaning it has
        # stopped: join() it and re-raise the exception
        if exc is not None:
          self.__win_reader_thread.join()
          self.__win_reader_thread = None
          raise exc

      else:
        if select.select([self.__sshproc.stdout], [], [],
				timeout_tstamp - now)[0]:
          data = self.__sshproc.stdout.read(4096)

      # Did we get something?
      if data:

        # Append the data to the receive buffer
        self.__ssh_recvbuf += data

        # If we have a line to return in the receive buffer, return it
        try:
          i = self.__ssh_recvbuf.index(b"\n")
          l = self.__ssh_recvbuf[:i].decode("ascii").strip("\r")
          self.__ssh_recvbuf = self.__ssh_recvbuf[i + 1:]
          return l

        except:
          pass

      # Check if the client has already posted a return code
      retcode: Optional[int] = self.__sshproc.poll()

      # If the client has died, close the connection and raise an exception
      if retcode is not None:
        self.close()
        raise RuntimeError("SSH client terminated with return code {}"
				.format(retcode))



  def __decode_msg(self: LineLidar,
			msg: bytes) \
			-> LLresponse:

    """! Decode a message from the %LineLidar.

    @param msg Packet to decode

    @return Received response
    """

    # The smallest response is 12 bytes long
    l: int = len(msg)
    if l < 12:
      raise ValueError("Response too short ({} bytes)".format(l))

    # Decode the response and detect a malformed response as we go:
    resp: LLresponse = LLresponse()
    i = 0

    uint_unpack: Callable = lambda b: unpack("<I", b)[0]

    # Decode the command that is being replied to (or NOTIFICATION), the message
    # ID and the status. In the case of a NOTIFICATION, there is no status in
    # the actual message (since it's always OK by definition) so assume OK.
    cmd: Optional[_LLcmd]
    cmd = self._val_to__LLcmd.get(uint_unpack(msg[i : i + 4]), None)
    if cmd is None:
      raise ValueError("Invalid command in response: [{}]".
			format(self.__highlighted_bytes(msg, i, i + 4)))
    resp.cmd = cmd
    i += 4

    resp.msgid = uint_unpack(msg[i : i + 4])
    i += 4

    # If the response isn't a NOTIFICATION, decode the status
    if resp.cmd != LLcmd.NOTIFICATION:

      status: Optional[_LLsta]
      status = self._val_to__LLsta.get(uint_unpack(msg[i : i + 4]), None)
      if status is None:
        raise ValueError("Invalid status in response: [{}]".
				format(self.__highlighted_bytes(msg, i, i + 4)))
      resp.status = status
      i += 4

      # Make sure the status is valid for the command, as getting an unexpected
      # status would be indicative of a malformed response
      if resp.cmd not in _LLSTA_CMDS[resp.status]:
        raise ValueError("Status {} invalid for command {} in response".
				format(resp.status, resp.cmd))

    # If the message is a reply to a READ command or a NOTIFICATION, decode
    # the service and the characteristic the READ or NOTIFICATION is about, then
    # the returned parameters
    if resp.cmd in (LLcmd.READ, LLcmd.NOTIFICATION):

      srv = self._val_to__LLsrv.get(uint_unpack(msg[i : i + 4]), None)
      if srv is None:
        raise ValueError("Invalid service in response: [{}]".
				format(self.__highlighted_bytes(msg, i, i + 4)))
      i += 4

      char: Optional[_LLchr]
      char = self._val_to__LLchr.get((srv, uint_unpack(msg[i : i + 4])), None)
      if char is None:
        raise ValueError("Invalid characteristic in response: [{}]".
				format(self.__highlighted_bytes(msg, i, i + 4)))
      resp.char = char
      i += 4

      # Decode the returned parameters if the response is a notification, or
      # if the status is OK
      if resp.cmd == LLcmd.NOTIFICATION or resp.status == LLsta.OK:

        # Decode the parameters listed in the characteristic's description
        f: Dict[str, _ENC_DEC_FCTS_VAL_TYPE]
        for f in _CHR_DESC[resp.char]["fields"] + ({"": None},):

          pname: str = list(f)[0]
          pname_expr: Optional[str] = None

          # Have we reached the terminator?
          if not pname:
            break

          # Get accepted decoded argument types, the number of bytes we expect
          # to decode for one element and the decoding function for this
          # parameter
          expected_bytes: int = f[pname][2]
          decoding_fct: Callable = f[pname][3]

          # Is there an expression between [...] at the end of the parameter
          # name to specify the number of parameter elements to decode?
          if pname[-1] == "]":

            # Split the parameter name and expression
            j: int = pname.find("[")
            pname_expr = pname[j + 1 : -1]
            pname = pname[0 : j]

            # Evaluate the expression to determine the number of elements
            # to decode
            nbelements: int
            nbelements = eval(pname_expr, resp.__dict__.copy())

            # Make sure the number of elements is positive
            if nbelements < 0:
              raise RuntimeError("Negative size {n} for argument "
					"\"{a}[{n}]\" to decode in response" \
					"[{r}]".
					format(n = nbelements, a = pname,
						r = self.__highlighted_bytes(
							msg, i, i)))

            # Make sure we have enough bytes left to decode all the elements
            expected_bytes *= nbelements
            if l - i < expected_bytes:
              raise ValueError("Not enough bytes left to decode {}[{}] "
				"in response [{}]".format(pname, nbelements,
					self.__highlighted_bytes(msg, i, l)))

            # Decode all the parameter's elements and add them to the response
            # if the parameter name isn't "_" (the "don't care" parameter)
            if pname != "_":
              resp.__dict__[pname] = decoding_fct(msg[i : i + expected_bytes],
							nbelements)

          # There is no expression between [...] at the end of the parameter
          # name: the parameter to decode is a scalar
          else:

            # Do we have enough bytes to decode the parameter?
            if l - i < expected_bytes:
              raise ValueError("Not enough bytes left to decode {} in "
				"response [{}]".format(pname,
					self.__highlighted_bytes(msg, i, l)))

            # Decode all the parameter's elements and add them to the response
            # if the parameter name isn't "_" (the "don't care" parameter)
            if pname != "_":
              resp.__dict__[pname] = decoding_fct(msg[i : i + expected_bytes],
							1)[0]

          i += expected_bytes

    # Make sure all the bytes in the message have been consumed
    if l > i:
      raise ValueError("{} extraneous undecoded bytes in response [{}]".
			format(l - i, self.__highlighted_bytes(msg, i, l)))

    if self._debug:
      print("<LL: {}".format(resp))
      sys.stdout.flush()

    return resp



  def _recv_msg(self: LineLidar,
		timeout: Optional[float] = None) \
		-> LLresponse:
    """! Get a message from the %LineLidar.

    @param timeout Communication timeout in seconds

    @return Received response
    """

    e: Exception

    if timeout is None:
      timeout = self.__default_timeout

    # Make sure either a communication socket exists or a SSH client has
    # been spawned
    if self.__socket is None and self.__sshproc is None:
      raise RuntimeError("Communication with the LineLidar is not open")

    # Receive a UDP datagram from the intended host
    if timeout != self.__timeout:
      self.__timeout = timeout

    timeout_tstamp: float = time() + self.__timeout

    while True:

      # If we waited too long to get a packet from the intended host, time out
      now: float = time()
      if now >= timeout_tstamp:
        raise TimeoutError

      pkt: Tuple[bytes, Tuple[str, int]]

      # Are we talking to the device directly?
      msg: bytes
      if self.__socket is not None:

        self.__socket.settimeout(timeout_tstamp - now)

        try:
          pkt = self.__socket.recvfrom(4096)

        except socket.timeout:
          continue

      # Are we talking to the device through a SSH relay host?
      elif self.__sshproc is not None:

        try:
          # Receive a line from the SSH relay stub. Strip any surrounding
          # "b'...'" bytes representation markers - i.e. if it came from a
          # Python3 SSH relay stub.
          s: str = self.__recv_ssh_line(timeout = timeout_tstamp - now)
          if s[0] == "b" and s[1] in ["'", '"'] and s[-1] == s[1]:
            s = s[2 : -1]

          # Recover and verify the format of the packet sent by the SSH relay
          # stub: in case of any error, ignore the line
          try:
            pkt_temp: Tuple[Union[bytes, str], Tuple[str, int]]
            pkt_temp = pickle.loads(b64decode(s), encoding = "latin1")
            assert(type(pkt_temp) in (list, tuple))
            assert(type(pkt_temp[0]) in (bytes, str))
            assert(type(pkt_temp[1]) in (list, tuple))
            assert(type(pkt_temp[1][0]) == str)
            assert(type(pkt_temp[1][1]) == int)
            assert(re.match("^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$", pkt_temp[1][0]))
          except:
            continue

          # If the UDP packet is a string -i.e. it came from a Python2 SSH
          # relay stub - turn it into bytes
          if isinstance(pkt_temp[0], str):
            pkt = (pkt_temp[0].encode("latin1"), pkt_temp[1])
          else:
            pkt = (pkt_temp[0], pkt_temp[1])

        except TimeoutError:
          continue

      # Was the packet sent by the intended host or hosts?
      accept_pkt: bool
      if self._netmask == 0xffffffff:
        accept_pkt =  pkt[1] == self.__addrport
      else:
        a1: str; a2: str; a3: str; a4: str
        a5: str; a6: str; a7: str; a8: str
        a1, a2, a3, a4 = pkt[1][0].split(".")
        a5, a6, a7, a8 = self.__addrport[0].split(".")
        accept_pkt = (((int(a1) << 24) | (int(a2) << 16) | \
			(int(a3) << 8) | int(a4)) & self._netmask == \
			((int(a5) << 24) | (int(a6) << 16) | \
			(int(a7) << 8) | int(a8)) & self._netmask) and \
			pkt[1][1] == self.__addrport[1]

      # If we accept the packet, extract the message and stop receiving
      # packets
      if accept_pkt:
        msg = pkt[0]
        break

    recv_local_timestamp: datetime = datetime.now()

    # Decode the packet and timestamp it
    resp: LLresponse = self.__decode_msg(msg)
    resp.recv_local_timestamp = recv_local_timestamp

    return resp



  def _get_cmd_response(self: LineLidar,
			exc_on_cmd: bool = True,
			exc_on_msgid: bool = True,
			exc_on_chr: bool = True,
			exc_on_nok: bool = True,
			ignore_sent_cmds_log: bool = False,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Get a response to the latest command (i.e. not a notification).

    @param exc_on_cmd Raise an exception if the response's command doesn't
                      match that of the last sent command
    @param exc_on_msgid Raise an exception if the response's message ID doesn't
                        match that of the last sent command
    @param exc_on_chr Raise an exception if the response's characteristic
                      doesn't match that of the last sent command in responses
                      to READ commands
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param ignore_sent_cmds_log If asserted, incoming messages will not be
                                checked against older entries in the sent
                                commands log to catch out-of-order responses,
                                and the last sent command will not be removed
                                from the log

    @param timeout Communication timeout in seconds

    if ignore_sent_cmds_log isn't asserted, if a response to a command is
    received and its message ID, command and service or characteristic match
    one of the older entries in the sent commands log, the response is
    considered an out-of-order message, the corresponding entry is removed
    from the sent commands log and the response is silently discarded.

    After discarding out-of-order messages, if either the message's command,
    message ID or characteristic (in the case of response to a READ command)
    don't match the last sent command's and either exc_on_cmd, exc_on_msgid or
    exc_on_chr are asserted (default), an exception is raised.

    if ignore_sent_cmds_log isn't asserted and the response matches the last
    sent command, it is removed from the log.

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    If a NOTIFICATION is received, it is pushed into the notifications stack
    and the function keeps on waiting for a response to a command.

    @return Received response
    """

    if timeout is None:
      timeout = self.__default_timeout

    # Get the last sent command's command, message ID and characteristic if any
    cmd: _LLcmd
    msgid: int
    chr_or_srv: Union[_LLsrv, _LLchr]
    msgid, cmd = self.__sent_cmds_log[0][-1]
    chr_or_srv = self.__sent_cmds_log[1][-1]

    # Prepare the beginning of the exception error message if we're to raise
    # one in case of error
    resp_errors: List[str] = ["In response to cmd"]

    if cmd is not None:
      resp_errors[0] += " {}".format(cmd.name)

    if msgid is not None or chr_or_srv is not None:
      resp_errors[0] += " with"

    if msgid is not None:
      resp_errors[0] += " msgid {}".format(msgid)

      if chr_or_srv is not None:
        resp_errors[0] += ","

    if chr_or_srv is not None:

      # Does the command concern a characteristic?
      if cmd in (LLcmd.READ, LLcmd.WRITE, LLcmd.SET_NOTIFICATION):
        assert isinstance(chr_or_srv, _LLchr)
        resp_errors[0] += " srv {}, chr {}".format(chr_or_srv.value[0].name,
							chr_or_srv.name)

      # Does the command concern a service?
      elif cmd in (LLcmd.SAVE_SERVICE, LLcmd.RESTORE_SERVICE):
        assert isinstance(chr_or_srv, _LLsrv)
        resp_errors[0] += " srv {}".format(chr_or_srv.name)

    resp_errors[0] += ": "

    # Get messages
    timeout_tstamp: float = time() + timeout
    while True:

      # If we waited too long to get the message we were waiting for, time out
      now: float = time()
      if now >= timeout_tstamp:
        raise TimeoutError

      resp: LLresponse = self._recv_msg(timeout = timeout_tstamp - now)

      # If we got a notification, push it into the notifications stack and
      # continue getting more messages
      if resp.cmd == LLcmd.NOTIFICATION:
        self.notifications.append(resp)
        continue

      # If we don't ignore the sent commands log, if we have an older entry
      # in it that corresponds to this response, remove the entry and discard
      # the response
      if not ignore_sent_cmds_log:
        ooo: bool = False
        while True:
          try:
            i = self.__sent_cmds_log[0][:-1].index((resp.msgid, resp.cmd))
            if getattr(resp, "char", self.__sent_cmds_log[1][i]) == \
					self.__sent_cmds_log[1][i]:
              self.__sent_cmds_log[0].pop(i)
              self.__sent_cmds_log[1].pop(i)
              ooo = True
          except ValueError:
            break
        if ooo:
          if self._debug:
            print("<LL: Out-of-order command response ignored: {}".format(resp))
            sys.stdout.flush()
          continue

      # Is the response a response to the last sent command and should we
      # raise an exception if it isn't?
      if exc_on_cmd and resp.cmd != cmd:
        resp_errors.append("response to command {} received but {} expected".
				format(resp.cmd.name, cmd.name))

      # Does the response's message ID match the last sent command's and should
      # we raise an exception if it doesn't?
      if exc_on_msgid and resp.msgid != msgid:
        resp_errors.append("response msgid {} received but {} expected".
				format(resp.msgid, msgid))

      # If the response has a characteristic, does the response's characteristic
      # match the last sent command's and should we raise an exception if it
      # doesn't?
      if exc_on_chr and getattr(resp, "char", chr_or_srv) != chr_or_srv:\
        resp_errors.append("response chr {} received but {} expected".
				format(resp.char.name, chr_or_srv.name))

      # If we don't ignore the sent commands log and we arrive here without any
      # errors in the response, it matches the last sent command, so we can
      # remove it from the log
      if not ignore_sent_cmds_log and len(resp_errors) == 1:
        self.__sent_cmds_log[0].pop(-1)
        self.__sent_cmds_log[1].pop(-1)

      # Is the status OK and should we raise an exception if it isn't?
      if exc_on_nok and resp.status != LLsta.OK:
        resp_errors.append("response status {}".format(resp.status.name))

      # If we have any errors with the response, raise an exception
      if len(resp_errors) > 1:
        raise ValueError(resp_errors[0] + " + ".join(resp_errors[1:]))

      return resp



  def __retry_cmd(self: LineLidar,
			cmd: _LLcmd,
			chr_or_srv: Union[_LLsrv, _LLchr],
			retries: int = None,
			exc_on_nok: bool = True,
			timeout: Optional[float] = None,
			*args: bool,
			**kwargs: _CHR_ARGS_TYPE) \
			-> LLresponse:
    """! Try to send a command to the device and get a response from it.
    If the response times out, retry sending the command and getting a new
    response until the number of allowed retries runs out.

    @param cmd Command to send
    @param chr_or_srv Characteristic or service
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK
    @param timeout Communication timeout in seconds
    @param args Positional arguments for _send_cmd
    @param kwargs Keyworded arguments for _send_cmd

    @return Response to the command
    """

    r: LLresponse
    e: Exception

    # Make sure either a communication socket exists or a SSH client has
    # been spawned
    if self.__socket is None and self.__sshproc is None:
      raise RuntimeError("Communication with the LineLidar is not open")

    assert retries is None or retries >= 0
    assert self.retries is not None

    for tries in range((self.retries if retries is None else retries), -1, -1):
      msgid: int = self._send_cmd(cmd, chr_or_srv, *args, **kwargs)
      try:
        r = self._get_cmd_response(exc_on_nok = exc_on_nok, timeout = timeout)
        break
      except Exception as e:
        if tries == 0:
          raise e
        pass

    return r



  def read_chr(self: LineLidar,
		char: _LLchr,
		retries: int = None,
		exc_on_nok: bool = True,
		timeout: Optional[float] = None) \
		-> LLresponse:
    """! Read a characteristic.

    @param char Characteristic to read
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return READ command response.
    """

    return self.__retry_cmd(LLcmd.READ, char, retries, exc_on_nok, timeout)



  def write_chr(self: LineLidar,
		char: _LLchr,
		retries: int = None,
		exc_on_nok: bool = True,
		timeout: Optional[float] = None,
		**kwargs: _CHR_ARGS_TYPE) \
		-> LLresponse:
    """! Write a characteristic

    @param char Characteristic to write
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds
    @param kwargs Keyworded arguments (see below)

    Pass the relevant parameters as keyworded arguments - e.g.
    write_chr(LLchr.MIN_DISTANCE, distance = 3.5)

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return WRITE command esponse
    """

    return self.__retry_cmd(LLcmd.WRITE, char, retries, exc_on_nok, timeout,
				**kwargs)



  def set_notification(self: LineLidar,
			char: _LLchr,
			enabled: bool,
			retries: int = None,
			exc_on_nok: bool = True,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Enable or disable notification on a characteristic

    @param char Characteristic to enable or disable notification on
    @param enabled Whether notification is enabled
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return SET_NOTIFICATION command response
    """

    return self.__retry_cmd(LLcmd.SET_NOTIFICATION, char, retries, exc_on_nok,
				timeout, enabled)



  def enable_notification(self: LineLidar,
				char: _LLchr,
				retries: int = None,
				exc_on_nok: bool = True,
				timeout: Optional[float] = None) \
				-> LLresponse:
    """! Enable notification on a characteristic

    @param char Characteristic to enable notification on
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return SET_NOTIFICATION command response
    """

    return self.set_notification(char, True, retries = retries,
					exc_on_nok = exc_on_nok,
					timeout = timeout)



  def disable_notification(self: LineLidar,
				char: _LLchr,
				retries: int = None,
				exc_on_nok: bool = True,
				timeout: Optional[float] = None) \
				-> LLresponse:

    """! Disable notification on a characteristic

    @param char Characteristic to disable notification on
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return SET_NOTIFICATION command response
    """

    return self.set_notification(char, False, retries = retries,
					exc_on_nok = exc_on_nok,
					timeout = timeout)



  def disable_all_notifications(self: LineLidar,
				incl_restricted: bool = False,
				retries: int = None,
				exc_on_nok: bool = True,
				timeout: Optional[float] = None) \
				-> LLresponse:

    """! Disable notification on all characteristics that support it

    @param incl_restricted Also disable notification on characteristics
                           where notifications are restricted
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds
    """

    r: LLresponse

    for char in _CHR_DESC:
      if LLcmd.SET_NOTIFICATION in _CHR_DESC[char]["supported_cmds"] and \
		(incl_restricted or LLcmd.SET_NOTIFICATION not in \
					_CHR_DESC[char]["restricted_cmds"]):
        r = self.disable_notification(char, retries = retries,
					exc_on_nok = False,
					timeout = timeout)
        if r.status != LLsta.OK:
          if exc_on_nok:
            raise ValueError("In response to cmd {}, srv {}, chr {}: "
				"response status {}".
				format(r.cmd.name, char.value[0].name,
					char.name, r.status.name))
            break

    return r



  def report_zero_results(self: LineLidar,
				on: bool,
				retries: int = None,
				exc_on_nok: bool = True,
				timeout: Optional[float] = None) \
				-> LLresponse:
    """! Enable or disable zero results reporting

    @param on Whether to report zero results
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return WRITE command esponse
    """

    return self.__retry_cmd(LLcmd.WRITE, LLchr.REPORT_ZERO_RESULTS, retries,
				exc_on_nok, timeout, on = on)



  def set_sampling_rate(self: LineLidar,
			frequency: int,
			retries: int = None,
			exc_on_nok: bool = True,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Set the sampling rate

    @param frequency Sampling frequency
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return WRITE command esponse
    """

    return self.__retry_cmd(LLcmd.WRITE, LLchr.MEASURING_RATE, retries,
				exc_on_nok, timeout, frequency = frequency)



  def stop_sampling(self: LineLidar,
			retries: int = None,
			exc_on_nok: bool = True,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Stop sampling

    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return WRITE command esponse
    """

    return self.__retry_cmd(LLcmd.WRITE, LLchr.MEASURING_RATE, retries,
				exc_on_nok, timeout, frequency = 0)



  def save_srv(self: LineLidar,
		srv: _LLsrv,
		retries: int = None,
		exc_on_nok: bool = True,
		timeout: Optional[float] = None) \
		-> LLresponse:
    """! Save a service to non-volatile memory

    @param srv Service to save to non-volatile memory
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return SAVE_SERVICE command response
    """

    return self.__retry_cmd(LLcmd.SAVE_SERVICE, srv, retries, exc_on_nok,
				timeout)



  def restore_srv(self: LineLidar,
			srv: _LLsrv,
			retries: int = None,
			exc_on_nok: bool = True,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Restore a service from non-volatile memory

    @param srv Service to restore from non-volatile memory
    @param retries How many times a failed command should be retried
    @param exc_on_nok Raise an exception on response not OK (see below)
    @param timeout Communication timeout in seconds

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.

    @return RESTORE_SERVICE command response
    """

    return self.__retry_cmd(LLcmd.RESTORE_SERVICE, srv, retries, exc_on_nok,
				timeout)



  def flush_notifications(self: LineLidar) \
				-> None:
    """! Clear the notifications stack
    """

    self.notifications = []



  def get_notification(self: LineLidar,
			chrmask: Union[List[_LLchr],
					Tuple[_LLchr], None] = None,
			timeout: Optional[float] = None) \
			-> LLresponse:
    """! Get a notification, either from the notifications stack, or receive it
    from the device if the stack is empty.

    @param chrmask List of expected notification characteristics (see below)
    @param timeout Communication timeout in seconds

    If a message other than a notification is received from the device (i.e. a
    response to a command), an exception is raised.

    If a list of characteristics is specified in chrmask and the notification's
    characteristic isn't in that list, an exception is raised.

    @return Received notification.
    """

    notif: LLresponse

    # If there are notifications in the stack, pop the oldest one...
    if self.notifications:
      notif = self.notifications.pop(0)

    # ...otherwise get a notification from the device
    else:

      notif = self._recv_msg(timeout = timeout)

      # Did we get a notification?
      if notif.cmd != LLcmd.NOTIFICATION:
        raise ValueError("Expected notification, got response to command {}".
				format(notif.cmd.name))

    # Is the notification's characteristic listed in the characteristics mask?
    if chrmask is not None and notif.char not in chrmask:
      raise ValueError("Notification chr {} received but {} expected".
			format(notif.char.name,
				", ".join([c.name for c in chrmask])[::-1].
				replace(",", "ro ", 1)[::-1]))

    return notif



  def wait_device_quiet(self: LineLidar,
			timeout: Optional[float] = None) \
			-> None:
    """! Wait until the device becomes silent.

    @param timeout Communication timeout in seconds
    """

    if timeout is None:
      timeout = self.__default_timeout / 4

    while True:
      try:
        notif = self._recv_msg(timeout = timeout)

      except TimeoutError:
        break



  def set_clean_state(self: LineLidar,
			incl_restricted: bool = False,
			retries: int = None,
			timeout: Optional[float] = None) \
			-> None:
    """! Stop any active ranging, disable all notifications and flush the
    notifications stack, so that the device is left in a known, stopped state.

    @param incl_restricted Also disable notification on characteristics
                           where notifications are restricted
    @param retries How many times a failed command should be retried
    @param timeout Communication timeout in seconds
    """

    # Stop any active ranging
    self.write_chr(LLchr.MEASURING_RATE, retries = retries, frequency = 0,
			timeout = timeout)

    # Disable notification on all characteristics
    self.disable_all_notifications(incl_restricted = incl_restricted,
					retries = retries, timeout = timeout)

    # clear the notifications stack
    self.flush_notifications()



  def reset(self: LineLidar,
		reconnect: bool = True,
		reconnect_timeout: Optional[float] = None,
		retries: int = None,
		exc_on_nok: bool = True,
		timeout: Optional[float] = None) \
		-> None:
    """! Soft-reset the device

    @param reconnect Attempt to reconnect the device after reset
    @param reconnect_timeout Timeout in seconds when attempting to reconnect
    @param retries How many times a failed command should be retried.
                   Only applies if reconnect is False.
    @param exc_on_nok Raise an exception on response not OK (see below).
                   Only applies if reconnect is False.

    @param timeout Communication timeout in seconds.
                   Only applies if reconnect is False.

    If exc_on_nok is asserted (default), an exception is raised when the
    response's status is not OK.
    """

    r: LLresponse

    # Send the reset command, expect no reply
    try:
      r = self.__retry_cmd(LLcmd.WRITE, LLchr.RESET_DEVICE, 0, False, timeout,
				reset = True)
      raise RuntimeError("Unexpected response to reset command: {}".format(r))
    except TimeoutError:
      pass

    reset_tstamp: float = time()

    # Attempt to reconnect if required
    if reconnect:
      reboot_timeout: float = _ll_default_reboot_timeout \
					if reconnect_timeout is None else \
				reconnect_timeout
      while True:
        try:
          r = self.__retry_cmd(LLcmd.READ, LLchr.RESET_DEVICE, retries, False,
				timeout)
          if r.cmd != LLcmd.READ or \
		r.char != LLchr.RESET_DEVICE or \
		r.status != LLsta.OK or \
		r.reset != False:
            raise RuntimeError("Unexpected response when reconnecting: {}".
				format(r))
          break

        except TimeoutError:
          if time() - reset_tstamp >= reboot_timeout:
            raise
          pass



## Routines
# @defgroup Routines Routines

def discover(network: Optional[str] = None,
		port: Optional[int] = _ll_default_port,
		sshcmd: Optional[str] = None,
		sshpypath: Optional[str] = None,
		retries: int = None,
		debug: bool = False,
		timeout: Optional[float] = None) \
		-> List[str]:
  """! Discover %LineLidar devices on a network

  @param network Network / netmask to discover devices on, in xx.xx.xx.xx/mm
         format, or None to discover on all interfaces
  @param port Port of devices to discover
  @param sshcmd If specified, ssh command to log into a shell account to use
                the host as a relay to talk to the LineLidar. Works with a
                Linux or Windows host with sshd and Python 2 or 3 installed.
  @param sshpypath Path of the Python executable on the SSH relay host
  @param retries How many more times the discovery packet should be sent for
                 redundancy
  @param debug Enable or disable debugging messages
  @param timeout Communication timeout in seconds

  @return Broadcast a read request for the NETWORK characteristic, wait for
          replies and return the list of addresses of the devices that
          replied.
  """

  is_win = sys.platform[0:3] == "win"

  bcast_addrs_netmasks: List[Tuple[str, int]]

  addr_int: int
  netmask_int: int
  hostmask_int: int
  bcast_addr_int: int

  # Should we discover on all the interfaces?
  if network is None or network == "":

    # Get the list of network interfaces that are up
    ifaces = psutil.net_if_addrs()

    # Get the list of IPv4 broadcast addresses and netmasks from the interfaces
    # Because the broadcast address is always None in Windows, calculate the
    # broadcast address ourselves on that platform
    bcast_addrs_netmasks = []
    for iface in ifaces:
      for s in ifaces[iface]:
        if s.family == socket.AddressFamily.AF_INET and \
		not s.address.startswith("127."):
          netmask_int = sum([int(v) << (24 - i * 8) for i, v in \
				enumerate(s.netmask.split("."))])
          if is_win:
            hostmask_int = 0xffffffff ^ netmask_int
            bcast_addr_int = sum([int(v) << (24 - i * 8) for i, v in \
					enumerate(s.address.split("."))]) | \
				hostmask_int
            bcast_addrs_netmasks.append(("{}.{}.{}.{}".format(
						bcast_addr_int >> 24,
						(bcast_addr_int >> 16) & 0xff,
						(bcast_addr_int >> 8) & 0xff,
						bcast_addr_int & 0xff),
					netmask_int))
          else:
            if s.broadcast is not None:
              bcast_addrs_netmasks.append((s.broadcast, netmask_int))

    if not bcast_addrs_netmasks:
      raise ValueError("No interface with an IPv4 broadcast address found")

  # Should we discover on a specific interface with a specific network address?
  else:

    # Is the network format correct?
    m: Optional[List] = re.findall("^([0-9]{1,3})\.([0-9]{1,3})\." \
					"([0-9]{1,3})\.([0-9]{1,3})/" \
					"([0-9]{1,2})$", network)
    if m and all([0 <= int(v) < 256 for v in m[0]]) and 0 < int(m[0][4]) < 32:

      # Work out the broadcast address and netmask
      hostmask_int = (1 << (32 - int(m[0][4]))) - 1
      netmask_int = 0xffffffff ^ hostmask_int
      bcast_addr_int = ((int(m[0][0]) << 24) | \
			(int(m[0][1]) << 16) | \
			(int(m[0][2]) << 8) | \
			int(m[0][3])) | hostmask_int
      bcast_addrs_netmasks = [("{}.{}.{}.{}".format(
					bcast_addr_int >> 24,
					(bcast_addr_int >> 16) & 0xff,
					(bcast_addr_int >> 8) & 0xff,
					bcast_addr_int & 0xff),
				netmask_int)]

    else:
      raise ValueError("invalid network address {}".format(network))

  ip_list = []

  # Iterate over the list of broadcast addresses to broadcast the discovery
  # packet to
  bcast_addr: str
  netmask: int
  for bcast_addr, netmask in bcast_addrs_netmasks:

    # Create a new LineLidar instance
    ll = LineLidar(debug = debug)

    # Override the netmask so recv_msg() accepts replies from any host on the
    # probed network
    ll._netmask = netmask

    try:
      # Open communication to the broadcast address
      bcast_socket = ll.open(bcast_addr, port, sshcmd, sshpypath,
				retries = retries,
				set_clean_state = False, timeout = timeout)

      # Broadcast a read command for the NETWORK characteristic
      assert ll.retries is not None
      for _ in range(ll.retries + 1):
        ll._send_cmd(LLcmd.READ, LLchr.NETWORK)

      # Get replies until we time out and collect the IPs from valid replies
      while True:
        try:
          r = ll._get_cmd_response(exc_on_nok = False,
					exc_on_msgid = False,
					ignore_sent_cmds_log = True,
					timeout = timeout)
        except TimeoutError:
          break

        except KeyboardInterrupt:
          raise

        except:
          pass

        # If we got a valid reply, add the responder's IP to the list
        if r.status == LLsta.OK:
          ip_list.append(r.addr.__repr__())

    finally:
      # Close the LineLidar instance
      ll.close()

  # Return a unique set of IP addresses
  return list(set(ip_list))
