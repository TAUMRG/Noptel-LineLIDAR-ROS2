"""! %LineLidar Low-level communication class

@file __init__.py

@mainpage Python3 %LineLidar class

@section main_page_description Description
Low-level class to communicate with a %LineLidar device in Python

@section main_page_notes Notes
Copyright 2023 (c) Noptel Oy, Oulu Finland

Tested on:
  - Linux
  - Windows

Authors:
  PCo
"""

from __future__ import annotations



### Submodules
__all__ = ["default", "linelidar"]



## Modules
#

from struct import pack, unpack, iter_unpack
from datetime import datetime, timedelta
from typing import List, Tuple, Dict, Type, Union, Optional, Callable



## Base classes
# @defgroup BaseClasses Base classes

class ipaddress:
  """! Simple IPv4 address class

  @ingroup BaseClasses
  """

  # ## Variables
  ip: bytes



  # ## Methods
  def __init__(self: ipaddress,
		addr: Union[str, bytes]) \
		-> None:
    """! Initialize the IP address bytes
    """

    # If the address was passed as a decimal representation, convert it
    # into bytes...
    if isinstance(addr, str):

      a1: str; a2: str; a3: str; a4: str
      a1, a2, a3, a4 = addr.split(".")

      ## Byte representation of an IPv4 address
      self.ip = bytes([int(a1), int(a2), int(a3), int(a4)])

    # ...otherwise store it as-is
    else:
      self.ip = addr



  def __eq__(self: ipaddress,
		other: object) \
		-> bool:
    """! ___eq__ method to compare two IP addresses
    """

    return type(other) == type(self) and \
		other.__dict__["ip"] == self.__dict__["ip"]



  def __repr__(self: ipaddress) \
		-> str:
    """! Return the decimal representation of the IP address
    """

    return "{}.{}.{}.{}".format(self.ip[0], self.ip[1], self.ip[2], self.ip[3])



class macaddress:
  """! Simple MAC address class

  @ingroup BaseClasses
  """

  # ## Variables
  mac: bytes



  # ## Methods
  def __init__(self: macaddress,
		addr: Union[str, bytes]) \
		-> None:
    """! Initialize the MAC address bytes
    """

    # If the address was passed as a decimal representation, convert it
    # into bytes...
    if isinstance(addr, str):

      a1: str; a2: str; a3: str; a4: str; a5: str; a6: str
      a1, a2, a3, a4, a5, a6 = addr.split(":")

      ## Byte representation of a MAC address
      self.mac = bytes([int(a1, 16), int(a2, 16), int(a3, 16),
			int(a4, 16), int(a5, 16), int(a6, 16)])

    # ...otherwise store it as-is
    else:
      self.mac = addr



  def __eq__(self: macaddress,
		other: object) \
		-> bool:
    """! ___eq__ method to compare two MAC addresses
    """

    return type(other) == type(self) and \
		other.__dict__["mac"] == self.__dict__["mac"]



  def __repr__(self: macaddress) \
		-> str:
    """! Return the decimal representation of the MAC address
    """

    return "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(
		self.mac[0], self.mac[1], self.mac[2],
		self.mac[3], self.mac[4], self.mac[5])



class _LLintEnum:
  """Pseudo-enum element to store an integer value

  @ingroup BaseClasses
  """

  # ## Variables
  name: str
  value: int



  # ## Methods
  def __init__(self: _LLintEnum,
		name: str,
		value: int) \
		-> None:
    """Initialize the enum element
    """

    self.__dict__["name"] = name
    self.__dict__["value"] = value



  def __eq__(self: _LLintEnum,
		other: object) \
		-> bool:
    """___eq__ method to compare two enum elements
    """

    return isinstance(other, _LLintEnum) and self.value == other.value



  def __setattr__(self: _LLintEnum,
			attr: str,
			val) \
			-> None:
    """Disabled setter, as enums are immutable
    """

    raise AttributeError("{} is immutable".format(self))



  def __hash__(self: _LLintEnum) \
		-> int:
    """__hash__ method
    """

    return self.value



class _LLcmd(_LLintEnum):
  """! LineLidar command enum element
  """
  pass



class _LLsrv(_LLintEnum):
  """! LineLidar service enum element
  """
  pass



class _LLsta(_LLintEnum):
  """! LineLidar status code enum element
  """
  pass



class _LLchr:
  """Pseudo-enum element to store a (_LLsrv, int) tuple describing a
  characteristic

  @ingroup BaseClasses
  """

  # ## Variables
  name: str
  value: Tuple[_LLsrv, int]
  _hash: int



  # ## Methods
  def __init__(self: _LLchr,
		name: str,
		value: Tuple[_LLsrv, int]) \
		-> None:
    """Initialize the enum element
    """

    self.__dict__["name"] = name
    self.__dict__["value"] = value
    self.__dict__["_hash"] = (value[0].value << 32 | value[1])



  def __eq__(self: _LLchr,
		other: object) \
		-> bool:
    """___eq__ method to compare two enum elements
    """

    return isinstance(other, _LLchr) and self._hash == other._hash



  def __setattr__(self: _LLchr,
			attr: str,
			val) \
			-> None:
    """Disabled setter, as enums are immutable
    """

    raise AttributeError("{} is immutable".format(self))



  def __hash__(self: _LLchr) \
		-> int:
    """__hash__ method
    """

    return self._hash



## Enums
# @defgroup Enums Enums
class LLcmd:
  """! Commands / command responses

  @ingroup Enums
  """

  ## Error
  ERROR:            _LLcmd
  ERROR             = _LLcmd("ERROR",            0x0000)

  ## Characteristic read command
  READ:             _LLcmd
  READ              = _LLcmd("READ",             0x0001)

  ## Characteristic write command
  WRITE:            _LLcmd
  WRITE             = _LLcmd("WRITE",            0x0002)

  ## Notification sent by the device - not a command
  NOTIFICATION:     _LLcmd
  NOTIFICATION      = _LLcmd("NOTIFICATION",     0x0005)

  ## Enable / disable notification on a characteristic
  SET_NOTIFICATION: _LLcmd
  SET_NOTIFICATION  = _LLcmd("SET_NOTIFICATION", 0x0006)

  ## Commit a service's characteristics to non-volatile memory
  SAVE_SERVICE:     _LLcmd
  SAVE_SERVICE      = _LLcmd("SAVE_SERVICE",     0x0007)

  ## Reload a service's characteristics from non-volatile memory
  RESTORE_SERVICE:  _LLcmd
  RESTORE_SERVICE   = _LLcmd("RESTORE_SERVICE",  0x0008)



class LLsrv:
  """! Services

  @ingroup Enums

  @section LLsrv_notes Notes
  Underscore in a name indicates a private service
  """

  ## Information about the device
  DEVICE_INFO:   _LLsrv
  DEVICE_INFO    = _LLsrv("DEVICE_INFO",   0x0001)

  ## Characteristics to configure and start / stop measurements
  DEVICE_CONFIG: _LLsrv
  DEVICE_CONFIG  = _LLsrv("DEVICE_CONFIG", 0x0002)

  ## Hardware configuration characteristics
  HW_CONFIG:     _LLsrv
  HW_CONFIG      = _LLsrv("HW_CONFIG",     0x0003)

  ## Control and report rangefinding results
  RESULTS:       _LLsrv
  RESULTS        = _LLsrv("RESULTS",       0x0004)

  ## Debugging facilities
  DEBUG:         _LLsrv
  DEBUG          = _LLsrv("DEBUG",         0x0005)



class LLchr:
  """! Characteristics

  @ingroup Enums

  @section Notes
  Underscore in a name indicates a private characteristic
  """

  # DEVICE_INFO service characteristics

  ## Serial number
  SERIAL_NUMBER:       _LLchr
  SERIAL_NUMBER        = _LLchr("SERIAL_NUMBER",
				(LLsrv.DEVICE_INFO, 0x0002))

  ## Firmware version
  FW_VERSION:          _LLchr
  FW_VERSION           = _LLchr("FW_VERSION",
				(LLsrv.DEVICE_INFO, 0x0003))

  ## NETWORK: network settings
  NETWORK:             _LLchr
  NETWORK              = _LLchr("NETWORK",
				(LLsrv.DEVICE_INFO, 0x0004))

  ## TIME: current time
  TIME:                _LLchr
  TIME                 = _LLchr("TIME",
				(LLsrv.DEVICE_INFO, 0x0005))

  ## Temperature of the device
  TEMPERATURE:         _LLchr
  TEMPERATURE          = _LLchr("TEMPERATURE",
				(LLsrv.DEVICE_INFO, 0x0007))

  ## MAC address
  MAC:                 _LLchr
  MAC                  = _LLchr("MAC",
				(LLsrv.DEVICE_INFO, 0x000b))

  ## Default network
  DEFAULT_NETWORK:     _LLchr
  DEFAULT_NETWORK      = _LLchr("DEFAULT_NETWORK",
				(LLsrv.DEVICE_INFO, 0x0013))

  # DEVICE_CONFIG service characteristics

  ## Minimum ranging distance
  MIN_DISTANCE:        _LLchr
  MIN_DISTANCE         = _LLchr("MIN_DISTANCE",
				(LLsrv.DEVICE_CONFIG, 0x0002))

  ## Maximum ranging distance
  MAX_DISTANCE:        _LLchr
  MAX_DISTANCE         = _LLchr("MAX_DISTANCE",
				(LLsrv.DEVICE_CONFIG, 0x0003))

  ## Amplitude threshold
  AMPLITUDE_THRESHOLD: _LLchr
  AMPLITUDE_THRESHOLD  = _LLchr("AMPLITUDE_THRESHOLD",
				(LLsrv.DEVICE_CONFIG, 0x0005))

  ## Minimum angle
  MIN_ANGLE:           _LLchr
  MIN_ANGLE            = _LLchr("MIN_ANGLE",
				(LLsrv.DEVICE_CONFIG, 0x000a))

  ## Maximum angle
  MAX_ANGLE:           _LLchr
  MAX_ANGLE            = _LLchr("MAX_ANGLE",
				(LLsrv.DEVICE_CONFIG, 0x000b))

  ## Trigger source
  TRIGGER_SOURCE:      _LLchr
  TRIGGER_SOURCE       = _LLchr("TRIGGER_SOURCE",
				(LLsrv.DEVICE_CONFIG, 0x000c))

  ## Number of peaks
  NB_PEAKS:            _LLchr
  NB_PEAKS             = _LLchr("NB_PEAKS",
				(LLsrv.DEVICE_CONFIG, 0x000d))

  ## Report zero results toggle
  REPORT_ZERO_RESULTS: _LLchr
  REPORT_ZERO_RESULTS  = _LLchr("REPORT_ZERO_RESULTS",
				(LLsrv.DEVICE_CONFIG, 0x000e))

  ## Trigger type
  TRIGGER_TYPE:        _LLchr
  TRIGGER_TYPE         = _LLchr("TRIGGER_TYPE",
				(LLsrv.DEVICE_CONFIG, 0x000f))

  ## Trigger division
  TRIGGER_DIVISION:    _LLchr
  TRIGGER_DIVISION     = _LLchr("TRIGGER_DIVISION",
				(LLsrv.DEVICE_CONFIG, 0x0010))

  # HW_CONFIG service characteristics

  ## Calibrated angles table
  CALIBRATED_ANGLES:   _LLchr
  CALIBRATED_ANGLES    = _LLchr("CALIBRATED_ANGLES",
				(LLsrv.HW_CONFIG, 0x0008))

  # RESULTS service characteristics

  ## Rangefinding results
  RANGE:               _LLchr
  RANGE                = _LLchr("RANGE",
				(LLsrv.RESULTS, 0x0003))

  ## Measuring rate
  MEASURING_RATE:      _LLchr
  MEASURING_RATE       = _LLchr("MEASURING_RATE",
				(LLsrv.RESULTS, 0x0008))

  # DEBUG service characteristics

  ## Reset device
  RESET_DEVICE:        _LLchr
  RESET_DEVICE         = _LLchr("RESET_DEVICE",
				(LLsrv.DEBUG, 0x0007))



class LLsta:
  """! Status codes

  @ingroup Enums
  """

  ## Last command successful
  OK:                    _LLsta
  OK                     = _LLsta("OK",                    0x0000)

  ## Characteristic decoding failed
  CHAR_DECODING_FAILED:  _LLsta
  CHAR_DECODING_FAILED   = _LLsta("CHAR_DECODING_FAILED",  0x0001)

  ## Characteristic out of range
  CHAR_OUT_OF_RANGE:     _LLsta
  CHAR_OUT_OF_RANGE      = _LLsta("CHAR_OUT_OF_RANGE",     0x0002)

  ## Procedure in progress
  PROCEDURE_IN_PROGRESS: _LLsta
  PROCEDURE_IN_PROGRESS  = _LLsta("PROCEDURE_IN_PROGRESS", 0x0003)

  ## Procedure failed
  PROCEDURE_FAILED:      _LLsta
  PROCEDURE_FAILED       = _LLsta("PROCEDURE_FAILED",      0x0004)

  ## Operation not allowed
  NOT_ALLOWED:           _LLsta
  NOT_ALLOWED            = _LLsta("NOT_ALLOWED",           0x0006)



## Main classes
# @defgroup MainClasses Main classes

class LLresponse:
  """! Decoded response class

  @ingroup MainClasses
  """

  # ## Methods
  def __init__(self: LLresponse,
		cmd: Optional[_LLcmd] = None,
		msgid: Optional[int] = None,
		status: Optional[_LLsta] = None,
		char: Optional[_LLchr] = None,
		recv_local_timestamp: Optional[datetime] = None) \
		-> None:
    """! %__init__ method
    """

    ## Reception timestamp (local time)
    self.recv_local_timestamp: Optional[datetime]
    self.recv_local_timestamp = recv_local_timestamp

    if cmd is not None:

      ## Command / command response
      self.cmd: _LLcmd
      self.cmd = cmd

    if msgid is not None:

      ## Message ID
      self.msgid: int
      self.msgid = msgid

    if status is not None:

      ## Status (only present if the response isn't a notification)
      self.status: _LLsta
      self.status = status

    if char is not None:

      ## Characteristic (only present if the response is a notification
      ## or a response to a read command)
      self.char: _LLchr
      self.char = char

    if False:	# Doxygen requires an assignment to list the attribute

      ## Attribute present in read command responses for characteristic
      ## SERIAL_NUMBER
      self.value: int
      self.value = property

      ## Attribute present in read command responses for characteristic
      ## FW_VERSION
      self.major: int
      self.major = property

      ## Attribute present in read command responses for characteristic
      ## FW_VERSION
      self.minor: int
      self.minor = property

      ## Attribute present in read command responses for characteristic
      ## FW_VERSION
      self.bugfix: int
      self.bugfix = property

      ## Attribute present read command responses for characteristic
      ## NETWORK and DEFAULT_NETWORK
      self.addr: ipaddress
      self.addr = property

      ## Attribute present in read command responses for characteristics
      ## NETWORK and DEFAULT_NETWORK
      self.defaultgw: ipaddress
      self.defaultgw = property

      ## Attribute present in read command responses for characteristics
      ## NETWORK and DEFAULT_NETWORK
      self.netmask: ipaddress
      self.netmask = property

      ## Attribute present in read command responses for characteristics
      ## NETWORK and DEFAULT_NETWORK
      self.port: int
      self.port = property

      ## Attribute present in read command responses for characteristic
      ## TIME
      self.time: timedelta
      self.time = property

      ## Attribute present in notifications or read command responses
      ## for characteristic TEMPERATURE
      self.temperature: float
      self.temperature = property

      ## Attribute present in read command responses for characteristic
      ## MAC
      self.mac: macaddress
      self.mac = property

      ## Attribute present in read command responses for characteristics
      ## MIN_DISTANCE and MAX_DISTANCE
      self.distance: float
      self.distance = property

      ## Attribute present in read command responses for characteristics
      ## MIN_ANGLE and MAX_ANGLE
      self.angle: float
      self.angle = property

      ## Attribute present in read command responses for characteristic
      ## TRIGGER_SOURCE
      self.source: int
      self.source = property

      ## Attribute present in read command responses for characteristic
      ## TRIGGER_TYPE
      self.type: int
      self.type = property

      ## Attribute present in read command responses for characteristic
      ## TRIGGER_DIVISION
      self.div: int
      self.div = property

      ## Attribute present in read command responses for characteristic
      ## MEASURING_RATE
      self.frequency: int
      self.frequency = property

      ## Attribute present in read command responses for characteristic
      ## AMPLITUDE_THRESHOLD
      self.threshold: int
      self.threshold = property

      ## Attribute present in read command responses for characteristic
      ## CALIBRATED_ANGLES
      self.angles: List[int]
      self.angles = property

      ## Attribute present in read command responses for characteristic
      ## NB_PEAKS
      self.peaks: int
      self.peaks = property

      ## Attribute present in notifications or read command responses
      ## for characteristic RANGE
      self.timestamp: timedelta
      self.timestamp = property

      ## Attribute present in notifications or read command responses
      ## for characteristic RANGE
      self.measurementid: int
      self.measurementid = property

      ## Attribute present in notifications or read command responses
      ## for characteristic RANGE
      self.nbresults: int
      self.nbresults = property

      ## Attribute present in notifications or read command responses
      ## for characteristic RANGE
      self.targets: List[Tuple[float, float, int]]

      ## Attribute present in read command responses for characteristic
      ## RESET
      self.reset: int
      self.reset = property



  def __repr__(self: LLresponse) \
		-> str:
    """! Generate a printable description of the response

    @return printable response
    """

    s: str = "cmd: {}, msgid: {}".format(
			self.cmd.name if "cmd" in self.__dict__ else "/",
			self.msgid if "msgid" in self.__dict__ else "/")

    if "status" in self.__dict__:
      s += ", status: {}".format(self.status.name)

    if "char" in self.__dict__:
      s += ", srv: {}, chr: {}".format(self.char.value[0].name, self.char.name)

    p: str
    for p in self.__dict__:
      if p not in ("cmd", "msgid", "status", "char"):
        s += ", {}: {}".format(p, self.__dict__[p].__repr__())

    return s



## Type definitions
#

## @cond # Excluded from Doxygen

_SERVICE_INFO_TYPE = _LLcmd

_CHR_DECODED_ENTRY_TYPE = Union[
				  Type[bool],
				  Type[str],
				  Type[int],
				  Type[float],
				  Type[list],
				  Type[ipaddress],
				  Type[macaddress],
				  Type[timedelta]
				]

_CHR_ARGS_TYPE = Union[
			  bool,
			  str,
			  int,
			  float,
			  List[Tuple[float, float, int]],
			  ipaddress,
			  macaddress,
			  timedelta,
			  List[int],
			  Tuple[int, ...]
			]

_ENC_DEC_FCTS_VAL_TYPE = Tuple[
				  Tuple[_CHR_DECODED_ENTRY_TYPE, ...],
				  Callable,
				  int,
				  Callable
				]

_CHR_INFO_TYPE = Union[
			  _LLcmd,
			  Dict[
			    str,
			    _ENC_DEC_FCTS_VAL_TYPE
			  ]
			]



## ICD descriptors
#

# ## Statuses' applicable commands
_LLSTA_CMDS: Dict[_LLsta, Tuple[_LLcmd, ...]] = {
    LLsta.OK:                    (LLcmd.READ, LLcmd.WRITE,
                                 LLcmd.NOTIFICATION, LLcmd.SET_NOTIFICATION,
                                 LLcmd.SAVE_SERVICE, LLcmd.RESTORE_SERVICE),

    LLsta.CHAR_DECODING_FAILED:  (LLcmd.READ, LLcmd.WRITE,
                                 LLcmd.SET_NOTIFICATION),

    LLsta.CHAR_OUT_OF_RANGE:     (LLcmd.READ, LLcmd.WRITE),

    LLsta.PROCEDURE_IN_PROGRESS: (LLcmd.READ, LLcmd.WRITE),

    LLsta.PROCEDURE_FAILED:      (LLcmd.ERROR, LLcmd.READ, LLcmd.WRITE,
                                 LLcmd.SAVE_SERVICE, LLcmd.RESTORE_SERVICE),

    LLsta.NOT_ALLOWED:           (LLcmd.READ, LLcmd.WRITE,
                                 LLcmd.SET_NOTIFICATION,
                                 LLcmd.SAVE_SERVICE, LLcmd.RESTORE_SERVICE)
  }



# ## Characteristics fields types description
_CHR_FTYPE: Dict[str, _ENC_DEC_FCTS_VAL_TYPE] = {
  # parameter type descriptive name: (
  #   accepted argument types for encoding,
  #   list, number of parameters to encode > encoding function for value p,
  #   number of encoded bytes per element,
  #   bytes, number of parameters to decode > decoding function for bytes b
  # ),
    "character": (
      (str,),						# Accepted arg types
      lambda p, n: p.encode("ascii"),			# Encoding function
      1,						# Bytes per element
      lambda b, n: b.decode("ascii")			# decoding function
    ),
    "unsigned byte bool": (
      (bool,),						# Accepted arg types
      lambda p, n: pack("B" * n,
			*[1 if v else 0 for v in p]),	# encoding function
      1,						# Bytes per element
      lambda b, n: [v != 0 for v in unpack("B" * n, b)]	# decoding function
    ),
    "unsigned byte": (
      (int,),						# Accepted arg types
      lambda p, n: pack("B" * n, *p),			# encoding function
      1,						# Bytes per element
      lambda b, n: unpack("B" * n, b)			# decoding function
    ),
    "short angle": (					# Scale: 1/1000°
      (int, float),					# Accepted arg types
      lambda p, n: pack("<" + "h" * n,
			*[round(v * 1000) for v in p]),	# encoding function
      2,						# Bytes per element
      lambda b, n: [v * 1e-3 for v in \
			unpack("<" + "h" * n, b)]	# decoding function
    ),
    "unsigned short": (
      (int,),						# Accepted arg types
      lambda p, n: pack("<" + "H" * n, *p),		# encoding function
      2,						# Bytes per element
      lambda b, n: unpack("<" + "H" * n, b)		# decoding function
    ),
    "unsigned short dist": (				# Scale: 0.1 mm
      (int, float),					# Accepted arg types
      lambda p, n: pack("<" + "H" * n,
			*[round(v *10000) for v in p]),	# encoding function
      2,						# Bytes per element
      lambda b, n: [v * 1e-4 for v in \
			unpack("<" + "H" * n, b)]	# decoding function
    ),
    "int dist": (					# Scale: 0.1 mm
      (int, float),					# Accepted arg types
      lambda p, n: pack("<" + "i" * n,
			*[round(v *10000) for v in p]),	# encoding function
      4,						# Bytes per element
      lambda b, n: [v * 1e-4 for v in \
			unpack("<" + "i" * n, b)]	# decoding function
    ),
    "int": (
      (int,),						# Accepted arg types
      lambda p, n: pack("<" + "i" * n, *p),		# encoding function
      4,						# Bytes per element
      lambda b, n: unpack("<" + "i" * n, b)		# decoding function
    ),
    "unsigned int bool": (
      (bool,),						# Accepted arg types
      lambda p, n: pack("<" + "I" * n,
			*[1 if v else 0 for v in p]),	# encoding function
      4,						# Bytes per element
      lambda b, n: [v != 0 for v in \
			unpack("<" + "I" * n, b)]	# decoding function
    ),
    "unsigned int": (
      (int,),						# Accepted arg types
      lambda p, n: pack("<" + "I" * n, *p),		# encoding function
      4,						# Bytes per element
      lambda b, n: unpack("<" + "I" * n, b)		# decoding function
    ),
    "target": (
      (list,),						# Accepted arg types
      lambda p, n: pack("<" + "IhH" * n, *sum(
			[(round(v[0] * 10000),
				round(v[1] * 1000),
				v[2]) \
			for v in p], ())),		# encoding function
      8,						# Bytes per element
      lambda b, n: [(v[0] * 1e-4, v[1] * 1e-3, v[2]) \
			for v in iter_unpack("<IhH",b)]	# Decoding function
    ),
    "ip address": (
      (ipaddress,),					# Accepted arg types
      lambda p, n: b"".join([v.ip for v in p]),		# encoding function
      4,						# Bytes per element
      lambda b, n: [ipaddress(b[i * 4 : i * 4 + 4]) \
			for i in range(n)]		# decoding function
    ),
    "mac address": (
      (macaddress,),					# Accepted arg types
      lambda p, n: b"".join([v.mac for v in p]),	# encoding function
      6,						# Bytes per element
      lambda b, n: [macaddress(b[i * 6 : i * 6 + 6]) \
			for i in range(n)]		# decoding function
    ),
    "time": (
      (timedelta,),					# Accepted arg types
      lambda p, n: pack("<" + "Q" * n,
			*[int(v.total_seconds() * 1e7) \
				for v in p]),		# encoding function
      8,						# Bytes per element
      lambda b, n: [timedelta(seconds = v / 1e7) for v in \
			unpack("<" + "Q" * n, b)]	# decoding function
    ),
    "float": (
      (int, float),					# Accepted arg types
      lambda p, n: pack("<" + "f" * n, *p),		# encoding function
      4,						# Bytes per element
      lambda b, n: unpack("<" + "f" * n, b)		# decoding function
    )
  }



# ## Services description
_SRV_DESC: Dict[_LLsrv, Dict[str, Tuple[_SERVICE_INFO_TYPE, ...]]] = {
  # service: {
  #   "supported_cmds": (command, command, ...),
  #   "restricted_cmds": (command, command, ...),
  # },
    LLsrv.DEVICE_INFO: {
      "supported_cmds":  (LLcmd.RESTORE_SERVICE, LLcmd.SAVE_SERVICE),
      "restricted_cmds": (),
    },
    LLsrv.DEVICE_CONFIG: {
      "supported_cmds":  (LLcmd.RESTORE_SERVICE, LLcmd.SAVE_SERVICE),
      "restricted_cmds": (),
    },
    LLsrv.HW_CONFIG: {
      "supported_cmds":  (LLcmd.RESTORE_SERVICE, LLcmd.SAVE_SERVICE),
      "restricted_cmds": (),
    },
    LLsrv.RESULTS: {
      "supported_cmds":  (),
      "restricted_cmds": (),
    }
  }



# ## Characteristics description
_CHR_DESC: Dict[_LLchr, Dict[str, Tuple[ _CHR_INFO_TYPE, ...]]] = {
  # characteristic: {
  #   "supported_cmds": (command, command, ...),
  #   "restricted_cmds": (command, command, ...),
  #   "fields": (
  #     {"name": (accepted argument types for encoding,
  #               encoding function,
  #               number of encoded bytes per element,
  #               decoding function)},
  #     ...
  #   )
  # },
    LLchr.SERIAL_NUMBER: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (LLcmd.WRITE,),
      "fields": (
        {"value":                  _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.FW_VERSION: {
      "supported_cmds":  (LLcmd.READ,),
      "restricted_cmds": (),
      "fields": (
        {"major":                  _CHR_FTYPE["unsigned int"]},
        {"minor":                  _CHR_FTYPE["unsigned int"]},
        {"bugfix":                 _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.NETWORK: {
      "supported_cmds":  (LLcmd.READ,),
      "restricted_cmds": (),
      "fields": (
        {"addr":                   _CHR_FTYPE["ip address"]},
        {"defaultgw":              _CHR_FTYPE["ip address"]},
        {"netmask":                _CHR_FTYPE["ip address"]},
        {"port":                   _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.TIME: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"time":                   _CHR_FTYPE["time"]},
      )
    },
    LLchr.TEMPERATURE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.SET_NOTIFICATION),
      "restricted_cmds": (),
      "fields": (
        {"temperature":            _CHR_FTYPE["float"]},
      )
    },
    LLchr.MAC: {
      "supported_cmds":  (LLcmd.READ,),
      "restricted_cmds": (),
      "fields": (
        {"mac":                    _CHR_FTYPE["mac address"]},
        {"_":                      _CHR_FTYPE["unsigned short"]},
      )
    },
    LLchr.DEFAULT_NETWORK: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"addr":                   _CHR_FTYPE["ip address"]},
        {"defaultgw":              _CHR_FTYPE["ip address"]},
        {"netmask":                _CHR_FTYPE["ip address"]},
        {"port":                   _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.MIN_DISTANCE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"distance":               _CHR_FTYPE["float"]},
      )
    },
    LLchr.MAX_DISTANCE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"distance":               _CHR_FTYPE["float"]},
      )
    },
    LLchr.AMPLITUDE_THRESHOLD: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"threshold":              _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.MIN_ANGLE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"angle":                  _CHR_FTYPE["float"]},
      )
    },
    LLchr.MAX_ANGLE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"angle":                  _CHR_FTYPE["float"]},
      )
    },
    LLchr.TRIGGER_SOURCE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"source":                 _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.NB_PEAKS: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"peaks":                  _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.REPORT_ZERO_RESULTS: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"on":                     _CHR_FTYPE["unsigned int bool"]},
      )
    },
    LLchr.TRIGGER_TYPE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"type":                   _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.TRIGGER_DIVISION: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"div":                    _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.CALIBRATED_ANGLES: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (LLcmd.WRITE,),
      "fields": (
        {"angles[256]":            _CHR_FTYPE["short angle"]},
      )
    },
    LLchr.RANGE: {
      "supported_cmds":  (LLcmd.SET_NOTIFICATION,),
      "restricted_cmds": (),
      "fields": (
        {"timestamp":              _CHR_FTYPE["time"]},
        {"measurementid":          _CHR_FTYPE["unsigned int"]},
        {"nbresults":              _CHR_FTYPE["unsigned int"]},
        {"triggercounter":         _CHR_FTYPE["int"]},
        {"targets[nbresults]":     _CHR_FTYPE["target"]},
      )
    },
    LLchr.MEASURING_RATE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"frequency":              _CHR_FTYPE["unsigned int"]},
      )
    },
    LLchr.RESET_DEVICE: {
      "supported_cmds":  (LLcmd.READ, LLcmd.WRITE),
      "restricted_cmds": (),
      "fields": (
        {"reset":                  _CHR_FTYPE["unsigned int bool"]},
      )
    }
  }

##\endcond # /Excluded from Doxygen
