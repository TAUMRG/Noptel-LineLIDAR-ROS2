"""! %LineLidar Low-level communication class

@file default.py

@mainpage Python3 %LineLidar class

@section main_page_description Description
Low-level class to communicate with a %LineLidar device in Python

Default parameters

@section main_page_notes Notes
Copyright 2023 (c) Noptel Oy, Oulu Finland

Tested on:
  - Linux
  - Windows

Authors:
  PCo
"""

from __future__ import annotations



## Default parameters
# @defgroup DefaultParameters Default parameters

## Default UDP port
_ll_default_port: int
_ll_default_port = 9907

## Default UDP communication timeout in seconds
_ll_default_udp_timeout: float
_ll_default_udp_timeout = 1.5 #s

## Default SSH communication timeout in seconds
_ll_default_ssh_timeout: float
_ll_default_ssh_timeout = 5 #s

## Default reboot timeout in seconds
_ll_default_reboot_timeout: float
_ll_default_reboot_timeout = 5 #s

## Default number of retries for fault-tolerance
_ll_default_retries: int
_ll_default_retries = 1

## Default depth of the sent commands log
_ll_default_sent_cmds_log_depth: int
_ll_default_sent_cmds_log_depth = 16

## Default path for the Python executable on the SSH relay host
_ll_default_ssh_python_path: str
_ll_default_ssh_python_path = "python"
