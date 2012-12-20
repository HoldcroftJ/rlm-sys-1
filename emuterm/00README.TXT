This directory contains the 'emuterm', a util that acts as a terminal
for interacting with a RUSSBOX virtual serial device.

The RUSSBOX virtual OS-9/68k system has one (or more) serial devices, which
are ports to the outside world via virtual RS-232. They are very simple, with
no hardware handshaking and no meaningful baud rate settings (since they go
through virtual channels), though they have registers which could, in theory,
be used to specify a real rate when going over a hosts system's real serial
devices.

