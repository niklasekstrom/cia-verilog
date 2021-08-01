# Implementation of 8520 Complex Interface Adapter (CIA) in Verilog

I believe this should be mostly working, but it hasn't been tested so it also most likely contains a few bugs.

Compiles in Quartus and uses 483 logic elements in an Intel MAX II CPLD.

## Changes that could be made to reduce LEs

- The Time of Day clock (TOD) feature could probably be removed or significantly simplified to save LEs.
- The serial port implementation could be simplified a bit, considering how it's used in the Amiga.
