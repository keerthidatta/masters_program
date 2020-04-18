################################################################################
#		Copyright Notice and Disclaimer for the mosartMCU Project.
# Copyright (C) 2016 EAS Group. All rights reserved.
# 
# This file was released for educational use within the 
#             Embedded Automotive Systems Group, 
#             Institute of Technical Informatics,
#           Graz University of Technology, Austria
# This copyright may not be removed, modified or relocated within this file.
################################################################################

################################################################################
## MCSmartOS build environment V0.3
################################################################################


## Supported build targets:
##       all: build the middleware, and put it into the lib directory
##     clean: remove all build results *excluding* the middleware itself
##  libclean: remove all build results *including* the middleware


## Specific build variables:
##   SOURCES: List of sources to be compiled and linked (will only consider *.c *.S)
##   DEFINES: List of arbitrary defines. These will simply be passed to the compiler/assembler
##              Example: "DEFINES += XXX" or "DEFINES += XXX=1"
SOURCES 	= $(wildcard *.c) $(wildcard *.S)
SOURCES 	+= $(wildcard ../_common/*.c) $(wildcard ../_common/*.S)
DEFINES		+=

## Specific automatic build variables, generated from the directories:
##    _BOARD: (auto) The target hardware for which to build the middleware in format CPU[.PCB].
##       CPU: (auto) The target CPU, extracted from _BOARD.
##      TYPE: (auto) The middleware type: Exactly one of the following
##						* kernel	operating system kernels
##						* arch		processor-specific: drivers for on-chip peripherals
##						* drv		processor-independent: drivers for off-chip peripherals
##						* lib		processor-independent: libraries without tasks
##						* services	processor-independent: libraries with tasks
##      NAME: (auto) Name of the middleware. Final target filename: $(TYPE).$(NAME).$(_BOARD).a
_BOARD		= $(notdir $(abspath $(CURDIR)))
CPU 		= $(basename $(_BOARD))
TYPE 		= $(notdir $(abspath $(CURDIR)/../..))
NAME		= $(TYPE).$(notdir $(abspath $(CURDIR)/..))

## Environment build variables:
##	 GROUP: (auto) The group name where the libs will be taken from
##	PUBLIC: The directory name of the public repository (where public libs are)
GROUP 		= $(notdir $(abspath $(CURDIR)/../../../../..))
PUBLIC = RTOS_Public_SS17

## Generic build variables:
##      ROOT: Relative path to the SmartOS root directory
##   MAKELIB: The make library to be used
ROOT		= ../../../../../..
MAKELIB	= $(ROOT)/$(PUBLIC)/buildenv/makelib/middleware.make

include $(MAKELIB)
