#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

Generate release files and push them to the servers

"""

import argparse
import subprocess
import re
import sys, os
import time
import msvcrt
import shutil


if sys.version_info[0] < 3:
	print("\n")
	print("***********************************************")
	print("** The script requires Python 3.0 or higher! **")
	print("***********************************************")
	print("\n")
	print("conda activate Python3")
	print("python make_BL_settings.py")
	print("\n")
	exit()



"""
Main
"""
#
# Parse the command line arguments.
#
parser = argparse.ArgumentParser(description = 'Generate the bootloader setting file for the specified TL5xx build.')
parser.add_argument('-target', dest = 'target', help = 'target device', default = "TL500")
parser.add_argument('-config', dest = 'config', help = 'target configuration (\'DBG\', \'DBG_NO_SD\' or \'REL\'', default = "REL")
arguments = parser.parse_args()
config = arguments.config.upper()
target = arguments.target.upper()


if config != "REL" and config != "DBG" and config != "DBG_NO_SD":
	print("ERROR: Configuration must be either (\'DBG\', \'DBG_NO_SD\' or \'REL\'!")
	exit()
if config == "DBG":
	target_config = "Debug"
elif config == "DBG_NO_SD":
	target_config = "Debug No SD"	
else:
	target_config = "Release"	
	
	
# Change to the appropriate SDK pc_nrfutil directory for the following steps.
cwd = os.getcwd()
os.chdir("..\\..\\..\\TRAXxs_pc-nrfutil\\v6.1.3")	
	
	
# Generate bootloader settings file
print("************************************************************************************")
print("generate bootloader settings file")
print("************************************************************************************")
print()
if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "settings", "generate",
				 "--family", "NRF52840",
				 "--application", "%s\\Output\\%s\\Exe\\%s.hex" % (cwd, target_config, target),
				 "--application-version", "1",
				 "--bootloader-version", "1",
				 "--bl-settings-version", "1",
				 "%s\\Output\\%s\\Exe\\bl_settings.hex" % (cwd, target_config)]) != 0:		
	print()
	print("*")
	print("ERROR: could not generate bootloader settings file!")
	print("*")
	exit()

# Go back to original  directory
os.chdir(cwd)