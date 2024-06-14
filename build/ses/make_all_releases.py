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

# Import TL modules
importError = True
if importError:	
	try:
		sys.path.insert(1, '../../../../../../TL_scripts/TL_common_python_modules')
		import server_api_if as api
		import tl_check_command_IDs
		import tl_xcheck_server_command_doc
	except:
		importError = True

if importError:	
	try:
		sys.path.insert(1, '.')
		import server_api_if as api
		import tl_check_command_IDs
		import tl_xcheck_server_command_doc
	except:
		print("ERROR: Could not find TL-specific packages required for this tool!")
		sys.exit(1)
		
if sys.version_info[0] < 3:
	print("\n")
	print("***********************************************")
	print("** The script requires Python 3.0 or higher! **")
	print("***********************************************")
	print("\n")
	print("conda activate Python3")
	print("python make_all_releases.py")
	print("\n")
	exit()

# Target project.
# The keys of the hwPlatforms dictionary are the build targets. The values are lists of HW platforms to which the build corresponds.
# The HW platforms appear as selectable object in XSManager -> Production -> Add new firmware
hwPlatforms = {
				"TL500":	["TL500", "TL500-EX2"],
				"TL501":	["TL501"],
				"TL502":	["TL502", "TL503", "TL504"],
				"TL510":	["TL510"],
				"TL512":	["TL512"]
			  }
	
# Directory containing the Segger C compiler.
emStudioDir = "C:\\Program Files\\SEGGER\\SEGGER Embedded Studio for ARM 5.42a\\bin"

# SDK Directory containing the SD binary.
NordicSDK = "C:\\Nordic\\nRF5_SDK_17.1.0_ddde560\\components\\softdevice\\s140\\hex\\"

# SDK name of the SD.
SoftDeviceSDKName = "s140_nrf52_7.2.0_softdevice.hex"

# Name under which the SD update files are generated. 
# ATTENTION: No extensions! Extensions will be added by the script.
SoftDeviceTargetName = "sd_s140_7.2.0"							

# Private key for signing the FW. Any FW update must use the same key to be accepted.
PrivateEncrKey = "privateTRAXxsBLE.pem"					



"""
Main
"""
#
# Parse the command line arguments.
#
parser = argparse.ArgumentParser(description = 'Make release files for all TL5xx targets. The version number is taken from git.')
parser.add_argument('--config', dest = 'config', help = 'target configuration (\'DBG\' or \'REL\'), default REL.', default = "REL")
parser.add_argument('--ignoregit', dest = 'ignoregit', help = 'ignore if not all changes are checked into git. ', action = "store_true")
arguments = parser.parse_args()
config = arguments.config.upper()

if config != "REL" and config != "DBG":
	print("ERROR: Configuration must be either (\'DBG\' or \'REL\'!")
	exit()
	

# Check if everything has been checked in.
if not arguments.ignoregit:
	process = subprocess.Popen(["git", "status"], stdout = subprocess.PIPE, universal_newlines = True)
	stdout, stderr = process.communicate()
	if not("Your branch is up to date with" in stdout):
		print("ERROR: Branch is not up to date!")
		print("Use switch \'-ignoregit\' to override.")
		exit()
		
	if "modified:" in stdout:
		print("ERROR: Not all modifications checked in!")
		print("Use switch \'-ignoregit\' to override.")
		exit()

# Cross-check the consistency of server command IDs over all active TL projects.
print("Checking server command ID consistency over all active TL projects...")
if not tl_check_command_IDs.CheckCommandIdConsistency():
	print("ERROR: Server command ID are not consistent!")
	exit()
else:
	print("Server command IDs are consistent.")
	
# Cross-check the server commands in the code with the ones in the documentation.
print("Checking server command consistency between code base and documentation...")
if not tl_xcheck_server_command_doc.XCheckServerCommandDoc("..\\..\\TL500 server_commands.txt", "..\\..\\source\\parser.c"):
	print("ERROR: Server commands in documentation are not consistent with the code base!")
	exit()
else:
	print("Server commands in documentation are consistent with the code base.")
	
# Get the version number.
process = subprocess.Popen(["git", "describe"], stdout = subprocess.PIPE, universal_newlines = True)
stdout, stderr = process.communicate()
revisionStrg = re.search("^(V\d+\.\d+(-*\d*)?).*", stdout.upper())
revision = revisionStrg.group(1).upper()
print()
print("************************************************************************************")
print("New revision number: %s" % revision)
print("************************************************************************************")
print()

# Generate the release binaries
print()

for target in hwPlatforms.keys():
	# make release, force rebuild
	print("************************************************************************************")
	print("make clean %s, configuration %s" % (target, config))
	print("************************************************************************************")
	print()
	if config == "DBG":
		target_config = "Debug"
	else:
		target_config = "Release"
			
	if subprocess.call([emStudioDir + "\\emBuild.exe", "-rebuild", "-config", target_config, "-verbose", target + ".emProject"]) != 0:
		print()
		print("*")
		print("* ERROR: make release failed for target %s, configuration %s!" % (target, config))
		print("*")
		exit()

	# Copy and rename FW result file
	print("************************************************************************************")
	print("copy and rename .HEX file")
	print("************************************************************************************")
	print()
	shutil.copy2("Output\\%s\\Exe\\%s.hex" % (target_config, target), "..\\..\\releases\\%s_%s_%s.hex" % (target, config, revision))

	# Copy and rename SD file
	print("************************************************************************************")
	print("copy and rename SD .HEX file")
	print("************************************************************************************")
	print()
	shutil.copy2(NordicSDK + SoftDeviceSDKName, "..\\..\\releases\\" + SoftDeviceTargetName + ".hex")

# Change to the appropriate SDK pc_nrfutil directory for the following steps.
cwd = os.getcwd()
os.chdir(r"..\\..\\..\\TRAXxs_pc-nrfutil\\v6.1.3")

for target in hwPlatforms.keys():
	# Generate FW package file
	print("************************************************************************************")
	print("generate FW package file")
	print("************************************************************************************")
	print()
	if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "pkg", "generate",
					 "--hw-version", "52",
					 "--application-version", "1",
					 "--sd-req", "0x100",
					 "--application", "..\\..\\TL500\\build\\ses\\Output\\%s\\Exe\\%s.hex" % (target_config, target),
					 "--key-file", "..\\privateTRAXxsBLE.pem",
					 "..\\%s_%s_%s_pkg.zip" % (target, config, revision)]) != 0:		
		print()
		print("*")
		print("ERROR: could not generate FW package file!")
		print("*")
		exit()
		
	# Generate bootloader settings file
	print("************************************************************************************")
	print("generate bootloader settings file")
	print("************************************************************************************")
	print()
	if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "settings", "generate",
					 "--family", "NRF52840",
					 "--application", "..\\..\\TL500\\build\\ses\\Output\\%s\\Exe\\%s.hex" % (target_config, target),
					 "--application-version", "1",
					 "--bootloader-version", "1",
					 "--bl-settings-version", "1",
					 "..\\bl_settings_%s_%s_%s.hex" % (target, config, revision)]) != 0:		
		print()
		print("*")
		print("ERROR: could not generate bootloader settings file!")
		print("*")
		
		exit()
		
	# Copy bootloader settings file
	print("************************************************************************************")
	print("copy bootloader settings")
	print("************************************************************************************")
	print()
	shutil.copy2("..\\bl_settings_%s_%s_%s.hex" % (target, config, revision), "..\\..\\TL500\\releases")

	# Generate the FW-update script file
	print("************************************************************************************")
	print("generate FW-update script file")
	print("************************************************************************************")
	print()
	if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "dfu", "traxxs-generate",
					"-pkg", "..\\%s_%s_%s_pkg.zip" % (target, config, revision),
					"-ulf", "..\\%s_%s_%s.scr" % (target, config, revision)]) != 0:		
		print()
		print("*")
		print("ERROR: could not generate FW-update script file!")
		print("*")
		exit()
		
	# Generate SD package file
	print("************************************************************************************")
	print("generate SD package file")
	print("************************************************************************************")
	print()
	if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "pkg", "generate",
					 "--hw-version", "52",
					 "--sd-req", "0x100",
					 "--softdevice", NordicSDK + SoftDeviceSDKName,
					 "--key-file", "..\\privateTRAXxsBLE.pem",
					 "..\\" + SoftDeviceTargetName + ".zip"]) != 0:		
		print()
		print("*")
		print("ERROR: could not generate SD package file!")
		print("*")
		exit()
		
	# Generate the SD-update script file
	print("************************************************************************************")
	print("generate SD-update script file")
	print("************************************************************************************")
	print()
	if subprocess.call(["python", ".\\nordicsemi\\__main__.py", "dfu", "traxxs-generate",
					"-pkg", "..\\" + SoftDeviceTargetName + ".zip",
					"-ulf", "..\\" + SoftDeviceTargetName + ".scr"]) != 0:		
		print()
		print("*")
		print("ERROR: could not generate SD-update script file!")
		print("*")
		exit()
		
	# Copy FW update script file
	print("************************************************************************************")
	print("copy FW-update script file")
	print("************************************************************************************")
	print()
	shutil.copy2("..\\%s_%s_%s.scr" % (target, config, revision), "..\\..\\TL500\\releases")

	# Copy SD update script file
	print("************************************************************************************")
	print("copy SD-update script file")
	print("************************************************************************************")
	print()
	shutil.copy2("..\\" + SoftDeviceTargetName + ".scr", "..\\..\\TL500\\releases")

	# SCP the FW .scr file to the server
	subprocess.call(["scp", "..\\%s_%s_%s.scr" % (target, config, revision), "ubuntu@firmup.xsole.net:/var/traxxs/firmup"])

	# SCP the SD .scr file to the server
	subprocess.call(["scp", "..\\" + SoftDeviceTargetName + ".scr", "ubuntu@firmup.xsole.net:/var/traxxs/firmup"])

# Generate .zip
print()
print("************************************************************************************")
print("generate .ZIP")
print("************************************************************************************")
print()
zipExeStrg = "c:\\Program Files\\7-Zip\\7z.exe"
zipArgStrg = "..\\%s_%s_%s.zip" % (list(hwPlatforms.keys())[0], config, revision)
subprocess.call([zipExeStrg, "a", zipArgStrg, "..\\..\\TL500"])

# Copy .zip file to archive
print("************************************************************************************")
print("copy .zip file to archive")
print("************************************************************************************")
print()
shutil.copy2("..\\%s_%s_%s.zip" % (list(hwPlatforms.keys())[0], config, revision), "..\\..\\..\\..\\archive")
os.remove("..\\%s_%s_%s.zip" % (list(hwPlatforms.keys())[0], config, revision))

# Go back to original  directory
os.chdir(cwd)

# Add the firmware to the list on the server.
api.SignInToServer('ngrunert+fwtest@traxxs.net' , '#Traxxs#fwtest1')
for target in hwPlatforms.keys():
	api.AddProductionFirmware("%s_%s_%s.scr" % (target, config, revision), "core", hwPlatforms[target], flags = ['beta'])

# Generate changelog.
cwd = os.getcwd()
os.chdir(r"..\\..\\..\\..\\..\\changelogs")

if subprocess.call(["git", "pull"]) != 0:
	print()
	print("*")
	print("* ERROR: could not pull git repository for changelogs!")
	print("*")
	exit()

if subprocess.call(["git", "checkout"]) != 0:
	print()
	print("*")
	print("* ERROR: could not checkout git repository for changelogs!")
	print("*")
	exit()

if subprocess.call(["python", "./changelog_update.py", "git@gitlab.com:traxxs/firmware/application_nina_freertos", "TL500", "--new_release"]) != 0:
	print()
	print("*")
	print("* ERROR: generating changelogs!")
	print("*")
	exit()

os.chdir(cwd)


# Add reminder for the rest to do manually
print()
print()
print("************************************************************************************")
print("* Done for the automatic part.                                       ")
print("*                                                                    ")
print("* Don't forget to:                                                   ")
print("*     1. Copy the .zip to the NAS.                                   ")
print("*     2. If required, add the SD .SCR file name to the server:       ")
print("*        %s.scr                           	                        " % (SoftDeviceTargetName))
print("************************************************************************************")



