
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

### [8.3-29] - 2023-03-03

    Reduced full charge current to fall better in the targeted current window.

### [8.3-28] - 2023-03-03

    Removed disabling the UART after sending a packet as this also inhibits
    the processor to receive notifications from the GSM module. TODO:
    check if there is another way to signal a communication need to the main
    processor.


### [8.3-27] - 2023-03-03

    - Add PWM power-down.
    - Removed 'AT' command ping from GSM hard-shutdown.
    - Added generic functions used in the test program: vAccPhyStop(),
      sGetBattCharge(), vTracePrintSp() and vLongToIntStrg(). This aligns both
      branches and makes merging new features easier.


### [8.3-26] - 2023-02-23

    - Added mechanism for closing the UART ports dynamically to save power.
      This concernes both GSM and TRC UART. The UART is closed in the
      TX interrupt routine as soon as the last character to transmit has been
      sent. A flag (bRequestClose) in the UART control structure has been added
      for that.
    - Also added a secret NORDIC register handling which seems to be required
      for the UART to release the HFCLK request.
    - Un-initialise the PWM HW as long as the device is not on the charger.
      This is in the hope of power reduction.
    - Translated timer for NoAbn detection from 16- to 32-bit. The code
      before contained actually a bug which set the timer to portMAX_DELAY
      if the remaining time was larger than 0xffff. portMAX_DELAY on 32-bit
      systems translates to 1193h instead of 65s (0xffff).
    - Invalidate the BLE/GPS data as soon as a STANDBY_MODE command has
      been received. There is still a slight possibility that new BLE data
      will be received between data invalidation and BLE stop in another task.


### [8.3-25] - 2023-02-17

    - Corrected bug in app_error_fault_handler() where sometimes the main stack
      instead of the process stack was accessed for stack saving resulting
      in a hard fault once the read pointer went behind the end of the RAM.
      Now checking that all accesses are within the RAM.
    - Added hardfault handler.
    - Caught all(!?!) xTimerChangePeriod() calls where the new period might
      be 0. In the case of 0, the function asserts.


### [8.3-24] - 2023-02-16

    - Corrected bug where the QTEL module was only shut down in STILL, SLEEP
      and INACTIVE states.
    - Added 2s timer when booting the module. This is to make sure that the
      current consumption stays low until the battery's PCM is enabled.


### [8.3-23] - 2023-02-14

    - Moved charger handler into own task.
    - Corrected transition to charging normal from extreme temperature
      condition. Before, the device would not go back to nominal charge
      current.
    - Made sure that normal charge current is used when placing the device
      on the charger and battery is almost full so that the charge current
      is <300mA (the Qi Rx current limitation) in unregulated mode.
    - Made current control loop slower to avoid overshoots.
    - Adjusted task stack sizes. The previous sizes were erroneously based on
      the assumption that sizes were given in bytes and not in 32-bit words.
      Thus saved ~18KB RAM.


### [8.3-22] - 2023-02-13

    - Implemented changes to charge control system to support TL501/511.
      At the same time, changed interpretation of charge settings as percentages
      for clarity (hopefully...).
    
    - Addressed issue with the accelerometer SPI handling where the nRF SPI
      driver returns "busy" even though the driver waits for each transfer to
      terminate.
    
    ATTENTION:
    When coming out of extended temperature range charging, VChrg may be above
    V_CHARGE_OK but the PWM setting is below max. In that case, the PWM is
    *not* set to nominal (CHRG_ENABLED).


### [8.3-21] - 2023-01-27

    Added attempt to avoid NRF_ERROR_BUSY condition in the accelerometer
    interrupt routine.


### [8.3-20] - 2023-01-27

    Corrected saving the stack in the app_error_fault_handler() which now
    distinguishes between main and process stack pointers. MSP is for
    interrupts only whereas PSP is for the threads.


### [8.3-19] - 2023-01-26

    Aligned project configurations for TL510, etc. to TL500


### [8.3-18] - 2023-01-26

    - Increased persistent RAM section to 64 bytes, aligned to bootloader
      v1.0-10.
    - Included more error information in the persistent RAM section and
      dump it at startup in the CTRL task to NVDS.
    - Added a few checks in the GSM task for BLE enable to make sure that BLE
      really stays off once disabled via config.
    - Beautifying a couple of code sections (adding spaces...) in
      port_cmsis_systick.c.
    - Increased persistent RAM size even more to 128bytes.
    - Made error file and line number available also in release and also
      for system resets.
      Added error ID for system resets.


### [8.3-17] - 2023-01-17

    Corrected function to determine the cellular module allwance to enter
    sleep. The GPIO readback had not been returning the pin level as the GPIO
    was configured as output.


### [8.3-16] - 2023-01-17

    Fixed power consumption in cellular idle mode.
    
    The high power consumption was caused by the Quectel module not entering
    power-save mode. The power-save is enabled by pulling DTR high but the
    corresponding GPIO on the nRF was not configured as output.


### [8.3-15] - 2023-01-16

    Added logging of the originating program addresses in case of soft resets.


### [8.3-14] - 2023-01-13

    maintenance


### [8.3-13] - 2023-01-13

    Fixed issue where the mode in the FWUPD could only have a single digit.
    In some usages, however, two digits are required, e.g. for BL and FW
    co-update (10).
    
    Fixed garbled error response if an FTP upload points to a non-existing
    file.


### [8.3-12] - 2023-01-12

    Fixed bug where the cellular module was not shut down in STANDBY.


### [8.3-11] - 2023-01-10

    Watchdog origin debug
    
    Replaced disabling of all interrupts inside macro portENTER_CRITICAL()
    with disabling only interrupts of priorities belonging to the
    application. This way, the watchdog timer interrupt intercepts the hardware
    watchdof reset and the interrupt return address be saved to uninitialised
    RAM. On reboot, the saved return address is saved to the trace buffer.
    
    This modification needs a bootloader which also preserves the unitialised
    RAM area to work.


### [8.3-10] - 2023-01-05

    Corrected that additional padding bytes were sent in between loc beacons
    inside distress beacon BLE packets.
    The padding bytes had been added by the compiler which aligned beacon fields
    to 16-bit in the xBLE_LOC_BEACON_ENTRY structure. Hence, beacon entries
    in xBleLocBcn[] and ucBleLocBeacon[] in the uBLE_LOC_BEACON_ENTRY union
    were not aligned.


### [8.3-9] - 2023-01-05

    - Corrected charger init so that battery voltage and temperature status
      will be set correctly before the OS tasks start.
    - Added battery depetion status (<=2.0V) below which the BLE will
      be suspended.
      Checking for the depletion status is in the RTC.
    - Removed BLE start from the BLE ctrl task startup. The BLE will be
      started anyway by the ctrl task's prvUnused_2_Inactive() call during
      task startup.


### [8.3-8] - 2022-12-19

    Further debug output to TRACE_GSM_GPS_NO_RECEPTION and syntax correction.


### [8.3-7] - 2022-12-19

    - Clarified uxNumSvnEntries() although this should not change the
      behaviour.
    - Added debug output to TRACE_GSM_GPS_NO_RECEPTION: fix, HDOP, H and V acc.


### [8.3-6] - 2022-12-13

    - Added 'AT' probes to the cellular module shutdown sequence to detect if
      the module is unresponsive. In those cases, convert the soft-shutdown to
      a hard one to accelerate the process.
    - Do not shut-down the GPS if no fix has been achieved and there is
      reasonable hope that the conditions will improve (module is moving).
    - In order to know whether a cellular module shut-down is useful,
      the calculation to the next cellular transmission (Hello-packet) in GPS
      recording mode is now taken into account. For this a new function
      (xGetTimeToNextHello()) has been introduced. This function dowes not
      give a precise information but merely a conservative estimation.


### [8.3-5] - 2022-12-08

    - Added over-/under-temperature blinking, mainly to detect badly connected
      batteries in production test phase III.
    - Corrected temperature conversion to ASCII-float for themperatures above/
      below 100C.
    - Fixed RFON counters.
    - Added debug output for GPS recording.


### [8.3-4] - 2022-12-02

    Added RF-module ON counters ofr GSM, GPS and BLE.


### [8.3-3] - 2022-12-01

    - Added originating program address to tracing faults.
    - Added Hello packet interval in position recording mode.


### [8.3-2] - 2022-11-07

    Corrected missing bEvacOngoing flag when receiving an evacuation via BLE.
    At the same time, merged vStartTxEvacuation() into vIndicateEvacuation().


### [8.3-1] - 2022-11-03

    - Corrected satellite counting in bGetGpsPosition(). The wrong counting
      might have resulted in shortened search times and thus no-GPS fix.
    - Corrected bad svn reporting in vSendGpsSvnField().
    - Removed uxGpsSvnCnt variable. All functions needing to know the length
      of the used xGpsSvnData field now count the number of allocated fields.
    - Moved usGPSMaxWaitForSat to long to be able to disable function.
    - Fixed bug where commands setting ulGPSMaxWaitForSat where in not in 10x
      time units.
    - Fixed bad zip file name in make_all_releases.py


### [8.3-0] - 2022-10-20

    Fixed make_all_releases.py to support tagged releases.


### [8.2-23] - 2022-10-20

    - Fixed queue auxiliary functions. Remove tyep/remove all functions did
      not work correctly.
      Made sure that these functions do not remove any server command-related
      messages.
    - Increased FIFO depth for retained queue messages in xReceiveResponse().
      The previous depth of 2 was insufficient. Also changed structure to a
      real FIFO.
    - Added function to clarify whether a GPS or any position is required:
          bCtrlPositionRequired()                            Positioning required (any type)
          bCtrlGPSPositionRequired()                 GPS positioning required


### [8.2-22] - 2022-10-13

    Corrected prvWriteLongParameterx10().


### [8.2-21] - 2022-10-13

    - Slight modification to the PDP acttivation sequence. The previous
      sequence was not working for the QTEL version 01.006.04.006_PATCH.
    - Check for GPS ON state while waiting for a fix. If the module entered
      SLEEP while being in this loop, the GPS gets shut down and the loop
      would continue to wait
    - Corrected writing xNvdsConfig.ulGPSMaxWaitForFix on a server command.
      While the variable is long. the server send a short which needs to be
      multiplied by 10 (FreeRTOS tick is 1ms).
    - Adapted project file for SES 6.34a


### [8.2-20] - 2022-09-27

    Added automated sver FW list update to make_all script.


### [8.2-19] - 2022-09-22

    Changed make_all_releases script to generate only one zip file containing
    all targets.


### [8.2-18] - 2022-09-22

    Chagned LED to DRV_LED.


### [8.2-17] - 2022-09-22

    Changed QTEL version query to get a more complete response.


### [8.2-16] - 2022-09-22

    - Let the TL510 project file point to the (new) TL510 bootloader.
    - Add BL version to startup messages and VERSION command response.
      For this, a bootloader version string search function was added to utils.


### [8.2-15] - 2022-09-20

    Adapted X/Y acceleration axes to accelerometer orientation in the design.


### [8.2-14] - 2022-09-20

    - Fixed self-test.
    - Increased time-out for TLS receive (the Quectel seems to chole from
      time to time).


### [8.2-13] - 2022-09-15

    - Ivertsed LED driver values for TL510.
    - Debugged make_all_releases script.
    - Adapted VBAT and NTC thresholds to TL510.


### [8.2-12] - 2022-09-15

    Support for TL510.


### [8.2-11] - 2022-09-09

    - Added LTE band 3.
    - Added HTTP GET timeout (1s less than the command timeout).
    - Charger: Read value for initial charge current calculation from ADC.
      The value in sBattVoltage (used before this change) might not be
      up-to-date at this point.
    - Shut down GPS in bGetGpsPosition() if GPS is no longer needed.
    - Added missing update of xCtrlStateBackup.
    - Removed check in vNonBlockingAlignBleActivitySuspension() for location
      start. This did not work in the charging case (not suspended but no
      localisation either).


### [8.2-10] - 2022-08-19

    Store ALERT and SOS control states in NVDS and restore on boot.
    This makes an alert/SOS state survive a hardware reset.


### [8.2-9] - 2022-08-12

    - Added test for \@, \r and \n in utility functions converting int string
      to a binary value (usIntStrgToShort( and ulIntStrgToLong()).
      This corrects a bug in correctly getting the precision from the server-
      provided rough location estimation.
    - Made bGoodGPSFix a local variable. All program parts which need to know
      the GPS fix state use now bHasGPSGoodFix().


### [8.2-8] - 2022-08-12

    Use of last GPS position if the position estimation server delivers no
    precise estimate (just the centre of the country).


### [8.2-7] - 2022-08-11

    - Set default position to that of the xsole location server. Although less
      precise, the TTFF is better.
    - Repeat HTTP request to the xsole location server twice.
    - Added missing restore of the backed-up BLE state in
      vResumeAllBleActivity().


### [8.2-6] - 2022-08-05

    Redesigned BLE suspend mechanism. Use of counting semaphores. See 'BLE suspend
    mechanism.txt'
    
    Ported changes from TL320:f04331dda2b72d07fa2278511b4e745159a398ce to
    2ab7bd27ecd7edf27b125fb6f508928616857d65:
    - Added EVAC_ID field to server packet.
    - Cleanup: Moved all evac related management to own module.
    - Corrected issue where the device sent evacuation on BLE if it has received
      evac on BLE before. This led to flooding of the network.
      Now the devie only transmits evac on BLE if it had received an evac command
      from the server before.
      An evacuation ID remains vaild until the device is put on the charger.


### [8.2-5] - 2022-07-07

    - Completed BLE module state kept in xBleModuleState. Moved bSendRelayedCmd
      to this bitfield.
    - The function vBleSetAdvertising() now evaluates xBleModuleState to
      determine if/how to start advertising.
      This is important as the function is periodically called from beacon
      update timer.
    - Moved starting/stopping of the softdevice to a separate function.
    - Removed code for polled temperature function from ble_main (was not
      called anyway).


### [8.2-4] - 2022-07-04

    Added support for immobility beacon from TL234:3a760f6c81585c8dcbd2d5730444217b09e30ffe


### [8.2-3] - 2022-06-30

    Changed VGSM_OFF_THRES to 1.5V instead of 300mV.


### [8.2-2] - 2022-06-16

    - Pulled twim.c into the TL software to be able to change the I2C drive
      strength setting to high.
    - Corrected vShortToIntStrg() which was suppressing '0' in the middle of a
      number. Problem was already corrected in other projects.


### [8.2-1] - 2022-06-07

    Added support for BLE_STD_ADV_INT command (see also TL320:c23303743f2ddc37f27eb6c84b2f321d4384a63e).
    Corrected cGsm_MNC parsing, see TL320:c230b0b54c54aab97ed770682fec9c771976af.


### [8.2-0] - 2022-05-19

    Removed useless GPS start on inactive transition. GPS is started on request
    from GSM process.


### [8.1-20] - 2022-05-19

    Fixed TCP reopen procedure after an unsolicited close.


### [8.1-19] - 2022-05-19

    Added option to activate BLE only a short while before attempting a GPS
    fix. The duration for activating BLE before GPS is configurable as well.


### [8.1-18] - 2022-05-19

    Added support for GOTO_PREALERT command also to ALERT and SOS states.
    Return an error message to the server if the device is in a state which
    does not support the command.
    
    Fixed bad transition to charging with position recording enabled.


### [8.1-17] - 2022-05-13

    Corrected syntax of GPS_POSREC command.


### [8.1-16] - 2022-05-13

    - Added GOTO_PREALERT command.
    - Added configuration for the red LED GPIO as fall-back (nRF UICR
    configuration).
    - Put configuration for BLE_MIN_BCN back to default (5 beacons).


### [8.1-15] - 2022-05-13

    Added SRVCMD_WR_GPS_POSREC also to remote BLE command list.


### [8.1-14] - 2022-05-13

    Revert "Corrected bad string termination for configuration strings. This was"
    
    This reverts commit 8e64bf4e837a3375e1dfbc33165a2474cc9e4f2c.


### [8.1-13] - 2022-05-13

    Corrected bad string termination for configuration strings. This was
    already fixed in the TL320.


### [8.1-12] - 2022-05-13

    - Added command to configure position recording.
    - Added missing command execution for location server configuration.


### [8.1-11] - 2022-05-13

    ATTENTION: Debug configuration (pos. recording, BLE positioning)!
    
    - Added position recording debug mode where position are recorded as well
      as sent to the server. This allows for debugging any issues, particularly
      with the step count.
    
    - Added battery last gasp status during which the position recording mode
      is overridden and packets are pushed to the server so that the user
      can be  notified that the battery is almost empty.
    
    - Added time stamp to NRF_LOG messages and aligned their urgency.


### [8.1-10] - 2022-05-12

    - First version with position recording working.
    - Bug fix: Disable negative values in uiReadADC(). Negative values are
               the result of small input voltages overlaid by noise.
    - Bug fix: Corrected temperature reporting in BLE distress beacons.
    - Simplification: charger.c (no functional change).
                      Defined CHRG_STATE_END_OF_CHARGE_MIN for cl;arification.


### [8.1-9] - 2022-05-02

    Merge branch 'TL500' into GPS_recording


### [8.1-8] - 2022-05-02

    Erase config record always when coming out of reset with the BL_FW_UPDATE
    or ROLLED_BACK flags set.


### [8.1-7] - 2022-05-02

    Fixes to allow compiling.


### [8.1-6] - 2022-04-29

    Merge branch 'TL500' into GPS_recording


### [8.1-5] - 2022-04-29

    Finally, decided to go back to commit 3cff3ee253ebd8d6fd for the handling
    of the failed boot counter. Here, BAD_FW_RESET_CNT (=30) resets due to FW
    malfunctioning or misconfiguration will let the FW roll back.
    
    The reason is that not the entire module resets when in a zone of
    bad reception, only the cellular module is reset (this was erroneous
    reasoning in commit 6d39457400b600ab8). Hence, the failed boot counter
    does not increase and there is no risk of FW roll-back.
    
    The only circumstance where a useless roll back could happen is when the
    module dies because of an empty battery and the reset count increases on
    the following recharge while there is not a single server communication
    for 30 of these cycles.


### [8.1-4] - 2022-04-28

    Increased failed boot count limit before rolling back FW since now the
    FW is rolled back if the count limit is reached for any system reset
    cause without being able to contact the server.
    
    System errors caused by malconfiguration will no longer result in FW
    roll-back. This is to avoid roll-backs in case the server could not be
    contacted due to eing in an area with no coverage.
    
    Clarified message sent to server when the FW has been rolled back.


### [8.1-3] - 2022-04-28

    Fixed bug where a corrupted configuration record could leave the system
    stuck on boot.
    
    1. Added dedicated counter for failed boot attempts. The counter is
       increased right after system reset and set to 0 only once the first
       packet to the server is acknowledged.
       If the counter exceeds a limit, the configuration record is erased,
       the failed boot counter reset and the FW is rolled back.
    
       This approach also protects the system against a configuration causing
       catastrophic system malfunction.
    
    2. Mitigated the effect of writing configurations. Queueing many requests
       at once (like in sequentially writing a string) overloads the FDS.
       Now, strings are other records (BLE keys) are buffered and written at
       once.
    
    3. Explicitly invalidate the configuration record atfer loading a new FW
       and prior to launching the bootloader.
    
    Changed the UART driver to allow it to run without the OS operational.
    This is required for pushing information in case of a system error
    before booting the OS.
    
    Corrected FSTORAGE_PAGE_START which is used when the SD is not enabled.
    It has always to point to the start of the bootloader. The starts of
    configuration records held in FDS and the system error record are
    calculatred from there.


### [8.1-2] - 2022-04-25

    Initial version for GPS recording.
    **** NOT WORKING YET *****


### [8.1-1] - 2022-04-07

    Combined updates from TL320:6250427ed2 to c3fb9ce82c
    
    - Test introducing a min. z acceleration limit for detecting SOS. This is
      intended to avoid alse detections when descending a staircase.
    - Forgotten files..
    - Changed size of SOS_Z_ACC_LO_HIST_WIN and SOS_Z_ACC_LO_THRES to 0.6g
      for easier SOS trigger.
    - Added additional trace saved to trace file.
    - Added parameter which cuts GPS satellite search short if there are less
      than 2 satellites received after a certain time. That time-out is
      configurable.
    - Added the forgotten parameter update via BLE.


### [8.1-0] - 2022-03-29

    Removed accelerometer VCC code which has no effect on TL500.


### [8.0-32] - 2022-03-04

    Corrected bug in PW extraction (last character clipped).


### [8.0-31] - 2022-03-04

    - Adapted FTP and FWUPD commands to support SoftDevice and bootloader
      update.
    - Reverted BL-hex project name back to TL500 due to incompatibility with
      BL setting eneration script.
    -


### [8.0-30] - 2022-03-03

    - Corrected GPS register read.
    - At GPS start, verification of current GNSS settings avoid to superflouous
      module resets and navigation database destruction when configuring GNSS.
    - Corrected FSTORAGE_PAGE_START address for builds without bootloader.


### [8.0-29] - 2022-02-28

    Corrected GPS GNSS configuration - but not tested.


### [8.0-28] - 2022-02-25

    - Added UBX_CFG_CFG command to GPS init sequence.
    - Added size check to FTP get (must be >0).


### [8.0-27] - 2022-02-25

    Shut down of the trace UART in SLEEP.
    Added an option to also shut down the SoftDevice when not ise use.
    This option, however, does not seem to be very useful as it saves no
    power and gives problems at subsequent startups
    (NRF_ERROR_SVC_HANDLER_MISSING).


### [8.0-26] - 2022-02-24

    Cleaned UARTE disable and GSM_TXD release.


### [8.0-25] - 2022-02-24

    - Added missing ASSERT definition to the RELEASE configuration.
    - Reduced wait time in SPI accelerometer read. Typically, that transfer is
      very fast so waiting for 50ms was useless.
    - Fixed string length bug in prvAddLocReqLen().
    - Replaced nrf_drv_clock.c with bug-fixed version (LFCLK was not started
      if WDT was detected running).


### [8.0-24] - 2022-02-21

    - Implemented workaround for pulling GSM_TXD low: use n.c. pin as TX pin.
    - Implemented workaround (safety) for the SoftDevice leaving Idle too
      early. Might not be necessary, though.
    - Allowed for negative voltages measured with VCHRG_ADC.
    - Modified DEBUG output pins to final module.


### [8.0-23] - 2022-02-17

    Removed support for prototype board.


### [8.0-22] - 2022-02-15

    Minor edit.


### [8.0-21] - 2022-02-15

    In the app_error_fault_handler(), extraction of the PC for the release
    builds.


### [8.0-20] - 2022-02-15

    If there was no fix, clobber all GPS data.


### [8.0-19] - 2022-02-14

    - Rework of reset logging system.
    - Removed remaining ping.
    - TEMPORARY: log is enabled by default.


### [8.0-18] - 2022-02-08

    Added missing check for already existing TCP/TLS connection before
    closing an inactive TCP socket.


### [8.0-17] - 2022-02-08

    - Added emergency error record storage to Flash in case of a system error.
    - Corrected reset counters (alignment with HW spec).
    - Request starting of BLE in timer only if BLE is really off.


### [8.0-16] - 2022-02-07

    - Removed trace data from FDS and put it into dedicated Flash instead as
      part of the system error record.
    - Stop all BLE activity while an FDS update is going on.
    - Moved maintenance of system reset count to the trace module.
    - Corrected bug in stats reporting to server.


### [8.0-15] - 2022-02-03

    Attemptred to make the FDS usage more stable in presence of high BLE
    load. The code still contains some debug code.


### [8.0-14] - 2022-02-01

    Increased number of reported location beacons to 12.


### [8.0-13] - 2022-01-31

    - Corrected FDS page size to 4096bytes (1024 long words).
    - Removed critical error reset from GPS driver on checksum errors.
    - Corrected issue with answering to commands received while hunting for
      a GPS fix before entering the actual packet send part.
      The GSM was erroneously taken to be off and thus incoming TCP indications
      were not handled.


### [8.0-12] - 2022-01-27

    Fixed incorrect beacon address and beacon use extractions.


### [8.0-11] - 2022-01-27

    Implemented reset counters in system error record in Flash.
    Moved app_error_fault_handler() to main.
    Added memory layout diagrams to tracker.h.


### [8.0-10] - 2022-01-26

    Added watchdog.


### [8.0-9] - 2022-01-21

    Add version information fields for softdevice and chip revision.


### [8.0-8] - 2022-01-21

    - Corrected interface with bootloader.
    - Better recovery from unsolicited TLS socket closures.
    - Added informational note about physical location of FDS pages.
    - Updated pointer to new ARM toolchain in make_all_releases script.
    - Changed SES projectfile to allow simultaneous source-code debugging
      of bootloader and application.


### [8.0-7] - 2022-01-18

    Corrected issue with handling accelerometer interrupts occurring while
    access to the accelerometer was locked.


### [8.0-6] - 2022-01-17

    - Removed NRFX driver enable from SDK settings. The NRFX driver is
      implicitly used since easyDMA is configured for the legacy driver.
      This will map all legacy calls to the new NRFX drivers.
    - Added missing EncryptStop to ParseDistressBeacon. Before, the AES was
      allocaed but never freed if a LR heartbeat beacon was received.
    - Added generating a BL settings file with a standard build from SES.
      The file is loaded along with the TL bootloader when debugging with
      SES.


### [8.0-5] - 2022-01-14

    Updated interface with bootloader.


### [8.0-4] - 2022-01-13

    - Made +CCID command response longer as the returned CCID is written
      to NVDS (which may take up to ~10s if garbage collection os required).
    - Added fail-safe mechanisms to FDS (drv_nvm).


### [8.0-3] - 2022-01-10

    Added configuration selection option to release script.


### [8.0-2] - 2022-01-07

    updated release script


### [8.0-1] - 2022-01-07

    Updated make_all_releases.py


### [8.0-0] - 2022-01-07

    Renamed project to TL500.


### [2.0-25] - 2022-01-04

    Cleaned release version. Introduced a new target for debug w/o
    softdevice.


### [2.0-24] - 2022-01-04

    Removed (some) debug outputs.


### [2.0-23] - 2022-01-04

    - New debug outputs DEBUG6/7.
    - Corrected bug in accelerometer interrupt handling:
      + The IRQ handler called vTaskDelay which - hwne interrupting the idle
        task - put the idle task onto the delayed task list which left no ready
        task. The scheduler used an arbitrary TCB with the ready list empty.
      + Made sure the interrupt handler was not called in vAccReleaseAccess()
        with INT1_ACC interrupt disabled. This happened e.g. in the wake-up
        interrupt hold-off period (state Inactive).


### [2.0-22] - 2021-12-29

    - Fixed a couple of issues with the accelerometer driver:
      + INT1 handler executes later than INT2 in case both interrupts fired.
      + vAccResumeDelayedTimerSvc() is moved out of the critical section in
        vAccReleaseAccess() as this cased a dead-lock (waiting for SPI interrupts
        with interrupts being disabled).
      + Added a 10ms delay in the loop in vAccReleaseAccess() in between calls
        to vAccIrqHandler() as the accelerometer takes ~2.5ms to reset its INT
        pin after acknowledgement.
    
    - Added total number of visible satellites to vPushGpsFixSynopsis().
    
    
    NOTE: There are still a couple of debug probes in the code.


### [2.0-21] - 2021-12-28

    - Added charge current control using PWM output.
    - Added external interrupt for accelerometer.
    - Increased timer stack size as overflows occurred.
    - Enabled usage of the VDDH ADC as the prototype board HW is now changed.
    - Fixed bug in GPS fix quality assessment where the same horiz. prec.
      value was used twice to check for improvements.
    - Adapted GSM configurable timer values to 1ms systick. For compatibility
      reasons, the value configured by command from the server is in units
      of 10ms.
    - Adjusted voltage thresholds.


### [2.0-20] - 2021-12-07

    - Corrected bug (wrong queue used) in GSM task (vGSMModulePowerDown()).
    - Added UARTE memory manager debug prints.
    - Added compile switch allowing to build a target without softdevice.
      Much better debug support.
    - Raised default log print level and allowed individual module to lower
      log.


### [2.0-19] - 2021-12-06

    - Simplified position transmission timer calculation in gsm.c as TickType_t
      is now 32bit instead of 16bit wide.
    - Corrected bug in cGsmACCH transfer.
    - Renmoved useless xGsmTcpRxMsgCnt variable (not set to received character
      count for QTEL as opposed to SARA).
    - Added safety check for block to free in drv_uart interryupt routine.
      It seems, however, that a block not marked as used and consequently not
      freed will eventually fragment the heap.


### [2.0-18] - 2021-12-06

    - Corrected UART TX buffer allocation scheme.
    - Added initialisation to vRemoveTypeFromQueue() as the message type is
      now (potentially) shorter than unsigned portBASE_TYPE.
      enums are 8-bit whereas portBASE_TYPE is 32-bit.


### [2.0-17] - 2021-12-03

    Formatted GPS fix synopsis as regular trace message.


### [2.0-16] - 2021-11-30

    - Changed UART TX memory allocation scheme to FreeRTOS heap_4
    - Corrected AES bug (incomplee init, swapped parameters).
    - Adapted battery voltage conversion factors.
    - Corrected bug in BLE_AD (missing timer creation).
    - BLE now operational.


### [2.0-15] - 2021-11-29

    - Corrected issues with AN file reception and storage.
    - Corrected bug in UART: COM0 TX queue was used for all interfaces.
    - Added debug task number trace via GPIO outputs.


### [2.0-14] - 2021-11-26

    Fixed NVDS issue.


### [2.0-13] - 2021-11-22

    First fully integrated version. Compiles but does not yet run.


### [2.0-12] - 2021-11-15

    Added parsing of advertiser data (originally in ble_parser.c).


### [2.0-11] - 2021-11-09

    - Implemented NVDS configuration handling
    - First version of trace
    - Added most missing modules.
    Not yet compiling!


### [2.0-10] - 2021-10-28

    Changed project structure.
    Got libuarte running under FreeRTOS.


### [2.0-9] - 2021-06-24

    Implemented release and setting low of SDA/SCL when GPS is off.


### [2.0-8] - 2021-06-17

    Release script fixing.


### [2.0-7] - 2021-06-17

    Changed project name once more.
    Created Python script to automate release.


### [2.0-6] - 2021-06-15

    Removed debug print to serial if the TX buffer is full.


### [2.0-5] - 2021-06-14

    Corrected response to AT+BTADDR? query.


### [2.0-4] - 2021-06-14

    - Support for System OFF state used vShutDown().
    - Purged ublox references from AT commands.


### [2.0-3] - 2021-06-11

    Added DCDC0 to power configuration.


### [2.0-2] - 2021-06-11

    Renamed project from NINA_B3 to nRF52840. No changes in contents.


### [2.0-1] - 2021-06-11

    - Adapted board definition for TL400.
      Temporarily disabled DCDC.
    - Made reset work (pulled nrf52.* instead of nrf52840.* files which had
      the wrong reset pin definition).
    - Removed NINA-B1 references.


### [2.0-0] - 2021-05-10

    Split GPS position report into several individual message to lower
    message character count.


### [1.1-18] - 2021-05-06

    Moved UBX and NMEA parsing from main controller to nRF. Send only a GPS
    fix quality report to the main controller. The GPS position report is
    available on command. Added a configuration which allows seeing the raw
    NMEA and UBX reports on the UART interface.


### [1.1-17] - 2021-05-03

    Flush the GPS receiver after sending the AssistNow data. The data
    accumulated during the update process is not useful and overloads the
    main processor.


### [1.1-16] - 2021-04-30

    Now streaming AssistNow data directly from UART to I2C while verifying
    the data at the same time. This gives the GPS receiver the time it needs
    in between UBX messages.
    Debug data and code is included in comments.
    Added more complete handshake with main controller.


### [1.1-15] - 2021-04-29

    Corrected message buffer overflow as UBX messages were never used/trashed
    in GPS process.
    Added buffer supervision in GPS driver.
    Interpret parameter in "AT+GPSANFILE as decimal as opposed to HEX value.


### [1.1-14] - 2021-04-15

    Added AssistNow support. Sample dataset included.


### [1.1-13] - 2021-04-08

    - Added OK response to GPS commands.
    - Still problems with the UART blocking. Attempt to define blocktime
      as 0.


### [1.1-12] - 2021-03-25

    Implemented GPS AT commands and removed debug code.


### [1.1-11] - 2021-03-24

    Completed GPS initialisation.


### [1.1-10] - 2021-03-24

    Cleaned return code handling (at bit. Still not consistent).
    Better wait loop in reading register.
    Work on write register.


### [1.1-9] - 2021-03-19

    First version with drv_gps almost complete.
    
    This test version polls the MON-VER register every 100ms, splits the
    data read back from ZOE into UBX and NMEA messages and verifies the
    checksum for each.
    
    The processor stops/reboots whenever there is any protocol error.
    
    This version runs the long-duration reliability test to verify that ZOE
    can handle SCK pulses shortened due to the nRF HW bug.


### [1.1-8] - 2021-03-16

    First version for nRF code for TL400.
    The chip serves as a handler for the GPS.


### [1.1-7] - 2021-01-12

    TL as custom board now in config directory


### [1.1-6] - 2020-11-03

    - Enabled DC/DC.
    - Corrected bug in filtering (string vs. binary comparison).
    - Filtering iBeacons for TRAXxs-specific signature.
    - Added configuration for seeing registers in debugger.


### [1.1-5] - 2020-09-07

    Narrowed filter for iBeacon reports.


### [1.1-4] - 2020-09-04

    Added iBeacon support.


### [1.1-3] - 2020-08-18

    Removed debug code.
    Removed autoatically generated 'version.c' file from being tracked by git.
    (This created a cyclic dependency resulting in version.c never matching
    the git SHA1).
    
    Signed-off-by: Norbert Grunert <ngrunert@traxxs.fr>


### [1.1-2] - 2020-08-18

    Changed averaging to average over linear values instead of logarithmic
    ones (rap knuckels !!!).


### [1.1-1] - 2020-08-17

    Commented debugging code.


### [1.1-0] - 2020-08-17

    Added configurable RSSI filtering in the ble_adreport module.


### [1.0-13] - 2019-12-06

    Corrected ASCII field length error when sending long advertiser indications
    to host. ASCII representation of +UBTD might be longer than 550 chars.


### [1.0-12] - 2019-12-06

    Corrected UART buffer problem when receiving big commands from host:
    - Doubled SERIAL_FIFO_RX_SIZE because its holds the 255-byte payload in
      ASCII, not in binary.
    - Dito for uartRX_BUFFER_SIZE.
    - Increased max message length when copying data in vSetLR125kbpsAdvData to
      2*256 for the same reason.
    - Changed ucIdx in cGetRxCharFromBufferWithIndex from char to portBASE_TYPE
      allowing it to become larger than 255 on ARM.


### [1.0-11] - 2019-10-29

    - Added BLE OS timeout.
    - Changed scan windows/intervals to avoid hiding BLE advertisers with 1s
      advertising interval (an advertiser interval of 1s is an integer
      multiple of a 320=200ms scan interval).


### [1.0-10] - 2019-03-04

    Added xMutexBleConfig semaphore to BLE parser.
    Added payload length check also to 1Mbps.


### [1.0-9] - 2019-02-27

    - Added missing BE configuration protection to vSetLR125kbpsAdvData().
      Re-arranged payload length consistency check.
    - Added 5 seconds filter to BLE advertising reports to remove pressure
      from the main ATXMega processor.


### [1.0-8] - 2019-02-26

    - Reduced BLE/BPR stack sizes to something more reasonable.
    - Increased SDH task stack size to 512 bytes.
    - Catch NRF_ERROR_BUSY for nrf_serial_write() as this only means that the
      mutex could not be taken. In this case, just retry.
    - Added mutext for BLE configuration to make sure that not both BLE and BPR
      try to change the configuration at the same time.
    - Added APP_ERROR_CHECK_ATIF() macro which pushes a string to UART before
      dying. Easier to debug.
    - Advertisement payload length check for consistency in BPR.


### [1.0-7] - 2019-02-18

    Added command to set the advertisement interval in function of the PHY
    mode.


### [1.0-6] - 2019-02-08

    Increased BPR / BLE stack size.
    Increased heap size (way too much).
    Corrected bug with inconsistent RX UART buffer sizes.


### [1.0-5] - 2019-02-06

    To accomodate larger extended payloads, increased:
    - UART FIFO sizes
    - cAdvReportStrg, cEncodedLR125kbpsAdvData and cLocalAdvData
    
    Added command to set the TX output power for LR and SR separately.


### [1.0-4] - 2019-02-01

    Set TX outut power to 6dBm


### [1.0-3] - 2019-01-29

    - Advertiser filtering on payload contents only (payload legnth indication,
      manufacturer ID and proprietary format).
    - Added correct parsing of address type (public, random static, etc.)


### [1.0-2] - 2019-01-08

    Corrected version string generation (Bash script was not called).
    Project file pointed to bad debug register definition file.


### [1.0-1] - 2019-01-07

    Added support for different transmitted LR and SR advertising packets.


### [1.0-0] - 2018-12-11

    Added REL/DBG to version information


### [0.0-3] - 2018-12-05

    First version NINA-B3 with custom application.


### [0.0-2] - 2018-11-27

    NINA software ported from B1 to B3. No change in functionality.


### [0.0-1] - 2018-10-31

    NINA appliucation FW initial version. Based on version with the same name
    in 'Nordic openCPU trials' directory


