# Hardware the NK 6.6 kernel does not drive yet. Each switch keeps its
# HALs, feature files and VINTF entries out of the build, so the framework
# does not wait on or restart a HAL whose kernel driver is missing. Set a
# switch to true once the kernel supports that block.
NK_HAS_MODEM ?= false
NK_HAS_CAMERA ?= false
NK_HAS_NFC ?= false
# GPS (lhd, gpsd and the GNSS HAL) sits behind the BCM4773 sensor hub on
# SPI1, which needs Samsung's BBD driver stack.
NK_HAS_GPS ?= false
# The Lineage touch HAL (glove, stylus, key disabler) drives Samsung's
# sec/tsp and sec/sec_touchkey sysfs nodes, which the mainline stmfts and
# touchkey drivers do not provide. Its VINTF entry must go with it:
# system_server blocks in getService() on a declared IStylusMode that never
# registers.
NK_HAS_SEC_TOUCH ?= false
# Vibrator: the 6.6 kernel drives the motor as a force-feedback input
# device (max77693-haptic), so it uses the input-FF HAL in
# hardware/vibrator, not Samsung's timed_output one.
NK_HAS_VIBRATOR ?= true
