# Hardware the NK 6.6 kernel does not drive yet. Each switch keeps its
# HALs, feature files and VINTF entries out of the build, so the framework
# does not wait on or restart a HAL whose kernel driver is missing. Set a
# switch to true once the kernel supports that block.
NK_HAS_MODEM ?= false
NK_HAS_CAMERA ?= false
NK_HAS_NFC ?= false
