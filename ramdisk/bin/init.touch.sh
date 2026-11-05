#!/vendor/bin/sh
#
# Find the input devices that init.power.rc inhibits while the display is
# not interactive, and publish their names (inputN): the stmfts
# touchscreen in vendor.touch.stmfts.input and the capacitive Recents and
# Back keys in vendor.touch.touchkey.input. Input numbers depend on probe
# order, so init.power.rc writes their inhibited attributes through these
# properties instead of fixed paths.

found=1

for dev in /sys/devices/platform/soc@0/14e10000.i2c/i2c-*/*-0049/input/input*; do
    [ -r "$dev/name" ] || continue
    read -r name < "$dev/name"
    if [ "$name" = "stmfts" ]; then
        setprop vendor.touch.stmfts.input "${dev##*/}"
        found=0
        break
    fi
done

for dev in /sys/devices/platform/i2c-touchkey/i2c-*/*-0020/input/input*; do
    [ -r "$dev/name" ] || continue
    read -r name < "$dev/name"
    if [ "$name" = "sec_touchkey" ]; then
        setprop vendor.touch.touchkey.input "${dev##*/}"
        found=0
        break
    fi
done

exit $found
