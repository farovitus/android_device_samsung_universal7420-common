#!/vendor/bin/sh
#
# Find the stmfts touchscreen's input device and publish its name
# (inputN) in vendor.touch.stmfts.input. The input number depends on
# probe order, so init.power.rc writes its inhibited attribute through
# this property instead of a fixed path.

for dev in /sys/devices/platform/soc@0/14e10000.i2c/i2c-*/*-0049/input/input*; do
    [ -r "$dev/name" ] || continue
    read -r name < "$dev/name"
    if [ "$name" = "stmfts" ]; then
        setprop vendor.touch.stmfts.input "${dev##*/}"
        exit 0
    fi
done
exit 1
