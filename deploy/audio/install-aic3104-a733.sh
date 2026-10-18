#!/bin/sh
set -eu

KERNEL_VERSION="$(uname -r)"
EXPECTED_KERNEL="6.6.98-sun60iw2"
SOURCE_DIR="/usr/src/linux-sun60iw2-6.6.98"
OVERLAY_NAME="microduck-aic3104"
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

die() {
	echo "error: $*" >&2
	exit 1
}

[ "$(id -u)" -eq 0 ] || die "run this installer with sudo"
[ -r /etc/orangepi-release ] || die "/etc/orangepi-release not found"
grep -qx 'BOARD=orangepizero3w' /etc/orangepi-release || \
	die "this installer only supports Orange Pi Zero 3W"
grep -qx 'BOARDFAMILY=sun60iw2' /etc/orangepi-release || \
	die "this installer only supports the A733/sun60iw2 board family"
[ "$KERNEL_VERSION" = "$EXPECTED_KERNEL" ] || \
	die "expected $EXPECTED_KERNEL, running $KERNEL_VERSION"
[ "$#" -eq 1 ] || die "usage: sudo $0 /path/to/linux-sun60iw2-6.6.98.tar.gz"

SOURCE_ARCHIVE=$1
[ -f "$SOURCE_ARCHIVE" ] || die "kernel source archive not found: $SOURCE_ARCHIVE"

for command in tar make gcc dtc depmod modprobe systemctl; do
	command -v "$command" >/dev/null 2>&1 || die "missing command: $command"
done

if [ ! -f "$SOURCE_DIR/Makefile" ]; then
	echo "Extracting the local kernel snapshot to $SOURCE_DIR"
	tar -xzf "$SOURCE_ARCHIVE" -C /usr/src
fi
[ -f "$SOURCE_DIR/Makefile" ] || die "archive did not create $SOURCE_DIR"

echo "Preparing the A733 kernel tree"
cp "/boot/config-$KERNEL_VERSION" "$SOURCE_DIR/.config"
cd "$SOURCE_DIR"
scripts/config --set-str LOCALVERSION "-sun60iw2"
scripts/config --disable LOCALVERSION_AUTO
scripts/config --module SND_SOC_TLV320AIC3X_I2C
make olddefconfig
make modules_prepare

BUILT_RELEASE=$(make -s kernelrelease)
[ "$BUILT_RELEASE" = "$KERNEL_VERSION" ] || \
	die "prepared tree has release $BUILT_RELEASE, expected $KERNEL_VERSION"

echo "Building the TLV320AIC3X core and I2C modules"
make -j"$(getconf _NPROCESSORS_ONLN)" \
	KBUILD_MODPOST_WARN=1 M=sound/soc/codecs modules

CORE_MODULE="sound/soc/codecs/snd-soc-tlv320aic3x.ko"
I2C_MODULE="sound/soc/codecs/snd-soc-tlv320aic3x-i2c.ko"
[ -f "$CORE_MODULE" ] || die "codec core module was not built"
[ -f "$I2C_MODULE" ] || die "codec I2C module was not built"

MODULE_DIR="/lib/modules/$KERNEL_VERSION/extra"
install -d -m 0755 "$MODULE_DIR"
install -m 0644 "$CORE_MODULE" "$MODULE_DIR/"
install -m 0644 "$I2C_MODULE" "$MODULE_DIR/"
depmod -a "$KERNEL_VERSION"

OVERLAY_DIR="/boot/dtb-$KERNEL_VERSION/allwinner/overlay"
[ -d "$OVERLAY_DIR" ] || die "overlay directory not found: $OVERLAY_DIR"
DTBO=$(mktemp)
trap 'rm -f "$DTBO"' EXIT HUP INT TERM
dtc -@ -I dts -O dtb -o "$DTBO" "$SCRIPT_DIR/aic3104-a733.dts"
install -m 0644 "$DTBO" "$OVERLAY_DIR/sun60i-a733-$OVERLAY_NAME.dtbo"

ENV_FILE=/boot/orangepiEnv.txt
[ -f "$ENV_FILE" ] || die "$ENV_FILE not found"
cp -a "$ENV_FILE" "$ENV_FILE.pre-aic3104-$(date +%Y%m%d-%H%M%S)"

if grep -q '^overlays=' "$ENV_FILE"; then
	OVERLAYS=$(sed -n 's/^overlays=//p' "$ENV_FILE" | tail -n 1)
	case " $OVERLAYS " in
		*" $OVERLAY_NAME "*) ;;
		*) sed -i "s/^overlays=.*/overlays=$OVERLAYS $OVERLAY_NAME/" "$ENV_FILE" ;;
	esac
else
	printf '\noverlays=%s\n' "$OVERLAY_NAME" >> "$ENV_FILE"
fi

install -m 0755 "$SCRIPT_DIR/aic3104-init.sh" /usr/local/bin/aic3104-init.sh
# A checkout prepared on Windows may carry CRLF even though the target is
# Linux. Strip the carriage return so the shebang remains executable.
sed -i 's/\r$//' /usr/local/bin/aic3104-init.sh
install -m 0644 "$SCRIPT_DIR/aic3104-a733.service" \
	/etc/systemd/system/aic3104-a733.service
systemctl daemon-reload
systemctl enable aic3104-a733.service

# Loading the modules is safe without the HAT. The codec will only bind after
# the overlay exists and an AIC3104 acknowledges address 0x18.
modprobe snd-soc-tlv320aic3x
modprobe snd-soc-tlv320aic3x-i2c

echo "Installed AIC3104 modules and $OVERLAY_NAME overlay."
echo "Reboot, then check: aplay -l; i2cdetect -y 2; dmesg | grep -Ei 'aic3|sound-mach'"
