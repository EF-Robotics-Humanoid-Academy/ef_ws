#!/usr/bin/env bash

# Fully automated Jetson Wi-Fi setup.
# Prompts only for SSID and password, then writes wpa_supplicant config,
# starts the service, and asks DHCP for an address.

set -Eeuo pipefail
umask 077

readonly SCRIPT_NAME="$(basename "$0")"
readonly INTERFACE="wlan0"
readonly CONFIG_DIR="/etc/wpa_supplicant"
readonly CONFIG_FILE="$CONFIG_DIR/wpa_supplicant-${INTERFACE}.conf"
readonly SERVICE="wpa_supplicant@${INTERFACE}.service"

TEMP_FILE=''
PASSWORD=''

die() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

warn() {
    printf 'Warning: %s\n' "$*" >&2
}

cleanup() {
    if [[ -n "$TEMP_FILE" && -e "$TEMP_FILE" ]]; then
        rm -f -- "$TEMP_FILE"
    fi
    unset PASSWORD
}
trap cleanup EXIT

[[ $EUID -eq 0 ]] || die "run this script with sudo: sudo ./$SCRIPT_NAME"
[[ -d "/sys/class/net/$INTERFACE" ]] || die "network interface does not exist: $INTERFACE"
command -v wpa_passphrase >/dev/null 2>&1 || die "wpa_passphrase is not installed"
command -v systemctl >/dev/null 2>&1 || die "systemctl is not available"
command -v ip >/dev/null 2>&1 || die "ip is not installed"

# Verify the service before replacing a known-good configuration. This keeps
# a typo or incomplete image from leaving wlan0 with a config that nothing
# will consume.
systemctl cat "$SERVICE" >/dev/null 2>&1 || die "$SERVICE is not installed on this Jetson"

read -r -p 'Wi-Fi SSID: ' SSID
[[ -n "$SSID" ]] || die "SSID cannot be empty"
[[ "$SSID" != *$'\n'* && "$SSID" != *$'\r'* ]] || die "SSID cannot contain newlines"

read -r -s -p 'Wi-Fi password: ' PASSWORD
printf '\n'
[[ -n "$PASSWORD" ]] || die "password cannot be empty"

install -d -m 0755 "$CONFIG_DIR"
TEMP_FILE="$(mktemp "$CONFIG_FILE.tmp.XXXXXX")"
chmod 600 "$TEMP_FILE"

if ! wpa_passphrase "$SSID" <<<"$PASSWORD" |
    sed '/^[[:space:]]*psk="/d' >"$TEMP_FILE"; then
    die "could not generate the wpa_supplicant configuration"
fi

unset PASSWORD

if [[ -f "$CONFIG_FILE" ]]; then
    BACKUP_FILE="${CONFIG_FILE}.backup.$(date +%Y%m%d-%H%M%S)"
    cp -a -- "$CONFIG_FILE" "$BACKUP_FILE"
    printf 'Backed up existing configuration to %s\n' "$BACKUP_FILE"
fi

mv -- "$TEMP_FILE" "$CONFIG_FILE"
TEMP_FILE=''
chmod 600 "$CONFIG_FILE"

if command -v rfkill >/dev/null 2>&1; then
    rfkill unblock wifi || warn "could not unblock Wi-Fi with rfkill"
fi

ip link set "$INTERFACE" up
systemctl enable "$SERVICE" >/dev/null
systemctl restart "$SERVICE"

if command -v dhclient >/dev/null 2>&1; then
    dhclient -r "$INTERFACE" >/dev/null 2>&1 || true
    dhclient "$INTERFACE" || warn "dhclient could not obtain an address for $INTERFACE"
elif command -v networkctl >/dev/null 2>&1; then
    networkctl renew "$INTERFACE" || warn "networkctl could not renew $INTERFACE"
else
    warn "no DHCP client found; check the connection manually with: ip addr show $INTERFACE"
fi

printf 'Wi-Fi configuration installed: %s\n' "$CONFIG_FILE"
if systemctl is-active --quiet "$SERVICE"; then
    printf '%s is active.\n' "$SERVICE"
else
    warn "$SERVICE is not active; inspect it with: journalctl -u $SERVICE -n 50 --no-pager"
fi

if command -v iw >/dev/null 2>&1; then
    iw dev "$INTERFACE" link || true
fi

ip -4 addr show "$INTERFACE" || true
