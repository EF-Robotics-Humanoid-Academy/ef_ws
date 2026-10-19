#!/usr/bin/env bash
set -Eeuo pipefail
umask 077

# Configure wlan0 with a derived WPA-PSK. Prefer the interactive password
# prompt so the secret does not appear in the shell history or process list.
# Usage: sudo ./setup_wpa_supplicant_wlan0.sh SSID [PASSPHRASE]

die() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

[[ $EUID -eq 0 ]] || die "run this script with sudo"
(( $# >= 1 && $# <= 2 )) || die "usage: sudo $0 SSID [PASSPHRASE]"
command -v wpa_passphrase >/dev/null 2>&1 || die "wpa_passphrase is not installed"
command -v systemctl >/dev/null 2>&1 || die "systemctl is not available"
systemctl cat wpa_supplicant@wlan0.service >/dev/null 2>&1 || die "wpa_supplicant@wlan0.service is not installed"
systemctl cat dhclient@wlan0.service >/dev/null 2>&1 || die "dhclient@wlan0.service is not installed"

SSID="$1"
COUNTRY="${WPA_COUNTRY:-DE}"
CONF="/etc/wpa_supplicant/wpa_supplicant-wlan0.conf"
CONFIG_DIR="${CONF%/*}"
TEMP_FILE=''
PASSPHRASE="${2:-}"

[[ -n "$SSID" ]] || die "SSID cannot be empty"
[[ "$SSID" != *$'\n'* && "$SSID" != *$'\r'* ]] || die "SSID cannot contain newlines"
[[ "$COUNTRY" =~ ^[A-Za-z]{2}$ ]] || die "WPA_COUNTRY must be a two-letter country code"

if [[ -z "$PASSPHRASE" ]]; then
  read -r -s -p 'Wi-Fi password: ' PASSPHRASE
  printf '\n'
fi
[[ ${#PASSPHRASE} -ge 8 && ${#PASSPHRASE} -le 63 ]] || die "password must be 8–63 characters"
[[ "$PASSPHRASE" != *$'\n'* && "$PASSPHRASE" != *$'\r'* ]] || die "password cannot contain newlines"

cleanup() {
  if [[ -n "$TEMP_FILE" && -e "$TEMP_FILE" ]]; then
    rm -f -- "$TEMP_FILE"
  fi
  unset PASSPHRASE
}
trap cleanup EXIT

install -d -m 0755 "$CONFIG_DIR"
TEMP_FILE="$(mktemp "$CONF.tmp.XXXXXX")"
chmod 600 "$TEMP_FILE"

{
  printf 'country=%s\nctrl_interface=DIR=/run/wpa_supplicant GROUP=netdev\nupdate_config=1\n' "${COUNTRY^^}"
  wpa_passphrase "$SSID" <<<"$PASSPHRASE" | sed '/^[[:space:]]*psk="/d'
} >"$TEMP_FILE" || die "could not generate the wpa_supplicant configuration"

unset PASSPHRASE
if [[ -f "$CONF" ]]; then
  backup="${CONF}.backup.$(date +%Y%m%d-%H%M%S)"
  cp -a -- "$CONF" "$backup"
  printf 'Backed up existing configuration to %s\n' "$backup"
fi
mv -- "$TEMP_FILE" "$CONF"
TEMP_FILE=''
chmod 600 "$CONF"

systemctl enable --now wpa_supplicant@wlan0.service
systemctl is-active --quiet wpa_supplicant@wlan0.service || die "wpa_supplicant failed to start"
printf 'Installed and started %s\n' "$CONF"
systemctl enable --now dhclient@wlan0.service
systemctl is-active --quiet dhclient@wlan0.service || die "dhclient@wlan0.service did not start; inspect its journal"
printf 'Enabled and started dhclient@wlan0.service (it waits for Wi-Fi association)\n'
