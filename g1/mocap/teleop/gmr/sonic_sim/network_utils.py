"""Network interface utilities (ported from gear_sonic.utils.network)."""

import platform
import re
import subprocess


def get_network_interfaces():
    """Get network interfaces with their IP addresses."""
    if platform.system() == "Windows":
        return _get_windows_interfaces()
    try:
        result = subprocess.run(
            ["/sbin/ip", "addr", "show"], capture_output=True, text=True, check=True
        )
        return _parse_ip_output(result.stdout)
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            result = subprocess.run(["ifconfig"], capture_output=True, text=True, check=True)
            return _parse_ifconfig_output(result.stdout)
        except (subprocess.CalledProcessError, FileNotFoundError):
            return {}


def _get_windows_interfaces():
    """Get network interfaces on Windows using ipconfig."""
    interfaces = {}
    try:
        result = subprocess.run(
            ["ipconfig"], capture_output=True
        )
        stdout = result.stdout.decode("mbcs", errors="replace")
        current_adapter = None
        for line in stdout.split("\n"):
            adapter_match = re.match(r"^(.+?):$", line.strip())
            if adapter_match and not adapter_match.group(1).startswith("Windows"):
                current_adapter = adapter_match.group(1).strip()
                interfaces[current_adapter] = []
            ipv4_match = re.search(
                r"(?:IPv4 Address|IPv4 .*)[.\s]*:\s*(\d+\.\d+\.\d+\.\d+)", line
            )
            if ipv4_match and current_adapter:
                interfaces[current_adapter].append(ipv4_match.group(1))
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    return interfaces


def _parse_ip_output(output):
    interfaces = {}
    current_interface = None
    for line in output.split("\n"):
        interface_match = re.match(r"^\d+:\s+(\w+):", line)
        if interface_match:
            current_interface = interface_match.group(1)
            interfaces[current_interface] = []
        ip_match = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", line)
        if ip_match and current_interface:
            interfaces[current_interface].append(ip_match.group(1))
    return interfaces


def _parse_ifconfig_output(output):
    interfaces = {}
    current_interface = None
    for line in output.split("\n"):
        interface_match = re.match(r"^(\w+):", line)
        if interface_match:
            current_interface = interface_match.group(1)
            interfaces[current_interface] = []
        ip_match = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", line)
        if ip_match and current_interface:
            interfaces[current_interface].append(ip_match.group(1))
    return interfaces


def find_interface_by_ip(target_ip):
    interfaces = get_network_interfaces()
    for interface, ip_list in interfaces.items():
        if target_ip in ip_list:
            return interface
    return None


def resolve_interface(interface: str) -> tuple[str, str]:
    """Resolve interface parameter to actual network interface name and env type.

    Args:
        interface: "sim", "real", or direct interface name or IP address.

    Returns:
        tuple: (interface_name, env_type) where env_type is "sim" or "real".
    """
    if re.match(r"^\d+\.\d+\.\d+\.\d+$", interface):
        if interface == "127.0.0.1":
            return interface, "sim"
        else:
            return interface, "real"

    if interface == "sim":
        if platform.system() == "Windows":
            # On Windows, CycloneDDS doesn't accept IP or "lo" as interface name.
            # Use empty string to trigger ChannelConfigAutoDetermine (auto-detect).
            return "", "sim"
        lo_interface = find_interface_by_ip("127.0.0.1")
        if lo_interface:
            if platform.system() == "Darwin" and lo_interface == "lo":
                return "lo0", "sim"
            return lo_interface, "sim"
        return ("lo0" if platform.system() == "Darwin" else "lo"), "sim"

    elif interface == "real":
        interfaces = get_network_interfaces()
        for iface, ip_list in interfaces.items():
            for ip in ip_list:
                if ip.startswith("192.168.123."):
                    return iface, "real"
        return interface, "real"

    else:
        interfaces = get_network_interfaces()
        if interface in interfaces:
            for ip in interfaces[interface]:
                if ip == "127.0.0.1":
                    return interface, "sim"
        if platform.system() == "Darwin" and interface == "lo":
            return "lo0", "sim"
        return interface, "real"
