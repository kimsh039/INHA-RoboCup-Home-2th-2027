"""Read the actual Livox host bindings used by its SDK."""
import ipaddress
import json
from pathlib import Path


def bindings(path):
    config = json.loads(Path(path).read_text())
    hosts = []
    for section in config.values():
        if isinstance(section, dict) and 'host_net_info' in section:
            entries = section['host_net_info']
            hosts.extend(entries if isinstance(entries, list) else [entries])
    if not hosts:
        raise ValueError('Livox 설정에 host_net_info가 없습니다: ' + str(path))
    result = set()
    for entry in hosts:
        address = str(ipaddress.IPv4Address(entry['host_ip']))
        result.add((address, 44332))
        for key, value in entry.items():
            if key.endswith('_port'):
                port = int(value)
                if not 0 <= port <= 65535:
                    raise ValueError('Livox UDP 포트 범위 오류')
                if port:
                    result.add((address, port))
    result.add(('0.0.0.0', 56000))
    return sorted(result)
