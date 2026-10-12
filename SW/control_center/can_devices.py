"""Identify SocketCAN hardware independently of its mutable interface name."""
import json
from pathlib import Path


def devices():
    found = []
    for interface in sorted(Path('/sys/class/net').iterdir()):
        try:
            if (interface / 'type').read_text().strip() != '280':
                continue
            physical = (interface / 'device').resolve()
            driver = (interface / 'device/driver').resolve().name
            entry = dict(interface=interface.name, driver=driver, usb_serial='',
                         usb_path='', vendor='', product_id='')
            for parent in [physical, *physical.parents]:
                if (parent / 'idVendor').is_file():
                    entry.update(usb_path=parent.name, vendor=(parent / 'idVendor').read_text().strip(),
                                 product_id=(parent / 'idProduct').read_text().strip())
                    serial = parent / 'serial'
                    entry['usb_serial'] = serial.read_text().strip() if serial.is_file() else ''
                    break
            found.append(entry)
        except OSError:
            continue
    return found


if __name__ == '__main__':
    print(json.dumps(devices(), ensure_ascii=False, indent=2))
