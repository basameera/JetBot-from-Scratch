from glob import glob

# /sys/devices/virtual/thermal/thermal_zone2/temp


if __name__ == "__main__":
    zone_list = sorted(glob('/sys/devices/virtual/thermal/thermal_zone*/temp'))

    print('===================')

    for zone in zone_list:

        a = zone.split('thermal_zone')[-1]
        num = int(a.split('/temp')[0])

        with open(zone, 'r') as z_file:
            value = float(z_file.read())/1000

        print('Thermal Zone {} : {:.1f} C'.format(num, value))
