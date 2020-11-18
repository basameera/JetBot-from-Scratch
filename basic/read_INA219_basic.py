import time
from ina219 import INA219
from ina219 import DeviceRangeError
# https://github.com/chrisb2/pi_ina219
SHUNT_OHMS = 0.1


def read():
    """
    INA219(shunt_ohms, max_expected_amps=None,
                    busnum=None, address=__ADDRESS,
                    log_level=logging.ERROR)
    """
    ina = INA219(SHUNT_OHMS, busnum=1, address=0x41)
    ina.configure()

    try:
        while True:

            try:
                V_BUS = ina.voltage()
                I_BUS = ina.current()
                P_BUS = ina.power()
                V_SHUNT = ina.shunt_voltage()

                print('Voltage: {:.3f} V | Current: {:.3f} mA | Power: {:.3f} mW | V Shunt: {:.3f} mV'.format(
                    V_BUS, I_BUS, P_BUS, V_SHUNT))

            except DeviceRangeError as e:
                # Current out of device range with specified shunt resistor
                print(e)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nBye!")


if __name__ == "__main__":
    read()
