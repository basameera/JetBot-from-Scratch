"""Simple example showing how to get gamepad events."""
from inputs import get_gamepad
import json
import os
import setup_path


key_map = dict()


def main():
    """Just print out some event infomation when the gamepad is used."""
    print('\n*** JetBot Gamepad Setup ***\n')
    print('First press one button or move one axis on the gamepad and then type the corresponding key-map you prefer using your keypad.')
    print('>>> Press button on Gamepad:')
    try:
        while True:

            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Sync':
                    continue
                if event.ev_type == 'Misc':
                    continue

                if(int(event.state) == 0):
                    key = event.ev_type + '-' + event.code
                    print(key)
                    print('>>> Enter mapping key on keyboard:', end=' ')
                    value = input()
                    print('Pair = ({}, {})'.format(key, value))
                    key_map.update({key: value})
                    print('\n>>> Press button on Gamepad (Press ctrl+c to save):')

    except KeyboardInterrupt:
        print("\nSaving keymap.json ....")
        print(key_map)

        print('\nChange the keymap.json file name to either laptop.json or jetson.json depending on the key-mapped system.')

        with open(os.path.join(os.getcwd(), 'jetbot_fs', 'keymap.json'), 'w') as outfile:
            json.dump(key_map, outfile)


if __name__ == "__main__":
    main()
