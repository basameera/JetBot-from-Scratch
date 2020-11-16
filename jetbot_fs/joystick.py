
"""Simple gamepad/joystick test example.
https://inputs.readthedocs.io/en/latest/user/examples.html
"""

import setup_path
import os
import inputs
import json

host_name = os.uname()[1]
print('Host name:', host_name)

if host_name == 'Kiwi':
    # Jetson
    EVENT_ABB = os.path.join(os.getcwd(), 'jetbot_fs', 'jetson.json')

if host_name == 'Linux':
    # # Laptop
    EVENT_ABB = os.path.join(os.getcwd(), 'jetbot_fs', 'laptop.json')


# This is to reduce noise from the PlayStation controllers
# For the Xbox controller, you can set this to 0
MIN_ABS_DIFFERENCE = 1


class GamePad(object):
    """Simple GamePad class."""

    def __init__(self, gamepad=None, abbrevs=EVENT_ABB, actions=None, MIN_ABS_DIFFERENCE=1):
        self.btn_state = {}
        self.old_btn_state = {}
        self.abs_state = {}
        self.old_abs_state = {}
        self.actions = actions
        self.MIN_ABS_DIFFERENCE = MIN_ABS_DIFFERENCE
        with open(abbrevs) as f:
            self.abbrevs = json.load(f)

        for key, value in self.abbrevs.items():
            if key.startswith('Absolute'):
                self.abs_state[value] = 0
                self.old_abs_state[value] = 0
            if key.startswith('Key'):
                self.btn_state[value] = 0
                self.old_btn_state[value] = 0

        self._other = 0
        self.gamepad = gamepad
        if not gamepad:
            self._get_gamepad()

    def _get_gamepad(self):
        """Get a gamepad object."""
        try:
            self.gamepad = inputs.devices.gamepads[0]
        except IndexError:
            raise inputs.UnpluggedError("No gamepad found.")

    def handle_unknown_event(self, event, key):
        """Deal with unknown events."""
        if event.ev_type == 'Key':
            new_abbv = 'B' + str(self._other)
            self.btn_state[new_abbv] = 0
            self.old_btn_state[new_abbv] = 0
        elif event.ev_type == 'Absolute':
            new_abbv = 'A' + str(self._other)
            self.abs_state[new_abbv] = 0
            self.old_abs_state[new_abbv] = 0
        else:
            return None

        self.abbrevs[key] = new_abbv
        self._other += 1

        return self.abbrevs[key]

    def process_event(self, event):
        """Process the event into a state."""
        if event.ev_type == 'Sync':
            return
        if event.ev_type == 'Misc':
            return
        key = event.ev_type + '-' + event.code
        try:
            abbv = self.abbrevs[key]
        except KeyError:
            abbv = self.handle_unknown_event(event, key)
            if not abbv:
                return
        if event.ev_type == 'Key':
            self.old_btn_state[abbv] = self.btn_state[abbv]
            self.btn_state[abbv] = event.state
        if event.ev_type == 'Absolute':
            self.old_abs_state[abbv] = self.abs_state[abbv]
            self.abs_state[abbv] = event.state

        self.output_state(event.ev_type, abbv)

    def format_state(self):
        """Format the state."""
        output_string = ""
        for key, value in self.abs_state.items():
            output_string += key + ':' + '{:>4}'.format(str(value) + ' ')

        for key, value in self.btn_state.items():
            output_string += key + ':' + str(value) + ' '

        return output_string

    def output_state(self, ev_type, abbv):
        """Print out the output state."""
        # handle buttons
        if ev_type == 'Key':
            try:
                if self.btn_state[abbv] != self.old_btn_state[abbv]:
                    action_key = str(abbv) + '_' + str(self.btn_state[abbv])
                    action_func = self.actions[action_key]
                    action_func()
                    # print(self.format_state())
                    return
            except KeyError as e:
                pass

        # # handle D-Pad buttons. I don't know why they are seperate???
        # if abbv[0] == 'H':
        #     print(self.format_state())
        #     return

        # # handle axis analogue value changes
        # difference = self.abs_state[abbv] - self.old_abs_state[abbv]
        # if (abs(difference)) > self.MIN_ABS_DIFFERENCE:
        #     print(self.format_state())

    def process_events(self):
        """Process available events."""
        try:
            events = self.gamepad.read()
        except EOFError:
            events = []
        for event in events:
            self.process_event(event)


if __name__ == "__main__":
    pass
