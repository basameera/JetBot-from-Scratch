import setup_path
from jetbot_fs.joystick import GamePad

# functions


def go_forward():
    print('Go Forward')


def go_backward():
    print('Go Backward')


def turn_left():
    print('Turn Left')


def turn_right():
    print('Turn Right')


def stop():
    print('STOP')


# actions consist of a list of tuples => [(button, state, callback), ... ]
action_list = [
    ('X', 0, stop),
    ('Y', 0, stop),
    ('A', 0, stop),
    ('B', 0, stop),
    ('X', 1, turn_left),
    ('Y', 1, go_forward),
    ('A', 1, go_backward),
    ('B', 1, turn_right),
]

actions = dict()

for line in action_list:
    actions[str(line[0])+'_'+str(line[1])] = line[2]


def main():
    """Process all events forever."""
    gp = GamePad(actions=actions)

    try:
        while True:
            gp.process_events()
    except KeyboardInterrupt:
        print("\nBye!")


if __name__ == "__main__":
    main()
