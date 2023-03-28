#!../venv/bin/python3

import roslibpy
import time
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        prog='user_publisher',
                        description='What the program does',
                        epilog='Text at the bottom of help')

    parser.add_argument('-i', '--ip', help='bot IP address')
    parser.add_argument('-n', '--bot_name', help='bot name')

    args = parser.parse_args()
    names = vars(args)
    print(names)
    if None in list(set([names[i] for i in names.keys()])):
        raise Exception('no params')

    client = roslibpy.Ros(host=args.ip, port=9001)
    client.run()
    bot = args.bot_name
    axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    buttons[7] = 1
    start_pub = roslibpy.Topic(client, f'/{bot}/trajectory/end_trajectory_sub', 'std_msgs/Bool')
    map_state = roslibpy.Topic(client, f'/{bot}/planning_node/end_trajectory', 'std_msgs/Bool')
    joy_pub = roslibpy.Topic(client, f'/{bot}/joy', 'sensor_msgs/Joy')
    start_pub.publish(roslibpy.Message({'data': True}))
    map_state.publish(roslibpy.Message({'data': True}))
    joy_pub.publish(roslibpy.Message({'axes': axes, 'buttons': buttons}))
    time.sleep(1)

    client.terminate()
