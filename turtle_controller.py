#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist

from evdev import InputDevice, list_devices, ecodes

# Keys required by Project 1
KEYS = set(['w','a','s','d','q','e','z','c'])
pressed = set()

def pick_keyboard_device():
    """
    Try to auto-detect a keyboard device from /dev/input/event*
    """
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_KEY in caps:
            # This is a heuristic: many keyboards include 'keyboard' in name
            name_lower = dev.name.lower()
            if "keyboard" in name_lower or "kbd" in name_lower:
                return dev
    # fallback: just take first EV_KEY device
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_KEY in caps:
            return dev
    return None

def compute_twist(lin=2.0, ang=2.0):
    """
    Hard option:
    - multiple keys at once (w+a, w+d, s+a, s+d)
    - contradictory keys (w+s or a+d) => no movement for that axis
    - move only while pressed; stop when released (publishing loop sends 0s when no keys)
    - includes q/e/z/c like turtlesim teleop (curved / diagonal behaviour)
    """
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0

    forward  = ('w' in pressed)
    backward = ('s' in pressed)
    left     = ('a' in pressed)
    right    = ('d' in pressed)

    # Contradictions cancel out
    if forward and backward:
        forward = backward = False
    if left and right:
        left = right = False

    # Single special keys q/e/z/c
    # (These take priority if held)
    if 'q' in pressed:
        t.linear.x = lin
        t.angular.z = ang
        return t
    if 'e' in pressed:
        t.linear.x = lin
        t.angular.z = -ang
        return t
    if 'z' in pressed:
        t.linear.x = -lin
        t.angular.z = ang
        return t
    if 'c' in pressed:
        t.linear.x = -lin
        t.angular.z = -ang
        return t

    # Multi-key combos (hard requirement)
    if forward and left:
        t.linear.x = lin
        t.angular.z = ang
        return t
    if forward and right:
        t.linear.x = lin
        t.angular.z = -ang
        return t
    if backward and left:
        t.linear.x = -lin
        t.angular.z = ang
        return t
    if backward and right:
        t.linear.x = -lin
        t.angular.z = -ang
        return t

    # Single axis moves
    if forward:
        t.linear.x = lin
    elif backward:
        t.linear.x = -lin

    if left:
        t.angular.z = ang
    elif right:
        t.angular.z = -ang

    return t

def main():
    rospy.init_node("turtle_controller", anonymous=True)

    # Project requirement: take turtle name from command line
    if len(sys.argv) < 2:
        print("Usage: rosrun lab3_turtlesim turtle_controller <turtle_name>")
        sys.exit(1)

    turtle_name = sys.argv[1]
    topic = "/" + turtle_name + "/cmd_vel"
    pub = rospy.Publisher(topic, Twist, queue_size=10)

    rospy.loginfo("Controlling turtle: %s", turtle_name)
    rospy.loginfo("Keys: w a s d + q e z c (multi-key supported). Ctrl+C to quit.")

    dev = pick_keyboard_device()
    if dev is None:
        rospy.logerr("No keyboard input device found in /dev/input. Are you in a VM without input access?")
        sys.exit(1)

    rospy.loginfo("Keyboard device: %s (%s)", dev.path, dev.name)

    # Grab exclusive access so keystrokes are consistent
    try:
        dev.grab()
    except Exception as e:
        rospy.logwarn("Could not grab device (continuing anyway). Error: %s", str(e))

    rate = rospy.Rate(30)

    try:
        for event in dev.read_loop():
            if rospy.is_shutdown():
                break

            if event.type == ecodes.EV_KEY:
                key_event = event
                key_code = key_event.code
                key_state = key_event.value  # 1=down, 0=up, 2=hold/repeat

                # Convert code -> key name (like 'KEY_W')
                key_name = ecodes.KEY.get(key_code, None)
                if not key_name:
                    continue

                # Interested only in letters we map
                # KEY_W -> 'w'
                if key_name.startswith("KEY_"):
                    k = key_name.replace("KEY_", "").lower()
                    if k in KEYS:
                        if key_state == 1:      # key down
                            pressed.add(k)
                        elif key_state == 0:    # key up
                            if k in pressed:
                                pressed.remove(k)

            # Publish continuously with current pressed set
            twist = compute_twist()
            pub.publish(twist)
            rate.sleep()

    finally:
        # Always stop turtle on exit
        stop = Twist()
        pub.publish(stop)
        try:
            dev.ungrab()
        except:
            pass

if __name__ == "__main__":
    main()
