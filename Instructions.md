# AIG240 – Project 1 (Option 2 HARD) – Step-by-step (Ubuntu 18.04 + ROS Melodic) + VirtualBox Tips

This guide is written so you can **save it in your repo** (e.g., `PROJECT1_STEP_BY_STEP.md`) and follow it **top-to-bottom** inside your VirtualBox Ubuntu VM.

---

## 0) What you are building (Project 1 requirements checklist)

Your node must:
- Be a ROS1 node in a catkin workspace (ROS **Melodic** on **Ubuntu 18.04**).
- Create a package named **`lab3_turtlesim`**.
- Publish **`geometry_msgs/Twist`** to `/<turtle_name>/cmd_vel`.
- Take the turtle name from the command line, e.g.:
  - `rosrun lab3_turtlesim turtle_controller turtle1`
- Implement keyboard controls:
  - `w a s d` and also `q e z c`
  - Turtle moves **only while key(s) are pressed**
  - Turtle stops when keys are released
- **Option 2 (Hard):** support **multiple keypresses at once** (e.g., `w+a`), and contradictory keys cancel (e.g., `w+s` => no forward/back).

---

## 1) VirtualBox: How to move this file/code into the VM (pick one)

### Option A (Recommended): Shared Folder (1-time setup)
1. Power off the VM (full shutdown, not “Save State”).
2. VirtualBox Manager → select your VM → **Settings** → **Shared Folders** → **+**.
3. Choose a host folder (e.g., Desktop `vmshare`), set:
   - Folder Name: `vmshare`
   - ✅ Auto-mount
   - ✅ Make Permanent
4. Start the VM.
5. Install Guest Additions:
   - VirtualBox window menu: **Devices → Insert Guest Additions CD Image**
   - In Ubuntu terminal:
     ```bash
     sudo mkdir -p /media/cdrom
     sudo mount /dev/cdrom /media/cdrom
     cd /media/cdrom
     sudo ./VBoxLinuxAdditions.run
     ```
   - Reboot:
     ```bash
     sudo reboot
     ```
6. After reboot, your shared folder usually appears as:
   - `/media/sf_vmshare`
   - If permission denied:
     ```bash
     sudo usermod -aG vboxsf $USER
     ```
     Log out/in (or reboot).

### Option B: Download from your GitHub repo inside the VM
Inside Ubuntu VM:
```bash
sudo apt update
sudo apt install -y git
git clone <YOUR_REPO_URL>
cd <YOUR_REPO_FOLDER>
```

### Option C: Use a terminal editor inside the VM (nano)
Inside Ubuntu VM:
```bash
nano ~/somefile.txt
```
Save: `Ctrl+O`, Enter. Exit: `Ctrl+X`.

---

## 2) Open a terminal and source ROS Melodic (Terminal #1)

```bash
source /opt/ros/melodic/setup.bash
```

Make it automatic every time (run once):
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3) Go to your catkin workspace (from Lab 3)

### If you already have a workspace (common in labs):
```bash
cd ~/catkin_ws
source devel/setup.bash
```

### If you do NOT have one, create it:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4) Create the required package `lab3_turtlesim`

```bash
cd ~/catkin_ws/src
catkin_create_pkg lab3_turtlesim rospy geometry_msgs
```

---

## 5) Install the extra Python package for the HARD option (multi-key)

We’ll use `evdev` (works well on Ubuntu 18.04 and supports press/release + multi-key).

```bash
sudo apt update
sudo apt install -y python-evdev
```

---

## 6) Create the controller script (entirely from terminal)

### 6.1 Create the scripts folder
```bash
cd ~/catkin_ws/src/lab3_turtlesim
mkdir -p scripts
```

### 6.2 Create the controller file (copy/paste inside the VM terminal)
> If you cannot paste, use Shared Folder or `nano` and type it.

```bash
cat > scripts/turtle_controller <<'EOF'
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
            name_lower = dev.name.lower()
            if "keyboard" in name_lower or "kbd" in name_lower:
                return dev
    # fallback: first EV_KEY device
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_KEY in caps:
            return dev
    return None

def compute_twist(lin=2.0, ang=2.0):
    """
    Option 2 (Hard):
    - multiple keys at once (w+a, w+d, s+a, s+d)
    - contradictory keys (w+s or a+d) cancel
    - move only while pressed; stop when released
    - includes q/e/z/c
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

    # Special keys q/e/z/c (priority)
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

    # Multi-key combos
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

    # Single-axis moves
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

    # Project requirement: turtle name from command line
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
        rospy.logerr("No keyboard input device found in /dev/input.")
        rospy.logerr("If you're in VirtualBox, see the troubleshooting section in this guide.")
        sys.exit(1)

    rospy.loginfo("Keyboard device: %s (%s)", dev.path, dev.name)

    # Grab exclusive access for consistent key events
    try:
        dev.grab()
    except Exception as e:
        rospy.logwarn("Could not grab device (continuing). Error: %s", str(e))

    rate = rospy.Rate(30)

    try:
        for event in dev.read_loop():
            if rospy.is_shutdown():
                break

            if event.type == ecodes.EV_KEY:
                key_code = event.code
                key_state = event.value  # 1=down, 0=up, 2=hold/repeat

                key_name = ecodes.KEY.get(key_code, None)
                if not key_name:
                    continue

                if key_name.startswith("KEY_"):
                    k = key_name.replace("KEY_", "").lower()
                    if k in KEYS:
                        if key_state == 1:      # down
                            pressed.add(k)
                        elif key_state == 0:    # up
                            pressed.discard(k)

            pub.publish(compute_twist())
            rate.sleep()

    finally:
        stop = Twist()
        pub.publish(stop)
        try:
            dev.ungrab()
        except:
            pass

if __name__ == "__main__":
    main()
EOF
```

### 6.3 Make it executable
```bash
chmod +x scripts/turtle_controller
```

---

## 7) Build the workspace and source it

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 8) Run everything (3 terminals)

### Terminal #2: roscore
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### Terminal #3: turtlesim
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun turtlesim turtlesim_node
```

### Terminal #1: your controller for turtle1
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun lab3_turtlesim turtle_controller turtle1
```

Test:
- Hold `w` forward, `s` back, `a` left turn, `d` right turn
- Hold `w+a` (forward + left curve) ✅
- Hold `w+d` (forward + right curve) ✅
- Contradictions:
  - `w+s` cancels forward/back => no linear motion
  - `a+d` cancels turning => no angular motion
- Release keys => turtle stops ✅

---

## 9) Multi-turtle test (recommended)
Spawn turtle2:
```bash
rosservice call /spawn 2.0 2.0 0.0 "turtle2"
```

Run another controller (new terminal):
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun lab3_turtlesim turtle_controller turtle2
```

---

## 10) Verify publishing (for assessment)

```bash
rostopic echo /turtle1/cmd_vel
```

Useful:
```bash
rostopic list
rostopic info /turtle1/cmd_vel
```

---

## 11) Launch file (to start turtlesim + controller together)
Create a launch file:

```bash
cd ~/catkin_ws/src/lab3_turtlesim
mkdir -p launch

cat > launch/project1.launch <<'EOF'
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="turtlesim" output="screen" />
  <node pkg="lab3_turtlesim" type="turtle_controller" name="controller_turtle1" args="turtle1" output="screen" />
</launch>
EOF
```

Build and run:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch lab3_turtlesim project1.launch
```

---

## 12) VirtualBox troubleshooting (keyboard events)

### Check if VM can see input devices
```bash
ls -l /dev/input/
ls -l /dev/input/event*
```

### List all input devices with names
```bash
python -c "from evdev import InputDevice, list_devices; [print(p, InputDevice(p).name) for p in list_devices()]"
```

### Permission fix (common)
Add your user to the `input` group:
```bash
sudo usermod -aG input $USER
```
Then log out/in (or reboot).

---

## 13) What to submit (Project 1 typical items)
- Your code (repo link)
- Your demo video (show multi-key working + stop on release)
- Your text answers to assessment questions (commands used, Twist, launch steps, rostopic echo, etc.)
- If you used GenAI tools, declare which tool and which part.

---

## 14) Quick “clean run” checklist
- `catkin_make` completes with no errors
- `roscore` running
- `turtlesim_node` running
- `rosrun lab3_turtlesim turtle_controller turtle1` runs without device errors
- Turtle moves only while keys are pressed
- Multi-key combos work (w+a etc.)
- `rostopic echo /turtle1/cmd_vel` shows Twist messages changing as you press keys
