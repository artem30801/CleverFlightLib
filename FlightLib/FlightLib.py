#!/usr/bin/python
from __future__ import print_function
import math
import sys
import time
import rospy
from clever import srv
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool


# init ros node
def init(node_name="CleverSwarmFlight", anon=True, no_signals=True):
    print("Initing")
    rospy.init_node(node_name, anonymous=anon, disable_signals=no_signals)
    print("Node inited")


# create proxy service
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

print("Proxy services inited")

# variables
x_current = 0
y_current = 0
z_current = 0


def check_isflipped(display=False):
    pi_2 = math.pi / 2
    telemetry = get_telemetry()
    flipped = not -pi_2 <= telemetry.pitch <= pi_2 or not -pi_2 <= telemetry.roll <= pi_2
    if display:
        print("Drone state - flipped:", flipped)
        if flipped:
            print("[!!] Drone IS flipped!")
    return flipped  #flipped=false means NOT flipped, everything good


def check_ismoving(display=False, tolerance=0.15, frame_id="fcu_horiz"):
    telemetry = get_telemetry(frame_id=frame_id)
    ismoving = abs(telemetry.vz) > tolerance or abs(telemetry.vx) > tolerance or abs(telemetry.vy) > tolerance
    if display:
        print("Drone state - is moving:", ismoving)
        if ismoving:
            print("[!!] Drone IS MOVING!")
    return ismoving

def safety_check(confirm=True): #TODO refactor as compilation of several cheks
    result = True
    isflipped(display=True)
    telemetry = get_telemetry(frame_id='aruco_map')
    print("Aruco telems are:", "x=", telemetry.x, ", y=", telemetry.y, ", z=", telemetry.z, "yaw=", telemetry.yaw, "pitch=",
          telemetry.pitch,
          "roll=", telemetry.pitch)
    telemetry = get_telemetry(frame_id='fcu_horiz')
    print("FCU telems are:", "V-z=", telemetry.vz, "voltage=", telemetry.voltage)
    if abs(telemetry.vz) > 0.2:
        print("[!!!] Estimated vartical speed is too high!")
    if confirm:
        raw_input("Are you sure about launch?")
        ans = raw_input("Are you ready to launch? Y/N: ")
        if ans.lower() != "y":
            sys.exit()
    return True


def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
    

def get_distance_global(lat1, lon1, alt1, lat2, lon2, alt2):
    dist_horiz = math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5
    return math.sqrt(dist_horiz ** 2 + (alt2 - alt1) ** 2)


def get_coordinates(frame_id='aruco_map'):
    telemetry = get_telemetry(frame_id=frame_id)
    return telemetry.x, telemetry.y, telemetry.z


def get_yaw(frame_id='aruco_map'):
    telemetry = get_telemetry(frame_id=frame_id)
    return telemetry.yaw


def get_distance_current(x, y, z, frame_id='aruco_map'):
    x_c, y_c, z_c = get_coordinates(frame_id=frame_id)
    return get_distance(x, y, z, x_c, y_c, z_c)


def capture_position(frame_id='aruco_map'):
    telemetry = get_telemetry(frame_id=frame_id)

    global x_current
    global y_current
    global z_current

    x_current = round(telemetry.x, 3)
    y_current = round(telemetry.y, 3)
    z_current = round(telemetry.z, 3)


def navto(x, y, z, yaw=float('nan'), speed=1.0, frame_id='aruco_map'):
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed)
    print(
        'Going to... | '
        'x: {:.3f} '
        'y: {:.3f} '
        'z: {:.3f} '
        'yaw: {:.3f}'.format(
            x, y, z, yaw
        ))
    return True


def reach(x, y, z, yaw=float('nan'), speed=1.0, tolerance=0.2, frame_id='aruco_map', wait_ms=100,
          timeout=7500):
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed)
    print('Reaching point | x: ', '{:.3f}'.format(x), ' y: ', '{:.3f}'.format(y), ' z: ', '{:.3f}'.format(z), ' yaw: ',
          '{:.3f}'.format(yaw), sep='')

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    time = 0
    while get_distance(x, y, z, telemetry.x, telemetry.y, telemetry.z) > tolerance:
        telemetry = get_telemetry(frame_id=frame_id)
        print('Reaching point | Telemetry | x: ', '{:.3f}'.format(telemetry.x), ' y: ', '{:.3f}'.format(telemetry.y),
              ' z: ', '{:.3f}'.format(telemetry.z), ' yaw: ', '{:.3f}'.format(telemetry.yaw), sep='')

        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout != 0 and (time >= timeout):
            print('Reaching point | Timed out! | t: ', time, sep='')
            return False
        rate.sleep()
    print("Point reached!")
    return True


def attitude(z, yaw=float('nan'), speed=1.0, tolerance=0.2, frame_id='aruco_map', wait_ms=100,
             timeout=5000):
    capture_position(frame_id=frame_id)
    navigate(frame_id=frame_id, x=x_current, y=y_current, z=z, yaw=yaw, speed=speed)
    print('Reaching attitude | z: ', '{:.3f}'.format(z), ' yaw: ', '{:.3f}'.format(yaw), sep='')

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    while abs(z - telemetry.z) > tolerance:
        telemetry = get_telemetry(frame_id=frame_id)
        print('Reaching attitude | Telemetry | z: ', '{:.3f}'.format(telemetry.z), ' yaw: ',
              '{:.3f}'.format(telemetry.yaw), sep='')

        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout != 0 and (time >= timeout):
            print('Reaching attitude | Timed out! | t: ', time, sep='')
            return False
        rate.sleep()
    print("Attitude reached!")
    return True


def rotate_to(yaw, tolerance=0.2, speed=1.0, frame_id='aruco_map', wait_ms=100, timeout=5000):
    capture_position(frame_id=frame_id)
    navigate(frame_id=frame_id, x=x_current, y=y_current, z=z_current, yaw=yaw, speed=speed)
    print('Reaching angle | yaw: ', '{:.3f}'.format(yaw), sep='')

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    while abs(yaw - telemetry.yaw) > tolerance:
        telemetry = get_telemetry(frame_id=frame_id)
        print('Reaching angle | Telemetry | yaw: ', '{:.3f}'.format(telemetry.yaw), sep='')

        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout != 0 and (time >= timeout):
            print('Reaching angle | Timed out! | t: ', time, sep='')
            return False
        rate.sleep()
    return True


def spin(yaw_rate=0.2, speed=1.0, frame_id='aruco_map', timeout=5000):
    capture_position(frame_id=frame_id)
    navigate(frame_id=frame_id, x=x_current, y=y_current, z=z_current, yaw=float('nan'), yaw_rate=yaw_rate, speed=speed)
    print('Spinning at speed | yaw_rate: ', '{:.3f}'.format(yaw_rate), sep='')
    rospy.sleep(timeout / 1000)
    if timeout is not None:
        navigate(frame_id=frame_id, x=x_current, y=y_current, z=z_current, yaw=float('nan'), yaw_rate=0.0, speed=speed)
        print('Spinning complete | Timeout | t: ', timeout, sep='')
    else:
        print('Spinning continuously')
    return True


def circle(x_point, y_point, z_point, r, speed=0.5, angle_init=0.0, angle_max=math.pi * 2, yaw=float('nan'),
           yaw_rate=0.0, frame_id='aruco_map', wait_ms=100):
    rate = rospy.Rate(1000 / wait_ms)
    delta = (wait_ms / 1000) * speed
    angle = angle_init

    x = (math.sin(angle) * r) + x_point
    y = (math.cos(angle) * r) + y_point

    print("Moving in circle | Moving to circle start point")
    reach(x, y, z_point, yaw=yaw, frame_id=frame_id)

    print('Moving in circle | Start point | x: ', '{:.3f}'.format(x_point), ' y: ',
          '{:.3f}'.format(y_point), ' z: ', '{:.3f}'.format(z_point), ' speed: ', '{:.3f}'.format(speed), sep='')
    while angle <= angle_max + angle_init:
        x = (math.sin(angle) * r) + x_point
        y = (math.cos(angle) * r) + y_point
        angle += delta

        telemetry = get_telemetry(frame_id=frame_id)
        print('Moving in circle | Start point | angle: ', '{:.3f}'.format(angle), sep='')
        print('Moving in circle | Telemetry | x: ', '{:.3f}'.format(telemetry.x), ' y: ',
              '{:.3f}'.format(telemetry.y), ' z: ', '{:.3f}'.format(telemetry.z), ' yaw: ',
              '{:.3f}'.format(telemetry.yaw), sep='')

        set_position(x=x, y=y, z=z_point, yaw=yaw, yaw_rate=yaw_rate, frame_id=frame_id)
        rate.sleep()
    print("Ended moving in circle")


def flip(side=False, invert=False, thrust=0.2):  
    # side=False for forward flip, side=True for right flip
    direction = -1 if invert else 1
    rate = (5 * math.pi) * direction
    capture_position()
    telemetry = get_telemetry()
    print("Starting flip...")
    if side:
        set_rates(roll_rate=rate, thrust=thrust)
        while abs(telemetry.roll) < math.pi / 2:
            telemetry = get_telemetry()

    else:
        set_rates( pitch_rate=rate, thrust=thrust)
        while abs(telemetry.pitch) < math.pi / 2:
            telemetry = get_telemetry()
    print("Stabilizing")
    set_position(x=x_current, y=y_current, z=z_current)

    print("Flipped, heading to flip start position")
    navto(x=x_current, y=y_current, z=z_current)


def takeoff(z=1, speed_takeoff=1.0, speed_inair=1.0, yaw=float('nan'), 
            frame_id_takeoff='fcu_horiz', frame_id_inair='aruco_map',
            tolerance=0.25, wait_ms=25, delay_fcu=1000, fixed_delay=False,
            timeout_arm=1500, timeout_takeoff=3000, timeout_inair=7500):
    if fixed_delay:
        fixed_delay_time = (delay_fcu+timeout_arm+timeout_takeoff+timeout_inair) / 1000
        delay_timer_start = rospy.get_rostime()
    print("Starting takeoff!")
    navigate(frame_id=frame_id_takeoff, x=0, y=0, z=z, yaw=float('nan'), speed=speed_takeoff, update_frame=False, auto_arm=True)

    telemetry = get_telemetry(frame_id=frame_id_takeoff)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    while not telemetry.armed:
        telemetry = get_telemetry(frame_id=frame_id_takeoff)
        print("Arming...")
        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout_arm != 0 and (time >= timeout_arm):
            print("Not armed, timed out.")
            break
            #print("Not ready to flight, exiting!")
            #sys.exit() #TODO maybe here can be another option...
        rate.sleep()

    print("In air!")
    rospy.sleep(1000 / delay_fcu)
    telemetry = get_telemetry(frame_id=frame_id_inair)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    while z - tolerance > telemetry.z:
        telemetry = get_telemetry(frame_id=frame_id_inair)
        print('Taking off | Telemetry | z: ', '{:.3f}'.format(telemetry.z), sep='')

        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout_takeoff != 0 and (time >= timeout_takeoff):
            print('Takeoff | Timed out! | t: ', time, sep='')
            break
        rate.sleep()

    print("Reaching takeoff attitude!")
    result = attitude(z, yaw=yaw, speed=speed_inair, tolerance=tolerance, timeout=timeout_inair, frame_id=frame_id_inair)
    if fixed_delay:
        dt = rospy.get_rostime() - delay_timer_start
        if dt < fixed_delay_time:
            time_to_sleep = fixed_delay_time - dt
            print("Fixed delay:", time_to_sleep)
            rospy.sleep(time_to_sleep)
    if result:
        print("Takeoff attitude reached. Takeoff completed!")
        return True
    else:
        print("Not reached takeoff attitude, timed out")
        return False


def land(preland=True, z=0.75, timeout_preland=10000, frame_id_preland='aruco_map', timeout_land=5000, wait_ms=100):
    if preland:
        print("Pre-Landing!")
        result = attitude(z, timeout=timeout_preland, frame_id=frame_id_preland)
        if result:
            print("Ready to land")
        else:
            print("Not ready to land, trying autoland mode.")

    telemetry = get_telemetry(frame_id='aruco_map')
    if telemetry.mode == "STABILIZED":
        print("Not in OFFBOARD mode, probably intercepted! | Not preforming autoland.")
        return False

    set_mode(base_mode=0, custom_mode='AUTO.LAND')
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()
    while telemetry.armed:
        telemetry = get_telemetry(frame_id='aruco_map')
        print('Landing | Telemetry | z: ', '{:.3f}'.format(telemetry.z), ' armed: ', telemetry.armed, sep='')

        time = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout_land != 0 and (time >= timeout_land):
            print("Not detected autoland, timed out. Disarming!")
            arming(False)
            return True
        rate.sleep()
    print("Land completed!")
    return True


if __name__ == "__main__":  # only if run FlightLib directly
    init()
    safety_check()
    takeoff()
    land()
