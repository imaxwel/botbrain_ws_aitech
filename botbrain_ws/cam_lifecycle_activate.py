#!/usr/bin/env python3
"""
Watchdog: configure+activate /g1_robot/front_camera lifecycle node.
After configure, the realsense SDK resets the USB device (~4s).
If activate is sent before the reset completes, it fails with null ptr.
We wait 8s after configure to let the device fully re-initialize.
"""
import subprocess, time, sys

NODE = '/g1_robot/front_camera'
SETUP = '/botbrain_ws/install/setup.bash'

def ros2(*args, timeout=10):
    cmd = f'source {SETUP} && ros2 ' + ' '.join(args)
    r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout.strip(), r.stderr.strip()

def get_state():
    rc, out, _ = ros2('lifecycle', 'get', NODE)
    if rc != 0:
        return None
    return out.split()[0] if out else None

def wait_for_state(target, timeout_s=15):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = get_state()
        if s == target:
            return True
        time.sleep(1)
    return False

def log(msg):
    print(f'[cam_lifecycle] {msg}', flush=True)

log('starting — waiting for ROS nodes...')
time.sleep(8)

while True:
    try:
        state = get_state()

        if state is None:
            log('node not found, waiting...')
            time.sleep(3)
            continue

        log(f'state: {state}')

        if state == 'active':
            time.sleep(20)
            continue

        if state in ('unconfigured', 'errorprocessing'):
            log('configuring...')
            ros2('lifecycle', 'set', NODE, 'configure', timeout=15)
            if not wait_for_state('inactive', timeout_s=12):
                log('configure did not reach inactive, waiting...')
                time.sleep(10)
                continue
            # USB reset happens during configure — wait for device to fully re-initialize
            log('waiting for USB device stabilization...')
            time.sleep(8)
            log('activating...')
            ros2('lifecycle', 'set', NODE, 'activate', timeout=15)
            time.sleep(2)
            new_state = get_state()
            if new_state == 'active':
                log('active — camera ready')
            else:
                # Activate failed; wait before retry to avoid triggering SIGABRT
                log(f'activate result: {new_state}, waiting 15s before retry...')
                time.sleep(15)
            continue

        if state == 'inactive':
            log('activating...')
            ros2('lifecycle', 'set', NODE, 'activate', timeout=15)
            time.sleep(2)
            continue

        time.sleep(5)

    except Exception as e:
        log(f'error: {e}')
        time.sleep(5)
