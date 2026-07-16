#!/usr/bin/env python3
import os, re, signal, subprocess, time

NODE = '/g1_robot/front_camera'
SETUP = '/botbrain_ws/install/setup.bash'
FRAME_FILE = '/run/latest_cam.bin'
STALE_TIMEOUT_S = 45.0
INITIAL_NODE_WAIT_S = 20.0
NOT_FOUND_RESTART_COUNT = 20
TRANSITION_CONFIGURE = 1
TRANSITION_ACTIVATE = 3
TRANSITION_DEACTIVATE = 4

def ros2(*args, timeout=10):
    cmd = f'source {SETUP} && ros2 ' + ' '.join(args)
    r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout.strip(), r.stderr.strip()

def get_state():
    rc, out, _ = ros2(
        'service', 'call',
        f'{NODE}/get_state',
        'lifecycle_msgs/srv/GetState',
        '"{}"',
        timeout=10)
    if rc != 0:
        return None
    m = re.search(r"label='([^']+)'", out)
    return m.group(1) if m else None

def change_state(transition_id, timeout=15):
    rc, _, err = ros2(
        'service', 'call',
        f'{NODE}/change_state',
        'lifecycle_msgs/srv/ChangeState',
        f'"{{transition: {{id: {transition_id}}}}}"',
        timeout=timeout)
    if rc != 0 and err:
        log(f'change_state({transition_id}) failed: {err}')
    return rc == 0

def wait_for_state(target, timeout_s=15):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = get_state()
        if s == target:
            return True
        time.sleep(1)
    return False

def frame_file_age():
    try:
        return time.time() - os.path.getmtime(FRAME_FILE)
    except OSError:
        return float('inf')

def log(msg):
    print(f'[cam_lifecycle] {msg}', flush=True)

def realsense_process_running():
    r = subprocess.run(
        ['pgrep', '-f', 'realsense2_camera_node'],
        capture_output=True, text=True)
    return r.returncode == 0

def do_configure_activate():
    log('configuring...')
    change_state(TRANSITION_CONFIGURE, timeout=15)
    if not wait_for_state('inactive', timeout_s=12):
        log('configure did not reach inactive, waiting...')
        time.sleep(10)
        return
    # USB reset happens during configure — wait for device to fully re-initialize
    log('waiting for USB device stabilization...')
    time.sleep(8)
    log('activating...')
    change_state(TRANSITION_ACTIVATE, timeout=15)
    time.sleep(2)
    new_state = get_state()
    if new_state == 'active':
        log('active — camera ready')
    else:
        # Activate failed; wait before retry to avoid triggering SIGABRT
        log(f'activate result: {new_state}, waiting 15s before retry...')
        time.sleep(15)

log('starting — waiting for ROS nodes...')
time.sleep(INITIAL_NODE_WAIT_S)

_not_found_count = 0

while True:
    try:
        state = get_state()

        if state is None:
            _not_found_count += 1
            proc_msg = 'process running' if realsense_process_running() else 'process not found'
            log(
                f'node not found ({_not_found_count}/{NOT_FOUND_RESTART_COUNT}, '
                f'{proc_msg}), waiting...')
            if _not_found_count >= NOT_FOUND_RESTART_COUNT:
                log('realsense node gone — restarting container (kill PID 1)...')
                os.kill(1, signal.SIGTERM)
                time.sleep(60)
                import sys; sys.exit(0)
            time.sleep(3)
            continue

        _not_found_count = 0
        log(f'state: {state}')

        if state == 'active':
            age = frame_file_age()
            if age > STALE_TIMEOUT_S:
                # Zenoh publisher session timed out silently — restart it
                log(f'frame file stale {age:.0f}s — deactivating to reset Zenoh publisher...')
                change_state(TRANSITION_DEACTIVATE, timeout=15)
                wait_for_state('inactive', timeout_s=10)
                time.sleep(2)
                do_configure_activate()
            else:
                time.sleep(20)
            continue

        if state in ('unconfigured', 'errorprocessing'):
            do_configure_activate()
            continue

        if state == 'inactive':
            log('activating...')
            change_state(TRANSITION_ACTIVATE, timeout=15)
            time.sleep(2)
            continue

        time.sleep(5)

    except Exception as e:
        log(f'error: {e}')
        time.sleep(5)
