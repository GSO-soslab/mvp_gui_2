import subprocess
import threading
import os
import signal
import time
import tempfile

# Initialize global variables
ros_process = None
process_lock_gui = threading.Lock()
TIMEOUT = 2
ROS_PID_FILE = os.path.join(tempfile.gettempdir(), "ros_gui_pid.txt")

def start_mvp_gui_node_process(env, venv_activate_path, python_executable_path, ros_workspace_path):
    """Launch ROS 2 GUI process by executing a dedicated shell script in a new terminal."""
    global ros_process
    
    with process_lock_gui:
        if ros_process is None:
            cleanup_existing_process()
            
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            launcher_script_path = os.path.join(project_root, 'mvp_gui', 'launch_ros_interface.sh')

            if not os.path.exists(launcher_script_path):
                print(f"ERROR: Launcher script not found at {launcher_script_path}")
                return

            if not python_executable_path:
                print("ERROR: Python executable path is not set. Cannot start ROS node.")
                return

            # The command now calls the shell script with four arguments:
            # 1. Path to 'activate' script (can be an empty string)
            # 2. Path for the PID file
            # 3. Absolute path to the python executable
            # 4. Path to the ROS workspace (can be an empty string)
            ros_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c', 
                 (f'"{launcher_script_path}" '
                  f'"{venv_activate_path if venv_activate_path else ""}" '
                  f'"{ROS_PID_FILE}" '
                  f'"{python_executable_path}" '
                  f'"{ros_workspace_path if ros_workspace_path else ""}"; '
                  'exec bash')],
                env=env
            )
            print(f"Started ROS 2 GUI process via launcher script (Terminal PID: {ros_process.pid})")
            
            time.sleep(1)

def cleanup_existing_process():
    """Check for and clean up any existing ROS process from a previous run."""
    if os.path.exists(ROS_PID_FILE):
        try:
            with open(ROS_PID_FILE, 'r') as f:
                old_pid = int(f.read().strip())
                
            print(f"Found existing ROS process (PID: {old_pid}), attempting to terminate...")
            try:
                os.kill(old_pid, signal.SIGTERM)
                time.sleep(0.5)
                
                if is_process_running(old_pid):
                    print(f"Process {old_pid} didn't terminate, sending SIGKILL...")
                    os.kill(old_pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process {old_pid} not found, it may have already terminated")
            except Exception as e:
                print(f"Error terminating existing process {old_pid}: {e}")
                
            os.remove(ROS_PID_FILE)
            print(f"Removed stale PID file")
        except (ValueError, IOError) as e:
            print(f"Error processing existing PID file: {e}")
            try:
                os.remove(ROS_PID_FILE)
            except:
                pass

def is_process_running(pid):
    """Check if a process with the given PID is running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except Exception:
        return False

def get_ros_pid():
    """Read the actual ROS process PID from the temp file."""
    try:
        if os.path.exists(ROS_PID_FILE):
            with open(ROS_PID_FILE, 'r') as f:
                pid = int(f.read().strip())
                return pid
        return None
    except (ValueError, IOError) as e:
        print(f"Error reading PID file: {e}")
        return None

def is_running():
    pid = get_ros_pid()
    if pid is not None:
        return is_process_running(pid)
    return False

def stop_mvp_gui_node_process(env):
    """Stop the ROS GUI process by terminating its process ID."""
    global ros_process
    
    with process_lock_gui:
        ros_pid = get_ros_pid()
        if ros_pid:
            try:
                print(f"Killing gui_node process with PID: {ros_pid}")
                os.kill(ros_pid, signal.SIGTERM)
                time.sleep(0.5)
                
                if is_process_running(ros_pid):
                    print(f"Process {ros_pid} didn't terminate gracefully, using SIGKILL...")
                    os.kill(ros_pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process with PID {ros_pid} not found, may have already terminated")
            except Exception as e:
                print(f"Error killing process with PID {ros_pid}: {e}")
            finally:
                ros_process = None
                if os.path.exists(ROS_PID_FILE):
                    os.remove(ROS_PID_FILE)
                    print(f"Removed PID file")