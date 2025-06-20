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

            # The command now calls the shell script with four arguments.
            # IMPORTANT: We have removed '; exec bash' from the end of the command.
            # This ensures that the terminal will close automatically when the script it runs is finished.
            ros_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c', 
                 (f'"{launcher_script_path}" '
                  f'"{venv_activate_path if venv_activate_path else ""}" '
                  f'"{ROS_PID_FILE}" '
                  f'"{python_executable_path}" '
                  f'"{ros_workspace_path if ros_workspace_path else ""}"')],
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
    """Stop the ROS GUI process and the gnome-terminal it's running in."""
    global ros_process
    
    with process_lock_gui:
        # Step 1: Stop the ROS node process itself. This should cause the launch script
        # to exit, which in turn should cause the gnome-terminal to close automatically.
        ros_pid = get_ros_pid()
        if ros_pid:
            try:
                print(f"Stopping gui_node process with PID: {ros_pid}")
                os.kill(ros_pid, signal.SIGTERM)
                time.sleep(0.5)
                
                if is_process_running(ros_pid):
                    print(f"Process {ros_pid} didn't terminate gracefully, using SIGKILL...")
                    os.kill(ros_pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"ROS process with PID {ros_pid} not found, may have already terminated.")
            except Exception as e:
                print(f"Error stopping ROS process with PID {ros_pid}: {e}")

        # Step 2: As a fallback, check on the gnome-terminal process.
        # It should close on its own, but we'll manage it just in case.
        if ros_process and ros_process.poll() is None: 
            try:
                # Wait for the terminal to close on its own now that its child process is killed.
                print(f"Waiting for gnome-terminal (PID: {ros_process.pid}) to close...")
                ros_process.wait(timeout=TIMEOUT)
                print("Gnome-terminal closed gracefully.")
            except subprocess.TimeoutExpired:
                # If it doesn't close in time, terminate it forcefully.
                print(f"Gnome-terminal (PID: {ros_process.pid}) did not close automatically, forcing termination...")
                ros_process.terminate()
                try:
                    ros_process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    print("Terminal did not respond to SIGTERM, sending SIGKILL.")
                    ros_process.kill()
            except Exception as e:
                print(f"Error while managing gnome-terminal process: {e}")
        
        # Step 3: Final cleanup
        ros_process = None
        if os.path.exists(ROS_PID_FILE):
            try:
                os.remove(ROS_PID_FILE)
                print("Removed PID file.")
            except OSError as e:
                print(f"Error removing PID file: {e}")