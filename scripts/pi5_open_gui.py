"""Open the ROS sensor viewer detached from the calling terminal."""
import os
from pathlib import Path
import subprocess


def main():
    existing = subprocess.run(["pgrep", "-u", str(os.getuid()), "-f",
                               "ros2 launch .*/scripts/pi5_gui[.]launch[.]py"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if existing.returncode == 0:
        subprocess.run(["wmctrl", "-a", "RViz"], check=False)
        print("Kenny sensor GUI is already running; brought RViz to the front.")
        return
    repo = Path(__file__).resolve().parents[1]
    output = repo / "artifacts/pi5"
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.setdefault("DISPLAY", ":0")
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    with (output / "gui.log").open("w") as log:
        process = subprocess.Popen([
            "bash", "-c",
            'source "$1/configs/pi5-env.bash" && exec ros2 launch "$KENNY_REPO/scripts/pi5_gui.launch.py"',
            "kenny-gui", str(repo),
        ], env=env, stdin=subprocess.DEVNULL, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
    print(f"Started Kenny sensor GUI (launch PID {process.pid}). Log: {output / 'gui.log'}")


if __name__ == "__main__":
    main()
