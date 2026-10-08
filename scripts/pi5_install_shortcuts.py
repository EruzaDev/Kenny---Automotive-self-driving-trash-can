"""Install the current checkout's Bash environment and desktop GUI shortcut."""
from pathlib import Path
import shlex
import subprocess


def main():
    repo = Path(__file__).resolve().parents[1]
    home = Path.home()
    (home / "kenny_env.bash").write_text(
        "# KENNY Raspberry Pi environment\nsource " + shlex.quote(str(repo / "configs/pi5-env.bash")) + "\n")
    bashrc = home / ".bashrc"
    text = bashrc.read_text() if bashrc.exists() else ""
    if "kenny_env.bash" not in text:
        bashrc.write_text(text + '\n# KENNY Raspberry Pi / ROS 2 Jazzy\n'
                         'if [ -f "$HOME/kenny_env.bash" ]; then\n'
                         '    source "$HOME/kenny_env.bash"\nfi\n')
    desktop = home / "Desktop"
    desktop.mkdir(exist_ok=True)
    shortcut = desktop / "Kenny-Sensors.desktop"
    executable = str(repo / "scripts/pi5_open_gui.py").replace("\\", "\\\\").replace('"', '\\"')
    shortcut.write_text("[Desktop Entry]\nType=Application\nName=Kenny Live Sensors\n"
                        "Comment=Live RGB, depth and red-near green-far 3D depth rays\n"
                        f'Exec=/usr/bin/python3 "{executable}"\n'
                        "Icon=camera-photo\nTerminal=false\nCategories=Science;Robotics;\n")
    shortcut.chmod(0o755)
    subprocess.run(["gio", "set", str(shortcut), "metadata::trusted", "true"], check=False)
    print(f"Installed {home / 'kenny_env.bash'} and {shortcut}")


if __name__ == "__main__":
    main()
