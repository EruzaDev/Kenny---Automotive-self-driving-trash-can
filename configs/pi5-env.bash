# Source from Bash. Keep ROS on system Python, separate from the RL venv.
source /opt/ros/jazzy/setup.bash
for workspace in "$HOME/kenny_ws" "$HOME/astra_ws"; do
    if [ -f "$workspace/install/local_setup.bash" ]; then
        source "$workspace/install/local_setup.bash"
    fi
done
unset workspace
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export KENNY_REPO="$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")"
alias kenny-python='"$KENNY_REPO/.venv/bin/python"'
