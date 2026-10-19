# shellcheck shell=bash
#
# Facts and helpers every baseline shares.
#
# A method directory holds its own adapters, its own manifest and its own
# scripts; everything true of *all* methods lives here, so a second method
# costs a manifest and a run.sh rather than a copy of the launcher.
#
# This file is sourced, never executed: robotsnap-baseline and every
# method script source it and use the helpers below.

# --- locations ----------------------------------------------------------------

baseline_root() {
    cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
}

# The workspace holding the application, the Python packages and the virtual
# environment. It defaults to the parent of this repository, which is how the
# workspace is laid out; ROBOTSNAP_WS overrides it for another layout.
workspace_root() {
    if [ -n "${ROBOTSNAP_WS:-}" ]; then
        printf '%s\n' "${ROBOTSNAP_WS}"
        return 0
    fi
    cd "$(baseline_root)/.." && pwd
}

venv_python() {
    printf '%s\n' "$(workspace_root)/venv_robotSNAP/bin/python"
}

unity_app_dir() {
    printf '%s\n' "$(workspace_root)/app/robotsnap-unity/${ROBOTSNAP_APP_VERSION:-0.1.0}"
}

unity_app() {
    printf '%s\n' "$(unity_app_dir)/robotsnap-unity.x86_64"
}

method_dir() {
    printf '%s\n' "$(baseline_root)/methods/$1"
}

method_list() {
    local path
    for path in "$(baseline_root)"/methods/*/; do
        [ -f "${path}run.sh" ] || continue
        basename "${path}"
    done
}

# --- the manifest of a method -------------------------------------------------

# One value of a method's manifest, or empty. The manifest is sourced in a
# subshell so a caller reads a single field without the file's other keys
# leaking into its own shell.
method_env_value() {
    local name="$1" key="$2" file
    file="$(method_dir "${name}")/method.env"
    [ -f "${file}" ] || return 0
    (
        # shellcheck disable=SC1090,SC1091
        . "${file}"
        eval "printf '%s' \"\${${key}:-}\""
    )
}

# The setup files a method declares in its manifest, resolved against the
# workspace and dropped when missing. A file the method lists but that is not
# there is a warning, not a hard stop: ROS itself ships on its own, and a build
# the user placed elsewhere should say so rather than abort a launch.
method_sources() {
    local name="$1" entry full list
    list="$(method_env_value "${name}" SOURCES)"
    local IFS=':'
    for entry in ${list}; do
        [ -n "${entry}" ] || continue
        full="$(workspace_root)/${entry}"
        if [ -f "${full}" ]; then
            printf '%s\n' "${full}"
        else
            warn "method ${name}: setup file missing, skipping: ${full}"
        fi
    done
}

# --- messaging ----------------------------------------------------------------

info() { printf '  %s\n' "$*"; }
ok()   { printf '  PASS  %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*" >&2; }
fail() { printf '  FAIL  %s\n' "$*" >&2; }
die()  { printf '%s\n' "$*" >&2; exit 1; }

# --- pins ---------------------------------------------------------------------

# shellcheck disable=SC1091
source_pins() { . "$(baseline_root)/pins.env"; }

# --- preconditions ------------------------------------------------------------

require_workspace() {
    local root
    root="$(workspace_root)"
    [ -d "${root}/app" ] || die "no application under ${root}; set ROBOTSNAP_WS to the workspace that holds app/ and venv_robotSNAP/"
    [ -x "$(venv_python)" ] || die "no virtual environment at $(venv_python); set ROBOTSNAP_WS, or build the workspace first"
}

require_method() {
    local name="${1:-}"
    [ -n "${name}" ] || die "which method? try: robotsnap-baseline list"
    [ -f "$(method_dir "${name}")/run.sh" ] || die "no method '${name}'; try: robotsnap-baseline list"
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "'$1' is not installed"
}

# --- sourcing ROS -------------------------------------------------------------

# Source one setup file from a script that runs under ``set -u``.
#
# ROS's own setup scripts, and the ament ones they chain to, read variables they
# never define - the first line of /opt/ros/humble/setup.bash trips on
# AMENT_TRACE_SETUP_FILES - so nounset has to be off around them or the source
# aborts before anything is exported. The previous setting is restored after.
source_setup() {
    local path="${1:?source_setup needs a path}" had_u=0 rc=0
    case "$-" in *u*) had_u=1 ;; esac
    set +u
    # shellcheck disable=SC1090,SC1091
    . "${path}" || rc=$?
    [ "${had_u}" = 1 ] && set -u
    return "${rc}"
}

# The ROS side of the environment, which every method needs.
source_ros() {
    source_setup "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
}

# --- the environment a method runs in -----------------------------------------

# The environment prefix every ROS 2 node needs, as a string to type into a
# pane. ROS comes first, then whatever setups the method adds, then the Python
# package, so a later source cannot put a shadowing "robotsnap" back in front.
#
# The ROS workspace installs an ament package *also* called "robotsnap" (the
# legacy robotsnap_pkg), which shadows the real Python package; the real one is
# put first so the name means one thing. FASTDDS_BUILTIN_TRANSPORTS pins the DDS
# transport, because machines of this class refuse to open the shared memory
# transport's ports and a node that cannot join the graph fails silently.
#
# A method's own directory is deliberately *not* added: its nodes are started by
# absolute path, so Python already puts the script's directory first, and a
# module name of theirs then cannot shadow a node of the stack.
ros_env() {
    local method="${1:?ros_env needs a method directory}"
    shift
    local prefix setup
    prefix="export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && source /opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    for setup in "$@"; do
        [ -n "${setup}" ] || continue
        prefix="${prefix} && source ${setup}"
    done
    prefix="${prefix} && export PYTHONPATH=$(workspace_root)/robotsnap/src:\${PYTHONPATH}"
    printf '%s' "${prefix}"
}

# --- the tmux session ---------------------------------------------------------

session_create() {
    tmux new-session -d -s "$1" -n unity -c "$2"
    tmux set-option -t "$1" automatic-rename off
    # A pane whose command exits - a node that cannot load a checkpoint, a stack
    # that fails to activate - would otherwise take its own error message with
    # it. It is a *window* option, so it is set globally to cover the windows
    # the method is about to create.
    tmux set-option -g remain-on-exit on
}

window_new() {
    tmux new-window -a -t "$1" -n "$2" -c "$3"
}

# Type one command into a window that already exists.
window_run() {
    tmux send-keys -t "$1:$2" "$3" Enter
}

# Split a window, then type into the pane the split made active. Targeting the
# window rather than a pane index keeps this independent of how many panes the
# window already holds.
pane_split() {
    tmux split-window -t "$1" "$2" -c "$3"
    tmux send-keys -t "$1" "$4" Enter
}

session_close() {
    tmux select-window -t "$1:unity"
    printf 'session "%s" started (windows: %s)\n' "$1" "$3"
    printf 'attach with "tmux attach -t %s", stop with "%s"\n' "$1" "$2"
}

# --- the windows every method gets --------------------------------------------

# The player, unless one is already open. The bracket in the pattern keeps this
# test from matching the pane's own command line, which is the test itself.
window_unity_guard() {
    window_run "$1" unity \
        "if pgrep -f '[r]obotsnap-unity.x86_64' >/dev/null 2>&1; then echo 'RobotSNAP is already open; leaving this pane free.'; else exec \"$(unity_app)\" $3; fi"
}

# RobotSNAP served to the ROS 2 graph: every method reads the session through it.
window_bridge() {
    window_new "$1" bridge "$2"
    window_run "$1" bridge "${3} && exec $(venv_python) -m robotsnap bridge --ros2"
}

# Hand the robot to ROS. The mode can only be set once a scenario is running,
# and the policy and the simulator start together, so the tool retries until the
# simulator accepts.
window_control() {
    window_new "$1" control "$2"
    window_run "$1" control \
        "${3} && exec $(venv_python) $(baseline_root)/tools/set_ros_control.py --robot ${4}"
}

# The view every method shares: one RViz2 on the shared configuration, and one
# marker pane beside it.
#
# RViz draws the map, the scan, the odometry and the TF tree on its own, but the
# two things that make social navigation *social* - where the people are and
# where the robot is trying to go - travel as JSON it cannot read. Without the
# marker node a method showed a map and a robot and the crowd and the goal were
# missing, while a method that ran it showed all four. That difference was a
# line in one run.sh and not a fact about the methods, so the pane lives here
# now and every method gets it.
#
# The robot argument is what the markers are drawn around: the bare topic names
# reach the primary robot and every other answers to /robot_<id>/..., so a view
# asked for the wrong one would silently follow robot_1.
window_rviz() {
    local session="$1" dir="$2" prefix="$3" config="${4:-}" robot="${5:-robot_1}"
    window_new "${session}" rviz "${dir}"
    if [ -n "${config}" ]; then
        window_run "${session}" rviz "${prefix} && exec rviz2 -d ${config}"
    else
        window_run "${session}" rviz "${prefix} && exec rviz2"
    fi
    pane_split "${session}:rviz" -v "${dir}" \
        "${prefix} && exec $(venv_python) $(baseline_root)/tools/scene_markers.py --robot ${robot}"
    tmux select-layout -t "${session}:rviz" tiled
}

# The goal a method is meant to reach, as the arguments `scenario_goal.py` wants,
# or nothing at all for a method that reads the session itself.
#
# The interface is a property of the method, so it is written in the method's own
# manifest beside the rest of what it is - not in its run.sh, where it would be a
# second place to change and could drift from the hint the window prints.
goal_interface_arguments() {
    local name="${1:-}" robot="${2:-robot_1}" interface topic
    [ -n "${name}" ] || return 0
    interface="$(method_env_value "${name}" GOAL_INTERFACE)"
    [ -n "${interface}" ] || return 0
    topic="$(method_env_value "${name}" GOAL_TOPIC)"
    printf -- '--interface %s --robot %s' "${interface}" "${robot}"
    # A method that reads a topic names it; one that answers an action does not.
    # The test is an `if` and not a `&&`: a trailing false would be this
    # function's exit status, and every caller runs under `set -e`, where
    # `arguments="$(goal_interface_arguments ...)"` would then take the whole
    # launch with it.
    if [ -n "${topic}" ]; then
        printf -- ' --topic %s' "${topic}"
    fi
    return 0
}

# The goal window: a shell to send a goal by hand, and - when the method declares
# one - the pane that hands it the scenario's own goal.
#
# Both are wanted. A scenario says where the robot has to go, and a method that
# never hears it stands still until somebody types one; but a benchmark also has
# to *try* a goal the scenario does not carry, which is what the shell is for.
window_goal() {
    local session="$1" dir="$2" prefix="$3" hint="${4:-}" name="${5:-}" robot="${6:-robot_1}"
    window_new "${session}" goal "${dir}"
    window_run "${session}" goal "${prefix}"
    window_run "${session}" goal "echo '${hint}'"

    local arguments
    arguments="$(goal_interface_arguments "${name}" "${robot}")"
    [ -n "${arguments}" ] || return 0

    pane_split "${session}:goal" -h "${dir}" \
        "${prefix} && exec $(venv_python) $(baseline_root)/tools/scenario_goal.py ${arguments}"
    tmux select-layout -t "${session}:goal" tiled
}

# --- les methodes ROS 1 -------------------------------------------------------

# Une methode ROS 1 a besoin de trois choses de plus qu'une methode ROS 2 : un
# maitre ROS 1, un pont entre ce maitre et le graphe ROS 2, et une vue ROS 1 des
# topics de RobotSNAP. Les trois sont dans tools/ros1/compose.yaml, une fois pour
# toutes ; une methode ne dit que ce qu'elle est (son image, sa commande, son
# environnement ROS 1). C'est ce qui evite que chaque methode recopie un pont.

ros1_project() {
    printf 'robotsnap_%s\n' "$1"
}

# La commande qui monte la session ROS 1 d'une methode, telle qu'on la tape dans
# un pane tmux. `%q` met chaque valeur a l'abri : un chemin avec une espace, une
# commande avec des guillemets n'ont pas besoin d'etre echappes a la main.
ros1_up_command() {
    local name="$1" image="$2" launch="$3" ros_env="$4" robot="$5"
    printf 'ROS1_BRIDGE_IMAGE=%q METHOD_IMAGE=%q METHOD_LAUNCH=%q METHOD_ROS_ENV=%q ADAPTER_DIR=%q ROBOT=%q docker compose -p %q -f %q up --remove-orphans' \
        "${ROS1_BRIDGE_IMAGE:?pins.env must define ROS1_BRIDGE_IMAGE}" \
        "${image}" "${launch}" "${ros_env}" \
        "$(baseline_root)/tools/ros1" "${robot}" \
        "$(ros1_project "${name}")" "$(baseline_root)/tools/ros1/compose.yaml"
}

# L'arret. `docker compose down` tue les quatre services ; tuer le client compose
# ne suffit pas, comme pour tout conteneur lance par un pane tmux.
#
# `down` reinterpole tout le fichier avant de savoir qu'il n'a rien a monter,
# donc les variables que le compose garde avec `:?` doivent exister meme si
# `down` ne s'en sert pas - sinon la commande est refusee et les conteneurs
# restent debout. Elles sont posees ici a des valeurs inertes, la ou `up` porte
# les vraies.
ros1_down_command() {
    local name="$1"
    printf 'ROS1_BRIDGE_IMAGE=%q METHOD_IMAGE=%q METHOD_LAUNCH=%q METHOD_ROS_ENV=%q ADAPTER_DIR=%q ROBOT=%q docker compose -p %q -f %q down --remove-orphans --timeout 5' \
        "${ROS1_BRIDGE_IMAGE:-unused}" unused unused unused /tmp unused \
        "$(ros1_project "${name}")" "$(baseline_root)/tools/ros1/compose.yaml"
}
