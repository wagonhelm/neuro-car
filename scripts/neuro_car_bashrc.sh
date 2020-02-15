NEURO_CAR_HOME=$( cd "$( dirname "${BASH_SOURCE[0]}")/.."; pwd -P )

source "$NEURO_CAR_HOME/catkin_ws/devel/setup.bash" --extend

alias neuro_car_stack='tmuxinator start neuro_car_stack'
alias die="tmux kill-server"

