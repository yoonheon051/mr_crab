# 대시보드 터미널용 rcfile: 평소 .bashrc를 읽은 뒤 env.sh로 ROS 환경을 다시 덮어쓴다.
[ -f ~/.bashrc ] && source ~/.bashrc
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
