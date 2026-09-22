#!/usr/bin/env bash
# E0509 연결 점검: 유선 IP / ping / 컨트롤러 포트 / 워크스페이스
ROBOT_IP=${1:-110.120.1.68}
echo "== 유선 IP (110.120.1.x 이어야 함)"; ip -br -4 a | grep 110.120.1. || echo "  !! 110.120.1.x 주소 없음 -> 로봇 공유기에 랜선 연결 확인"
echo "== ping $ROBOT_IP"; ping -c2 -W1 "$ROBOT_IP" >/dev/null && echo "  ok" || echo "  !! 응답 없음 (전원 켠 직후면 ~2분 대기)"
echo "== 컨트롤러 포트 12345"; timeout 2 bash -c "</dev/tcp/$ROBOT_IP/12345" 2>/dev/null && echo "  open" || echo "  !! 닫힘"
echo "== 인터넷 기본 경로"; ip route | grep '^default' | head -1
source /opt/ros/jazzy/setup.bash; source ~/doosan_ws/install/setup.bash
python3 -c "import DR_init" 2>/dev/null && echo "== DSR python 모듈 ok" || echo "== !! DR_init import 실패 (colcon build 확인)"
echo "== ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
