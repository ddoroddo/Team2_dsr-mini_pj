# Team2_dsr-mini_pj

두산 로보틱스 **E0509** 로봇팔 + **RH-P12-RN** 그리퍼 + **RealSense** 카메라로 하는 픽앤플레이스 미니 프로젝트 스크립트 모음.
이 문서는 **터미널 실행 명령만** 정리한 것입니다.

| 파일 | 용도 |
|---|---|
| `check_e0509.sh` | 로봇 연결 점검 (유선 IP · ping · 컨트롤러 포트 · 워크스페이스) |
| `gripper_repeat.py` | 한 자세에서 그리퍼 열기/닫기 반복 |
| `motion_seq.py` | **연속 자세 녹화/재생** (자세마다 그리퍼 동작 포함) |
| `pick_place_hardcoded.py` | pick / place 두 자세로 하는 단순 픽앤플레이스 |
| `vision_pick_place.py` | 카메라 화면 클릭 → 핸드-아이 캘리브 결과로 물체 위치 계산 → 집기 |
| `detector_node.py` | 카메라 + YOLO 검출 → 물체 3D 좌표를 `/detections` 토픽으로 발행 (임시 COCO 모델, 팀원 모듈로 교체 예정) |
| `object_picker.py` | `/detections` 를 받아서 **요청한 라벨의 물체**를 집어 place 에 놓기 |
| `sequences.json`, `poses.json` | 녹화된 자세 (`motion_seq.py` / `pick_place_hardcoded.py` 가 사용) |

---

## 0. 환경

- Ubuntu 24.04 + ROS 2 Jazzy, [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) (jazzy) 를 `~/doosan_ws` 에 빌드
- 그리퍼 패키지 `dsr_gripper`, `dsr_gripper_interfaces` 빌드 완료
- 로봇 컨트롤러 IP `110.120.1.68` (로봇마다 다름), `ROS_DOMAIN_ID=40`
- 스크립트 위치: `~/doosan_ws/scripts`

모든 터미널에서 먼저:

```bash
source /opt/ros/jazzy/setup.bash
source ~/doosan_ws/install/setup.bash
export ROS_DOMAIN_ID=40
cd ~/doosan_ws/scripts
```

## 1. 로봇 실행 (항상 먼저)

로봇 전원 ON → 약 2분 대기 → 연결 점검:

```bash
./check_e0509.sh              # 기본 IP 110.120.1.68
./check_e0509.sh 110.120.1.13 # 다른 로봇
```

**터미널 1 — bringup**

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.68 model:=e0509
```

**터미널 2 — 그리퍼 서비스**

```bash
ros2 run dsr_gripper gripper_service
```

그리퍼 단독 명령 (position: 0=닫힘 ~ 750=열림, current: 힘):

```bash
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd "{position: 750, current: 200}"  # 열기
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd "{position: 0, current: 300}"    # 닫기
```

종료: bringup 터미널에서 `Ctrl+C` (서보 토크 해제).

> ⚠️ **직접교시(손으로 팔 옮기기)와 gripper_service 는 동시에 쓸 수 없습니다.**
> gripper_service 가 컨트롤러에 DRL 프로그램을 띄워 두면 직접교시가 막힙니다.
> 녹화할 때는 gripper_service 를 끄고 아래로 DRL 을 멈추세요:
> ```bash
> ros2 service call /dsr01/dsr_controller2/drl/drl_stop dsr_msgs2/srv/DrlStop "{stop_mode: 2}"
> ```

---

## 2. `motion_seq.py` — 연속 자세 녹화 / 재생

### 녹화 (gripper_service 끈 상태)

```bash
python3 motion_seq.py record pick_box
```

manual 모드로 자동 전환됩니다. 직접교시 버튼으로 팔을 옮기고 **버튼에서 손을 뗀 뒤** 키 입력:

| 키 | 동작 |
|---|---|
| `Enter` | 현재 자세 추가 |
| `o` | 추가 + 도착 후 그리퍼 **열기** |
| `c` | 추가 + 도착 후 그리퍼 **닫기** |
| `l` | 다음에 추가할 자세는 **직선(movel)** 이동 |
| `u` | 마지막 자세 취소 |
| `s` | 목록 보기 |
| `q` | 저장하고 종료 (auto 모드로 복귀) |

한 줄씩 추가하는 방식:

```bash
python3 motion_seq.py add pick_box --grip close --move l --wait 1.0
```

### 확인 / 편집

```bash
python3 motion_seq.py list                 # 동작 이름 목록
python3 motion_seq.py list pick_box        # 자세 목록
python3 motion_seq.py del pick_box 3       # 3번 자세 삭제
python3 motion_seq.py clear pick_box       # 동작 전체 삭제 (sequences.json.bak 백업)
```

### 재생 (auto 모드 + gripper_service 실행 중)

```bash
python3 motion_seq.py run pick_box --dry           # 움직이지 않고 순서만 출력
python3 motion_seq.py go pick_box 2                # 2번 자세로만 이동 (그리퍼 동작 없음)
python3 motion_seq.py run pick_box                 # 1회 재생
python3 motion_seq.py run pick_box -n 3            # 3회 반복
python3 motion_seq.py run pick_box --grip-wait 2   # 그리퍼 후 2초 대기 (기본 1.5)
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `-n` | 1 | 반복 횟수 |
| `--vel` | 20 | 관절 속도 (deg/s) |
| `--lvel` | 50 | 직선 속도 (mm/s) |
| `--current` | 300 | 닫는 힘 (200~400) |
| `--grip-wait` | 1.5 | 그리퍼 명령 후 다음 이동까지 대기 (초) |
| `--yes` | – | 시작 확인(Enter) 생략 |

각 자세는 **도착 확인(모션 종료 + 관절각 0.5° 이내) 후에만** 그리퍼가 움직입니다.

---

## 3. `pick_place_hardcoded.py` — 두 자세 픽앤플레이스

```bash
python3 pick_place_hardcoded.py mode manual   # 직접교시 가능 상태
python3 pick_place_hardcoded.py teach pick    # 잡는 높이에서 저장
python3 pick_place_hardcoded.py teach place   # 놓는 높이에서 저장
python3 pick_place_hardcoded.py mode auto     # 실행 전 반드시 auto
python3 pick_place_hardcoded.py show          # 저장된 자세 보기
python3 pick_place_hardcoded.py run -n 1      # 실행
```

동작: 준비자세 → pick 100mm 위 → 수직 하강 → 닫기 → 상승 → place 위 → 하강 → 열기 → 상승 → 준비자세

```bash
python3 pick_place_hardcoded.py teach place2              # 이름은 자유
python3 pick_place_hardcoded.py run --place place2        # 다른 자세 사용
python3 pick_place_hardcoded.py delete place2             # 하나 삭제
python3 pick_place_hardcoded.py reset                     # 전부 삭제 (poses.json.bak 백업)
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--approach` | 100 | 접근/상승 높이 (mm) |
| `--vel` | 30 | 직선 속도 (mm/s) |
| `--grip` | 300 | 잡는 힘 |
| `--grip-wait` | 1.5 | 그리퍼 후 대기 (초) |
| `--pick` / `--place` | pick / place | 사용할 자세 이름 |

---

## 4. `vision_pick_place.py` — 카메라 클릭 픽앤플레이스

사전 조건: RealSense 를 **노트북 USB3 포트에 직결**, 핸드-아이 캘리브레이션 결과와 설정 파일
`~/HamdEyeCal/handeye_config.json` (캘리브 경로 · TCP 길이 · 오프셋) 이 있어야 합니다.
(캘리브레이션 GUI 는 강의 자료로 제공되며 이 레포에는 포함하지 않습니다.)

```bash
python3 vision_pick_place.py                  # 놓을 곳: poses.json 의 place
python3 vision_pick_place.py --place place2
```

1. 카메라 창에서 물체 **클릭 → Space** 확정 (`q` 종료)
2. 준비자세 → 현재 높이를 유지한 채 물체 위로 정렬
3. 터미널 메뉴

| 키 | 동작 |
|---|---|
| `Enter` | 한 스텝(10mm) 내리기 |
| `a` | 물체 윗면 +20mm 까지 한 번에 |
| `g` / `o` | 잡기 / 열기 |
| `u` | 처음 높이로 올리기 |
| `p` | place 로 옮겨 놓기 |
| `r` | 준비자세 → 다음 물체 |
| `q` | 종료 |

---

## 5. 물체 인식 픽앤플레이스 (`detector_node.py` + `object_picker.py`)

YOLO 는 전용 venv 에 설치되어 있습니다 (CPU, numpy 1.x 고정 — ROS 호환):

```bash
python3 -m venv ~/venv/yolo
printf 'numpy<2\nopencv-python<4.12\n' > ~/venv/yolo/constraints.txt
~/venv/yolo/bin/pip install -c ~/venv/yolo/constraints.txt torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/venv/yolo/bin/pip install -c ~/venv/yolo/constraints.txt "numpy<2" ultralytics pyrealsense2
```

**터미널 3 — 검출기** (카메라는 이 프로그램이 사용. 첫 실행 시 `models/yolo11n.pt` 자동 다운로드)

```bash
~/venv/yolo/bin/python detector_node.py --show                 # 검출 창 표시
~/venv/yolo/bin/python detector_node.py --classes cup bottle   # 특정 라벨만
ros2 topic echo /detections                                     # 발행 내용 확인
```

**터미널 4 — 집기**

```bash
python3 object_picker.py --dry                          # 좌표만 출력 (로봇·bringup 불필요)
python3 object_picker.py                                # 라벨 입력 → 확인(y) → 집어서 place 에 놓기
python3 object_picker.py --place place2 --grip-depth 15
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--grip-depth` | 20 | 물체 윗면에서 몇 mm 아래를 잡을지 |
| `--clearance` | 80 | 물체 윗면 위 접근 높이 (mm) |
| `--min-conf` | 0.5 | 이 신뢰도 미만 검출은 무시 |
| `--place` | place | 놓을 자세 이름 (`pick_place_hardcoded.py teach` 로 저장) |
| `--current` / `--grip-wait` | 300 / 1.5 | 잡는 힘 / 그리퍼 후 대기 |

검출기 교체: `/detections` (std_msgs/String, JSON) 형식만 지키면 됩니다 — 형식은 `detector_node.py` 맨 위 주석 참고.
`xyz` 는 **카메라 좌표계 3D (m)**, 로봇 좌표 변환은 `object_picker.py` 가 합니다.

---

## 6. `gripper_repeat.py` — 그리퍼 반복 테스트

```bash
python3 gripper_repeat.py --show                        # 현재 관절각만 출력
python3 gripper_repeat.py                               # 현재 자세에서 5회 반복
python3 gripper_repeat.py --posj 0 0 90 0 90 0 -n 10    # 준비자세로 이동 후 10회
python3 gripper_repeat.py --interval 2 --current 250
```

---

## 안전

- 처음 실행은 **물체 없이, 저속으로** 경로부터 확인
- 비상정지 버튼은 항상 손 닿는 곳에
- 모든 이동 명령은 실행 전 `Enter` 확인을 받습니다 (`--yes` 로 생략 가능)
