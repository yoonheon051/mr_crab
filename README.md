# mr_crab (OpenManipulator-X)

OpenManipulator-X를 ROS 2 Jazzy에서 켜고, 끄고, 움직이는 스크립트 모음입니다.

## 파일

| 파일 | 설명 |
|---|---|
| `fastdds_shm.xml` | FastDDS 공유메모리 세그먼트를 키운 설정 (env.sh가 적용). 카메라 영상 프레임 유실 방지 |
| `env.sh` | ROS 2 환경 설정 (Jazzy, FastDDS, 로컬호스트 디스커버리, 포트 `/dev/ttyACM0`). `.bashrc`는 건드리지 않음 |
| `bringup.sh` | 하드웨어 bringup. 종료 시 홈 자세로 이동한 뒤 종료 |
| `dashboard.py` / `dashboard.html` / `dashboard_rc.sh` | 웹 대시보드(관절 각도 + 카메라 영상 + 터미널). bringup이 자동으로 띄움 |
| `hand_follow.sh` / `hand_follow.py` | 손 따라가기: 손바닥이 카메라 중앙에 오도록 관절을 움직이고, 주먹이면 그리퍼를 닫음 |
| `hand_teleop.sh` / `hand_teleop.py` | 손 따라 움직이기(참참참): **고정된 카메라**로 손의 좌우/상하 위치를 보고 로봇팔이 같이 좌우/상하로 움직임 |
| `teleop_pose.json` | 텔레오퍼레이션 기준 자세(손이 화면 가운데일 때). 위아래 ±13cm 범위가 대칭이 되도록 고른 자세 |
| `stop.sh` | 다른 터미널에서 bringup을 안전하게 종료 |
| `go_home.py` | 팔 4축을 `home.json` 자세로 이동 (그리퍼는 그대로) |
| `home.json` | 홈 자세 (joint1~4, rad) |
| `ready.json` | 브링업 자세 (bringup이 켜질 때 팔이 가는 자세). 손 따라가기의 검지 제스처 목표 |
| `dance.py` | 춤 동작 실행 후 홈 자세로 복귀 |
| `moveit.sh` | MoveIt 실행 (bringup이 켜진 상태에서) |
| `gripper_reboot.sh` | 그리퍼 모터 재부팅 |

## 실행하는 법

1. 로봇 전원을 켜고 USB로 연결해 `/dev/ttyACM0`가 보이는지 확인합니다.
   ```bash
   ls /dev/ttyACM0
   ```
2. bringup을 실행합니다. 이 터미널은 계속 열어 둡니다.
   ```bash
   cd ~/mr_crab_ws
   ./bringup.sh
   ```
   웹캠(C270) 노드(usb_cam)도 같이 뜨고 영상이 `/camera1/image_raw` 토픽으로 나가며, bringup을 종료하면 함께 종료됩니다. 화면으로 보려면 `ros2 run rqt_image_view rqt_image_view`(설치 필요) 또는 RViz를 씁니다. 끄려면 `OMX_CAM=0 ./bringup.sh`, 다른 카메라는 `OMX_CAM=/dev/videoN ./bringup.sh`로 실행합니다.
   bringup이 웹 대시보드도 띄워 브라우저 창(Chrome 앱 창)이 자동으로 열립니다. 위쪽에 관절 각도(°/rad, 조작 가능)와 카메라 영상, 아래쪽에 `~/mr_crab_ws`에서 시작하는 터미널이 있고, 터미널에서는 `ros2` 명령이 바로 됩니다. 창을 닫았다면 bringup 터미널에 찍힌 `[bringup] 대시보드:` URL을 다시 열면 됩니다(토큰이 포함된 URL이며 bringup을 다시 켜면 바뀝니다). 서버는 `127.0.0.1`에만 열리고, 토큰과 Origin이 맞지 않으면 터미널 접속을 거부합니다. 끄려면 `OMX_DASH=0 ./bringup.sh`, 포트는 `OMX_DASH_PORT`(기본 8765)로 바꿉니다. 로그는 `/tmp/mr_crab_dashboard.log`입니다. 화면은 인터넷에서 xterm.js(jsdelivr CDN)를 불러오므로 오프라인이면 터미널이 표시되지 않습니다.
   **대시보드에서 관절 조작**: 기본은 잠겨 있고 "제어 활성화"를 체크하면 슬라이더(또는 숫자 입력 후 Enter)로 각도를 바꿀 수 있습니다. 파란 선은 실제 위치, 주황 손잡이는 목표이며 손을 떼면 명령이 나갑니다. 관절은 URDF 한계로 제한되고 속도는 최대 0.6 rad/s(최소 1초)입니다. "홈 자세"는 `home.json`으로, "정지"는 제어가 꺼져 있어도 현재 위치에서 멈춥니다. bringup 종료 중 홈 이동 동안에는 제어가 잠깁니다. 테이블·물체와의 충돌은 막아 주지 않으니 팔 주변을 비우고 사용하세요.
   포트가 다르면 `OMX_PORT=/dev/ttyACM1 ./bringup.sh` 처럼 지정합니다.
   `dialout` 그룹이 셸에 적용되지 않았으면 스크립트가 알아서 `sg`로 감싸 실행합니다.
3. 다른 터미널에서 동작을 실행합니다.
   ```bash
   cd ~/mr_crab_ws
   source env.sh
   python3 go_home.py   # 홈 자세로 이동
   python3 dance.py     # 춤 (약 22초, 끝나면 홈으로 복귀)
   ```

## 끄는 법

- **bringup 터미널에서 `Ctrl+C`**: 홈 자세로 천천히 이동한 뒤 종료합니다.
- **다른 터미널에서**:
  ```bash
  ~/mr_crab_ws/stop.sh
  ```
  똑같이 홈 이동 후 종료하고, 끝나면 터미널 벨과 데스크톱 알림으로 "이제 전원을 꺼도 됩니다"라고 알려 줍니다. 홈 이동이 실패했거나 60초 안에 끝나지 않으면 경고를 띄웁니다.
- **홈 이동 중에 `Ctrl+C`를 한 번 더**: 이동을 중단하고 즉시 종료합니다.
- `kill -9`이나 전원 차단은 홈 이동을 처리할 수 없습니다.

## 주의

- 동작을 실행하기 전에 팔 주변을 비우세요. 홈 자세는 접힌 자세이고, 동작 중에는 팔이 크게 회전합니다.
- `dance.py`의 속도는 `MAX_SPEED`(기본 1.2 rad/s)로, 동작은 `STEPS`로 바꿀 수 있습니다. 관절 한계는 joint2 ±1.5, joint3 -1.5~1.4, joint4 -1.7~1.97 rad입니다.
- `stop.sh`는 `mr_crab_ws/bringup.sh` 경로로 실행된 프로세스만 찾습니다. 예전 경로(`open_manipulator_ws`)로 띄운 bringup은 `pkill -INT -f "ros2 launch open_manipulator_bringup"`으로 종료하세요.

## 카메라 영상이 끊길 때

- 증상: `/camera1/image_raw`가 1~10fps로 들쭉날쭉하고 수 초씩 멈춤. 장치를 직접 읽으면(`v4l2-ctl --stream-mmap`) 30fps인데 ROS 토픽만 끊기면 이 문제입니다.
- 원인: 프레임(640x480 RGB ≈ 0.9MB)이 FastDDS 기본 공유메모리 세그먼트(≈0.5MB)보다 커서 UDP 조각 전송으로 밀려 유실됨. `env.sh`가 `fastdds_shm.xml`(세그먼트 128MB)을 적용해 해결합니다.
- 반드시 `env.sh`를 source한 셸에서 실행해야 적용됩니다(`bringup.sh`/`stop.sh`/대시보드 터미널은 자동). 해상도를 크게 올리면 세그먼트(`segment_size`)도 함께 키우세요.
- 확인: `source env.sh; ros2 topic hz /camera1/image_raw` (30fps 근처여야 정상). USB 허브 뒤에 꽂으면 카메라가 리셋되기도 하니 본체 포트에 직접 연결하세요.

## 손 따라가기 (hand_follow)

bringup이 켜진 상태에서 대시보드 터미널(또는 다른 터미널)에서 실행합니다.
```bash
cd ~/mr_crab_ws
./hand_follow.sh --dry-run     # 1) 로봇은 안 움직이고 인식만 확인 (대시보드에 손 뼈대와 중앙 십자가 표시됨)
./hand_follow.sh               # 2) 실제로 팔 제어. 종료는 Ctrl+C (팔은 그 자세 그대로, 홈으로 가지 않음)
```
- 손바닥(손목+손가락 뿌리 5점의 평균)이 화면 중앙에 오도록 **joint1(좌우)**, **joint4(상하 기울기)**를 비례제어합니다. 카메라가 팔 끝에 달려 있다고 가정합니다.
- **제스처**(손가락 접힘 기준, 서로 겹치지 않음. 터미널에 `[hand] …`로 알려 주고 화면에 상태가 표시됨)
  - **주먹**(네 손가락 모두 접힘) → 그리퍼 닫기 / **손 펴기**(접힌 손가락 1개 이하) → 그리퍼 열기. 4프레임 연속일 때만 바뀝니다(`--confirm`).
  - **정지**: 중지·약지를 접고 새끼는 펴면(검지는 상관없음) 팔이 그 자리에서 멈추고(`HOLD`), 풀면 다시 따라갑니다. 그리퍼는 그대로입니다.
  - **검지(브링업 자세)**: 검지만 펴고 나머지 셋을 접으면 **브링업 자세**(`ready.json` = bringup이 켜질 때 팔이 가는 자세 `[0, -1.0, 1.0, 0]`, 팔이 펴져 있어 추적을 이어가기 좋음)로 0.35 rad/s로 천천히 이동합니다(`--home-speed`). 접힌 홈 자세로 보내려면 `--pose-file home.json`, 다른 자세는 JSON 파일을 만들어 지정합니다. 큰 동작이라 10프레임 연속일 때만 발동하고(`--home-confirm`), 이동 중에는 추적을 멈추며, 중지·약지를 접으면(정지 제스처) 중단합니다. 도착하면 `자세 도착`이 찍히고, 검지를 풀 때까지 대기하다가 풀면 그 자세에서 다시 추적합니다. 계속 검지를 들고 있어도 다시 출발하지 않습니다. **Ctrl+C로 종료해도 이미 시작된 이동은 계속됩니다.** 충돌 감지는 없으니 팔이 움직이는 경로를 비워 두세요.
- **주먹**(네 손가락 모두 접힘, 4프레임 연속) → 그리퍼 닫기, **손 펴기**(접힌 손가락 1개 이하) → 열기. 손이 안 보이면 그리퍼와 팔 모두 그 상태로 멈춥니다.
- **처음에는 반드시 팔 주변을 비우고 `--dry-run` 후, 손을 천천히 움직여 방향을 확인하세요.** 손을 화면 오른쪽/아래로 옮겼을 때 팔이 반대로 돌면 `--pan-sign 1`(기본 -1), `--tilt-sign -1`(기본 1)로 뒤집습니다.
- 주요 옵션: `--gain`(기본 0.8, 클수록 빠름/흔들림), `--max-rate`(0.4 rad/s), `--deadband`(0.06), `--pan-range`(±1.5 rad), `--tilt-min/--tilt-max`(-1.0~1.5), `--no-gripper`, `--confirm`(4). 전체는 `./hand_follow.sh --help`.
- **앞뒤 이동**: 손이 화면에서 차지하는 크기(손바닥 대각선/화면 높이)를 거리 대용으로 씁니다. 시작할 때 손 크기(15프레임 평균)를 기준으로 삼고, 손이 작아지면(멀어지면) 다가가고 커지면(가까워지면) 물러납니다. joint2·3을 평면 역기구학으로 풀어 카메라가 시선 방향으로 평행 이동하고 joint4가 기울기를 보상합니다. 시작하면 먼저 손을 원하는 거리에 두세요(기준이 정해지면 `[hand] 기준 손 크기 …` 출력). 반대로 움직이면 `--depth-sign -1`, 끄려면 `--no-depth`, 기준값을 직접 주려면 `--ref-size`. 안전장치: 허용 오차 18%, 최대 0.06m/s, 한 번의 추적에서 최대 ±0.12m, 끝점·joint4 최소 높이 0.05m, 관절 한계 도달 시 그 방향 이동 중단(화면에 `limit: …` 표시). 손을 돌리거나 기울이면 크기가 변해 팔이 조금 움직일 수 있습니다.
- 인식 결과 영상은 `/camera1/image_hand`로 나가고, 대시보드는 이 영상이 오면 자동으로 그것을 보여 줍니다(멈추면 1초 뒤 원래 영상). 대시보드의 슬라이더로 동시에 조작하면 서로 충돌하니 하나만 쓰세요.
- 설치(이미 되어 있음): `pip install --target hand_libs --no-deps mediapipe`, 모델은 `models/hand_landmarker.task` (https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task). 시스템 파이썬은 건드리지 않고 `hand_libs/`에만 설치합니다.

### 손 따라가기에서 로봇이 떨릴 때

- 원인 가르기: 아래를 하나씩 켜 보며 떨림이 사라지는 것을 찾으세요.
  `./hand_follow.sh --no-depth`(앞뒤 이동 끔), `--no-gripper`(그리퍼 끔), `--dry-run`(로봇 명령 끔 — 이때도 떨리면 로봇이 아니라 영상/인식 문제).
- 이미 적용된 완화책: 손 위치·크기에 One Euro 필터(`--smooth`, 기본 1.0Hz, 작을수록 부드럽고 반응은 느림), 속도 명령 가속도 제한(`--accel`, `--depth-accel`), 목표가 실제로 바뀔 때만 명령 전송(`--pub-eps`, `--pub-rate`, `--horizon`; 실측: 10Hz+horizon 0.4s는 명령→실제 지연 650ms에 속도가 톱니 모양(잔물결 57%)이었고, 30Hz+0.05s는 지연 135ms에 잔물결 29%(엔코더 양자화 바닥 ≈18%); 같은 목표를 반복 전송하면 컨트롤러가 매번 측정 잡음에서 궤적을 다시 계산해 제자리에서 떨 수 있음), 앞뒤 허용 오차 18%.
- 그래도 떨리면 20초쯤 `./hand_follow.sh --log /tmp/hand.csv`로 기록한 뒤 `/tmp/hand.csv`를 확인하세요(원시/필터된 손 위치·크기, 속도 명령, 목표·현재 관절값, 명령 전송 여부). 5초마다 터미널에 `명령 N회/5s`도 나옵니다.

## 손 따라 움직이기 (hand_teleop, 카메라 고정)

`hand_follow`(카메라가 팔에 달려 손을 화면 중앙에 맞추는 방식)와 달리, 카메라는 **고정**이고 손의 화면 위치를 팔 자세에 직접 대응시킵니다. 카메라 지연이 제어 루프에 들어가지 않아 진동 위험이 작습니다.
```bash
# RealSense 컬러 영상 켜기 (깊이는 아직 쓰지 않으므로 끔). 토픽 이름은 ros2 topic list로 확인
ros2 launch realsense2_camera rs_launch.py enable_depth:=false rgb_camera.color_profile:=640x480x30
./hand_teleop.sh --dry-run                       # 1) 인식만 확인 (기본 토픽 /camera/camera/color/image_raw)
./hand_teleop.sh                                 # 2) 실제 제어. C270으로 시험하려면 --topic /camera1/image_raw
```
- 손이 화면 **가운데**면 팔은 기준 자세(`teleop_pose.json`), 손이 **좌우**로 가면 joint1이 회전(화면 끝에서 ±0.8rad, `--yaw-range`), **위아래**로 가면 끝점 높이가 변합니다(화면 끝에서 ±8cm, `--z-range`). 높이는 평면 역기구학으로 joint2·3을 풀고 joint4가 그리퍼 피치를 그대로 유지합니다. 모든 관절이 같은 비율로 동시에 움직이고(속도 `--max-speed` 0.5rad/s), 관절 한계·최소 높이(`--min-height`)를 넘으면 그 범위 안에서 멈춥니다.
- **방향이 반대면**: 손을 오른쪽으로 옮겼을 때 팔이 반대로 가면 `--pan-sign 1`(기본 -1, 카메라가 로봇과 같은 방향을 볼 때), 손을 올렸는데 팔이 내려가면 `--z-sign -1`. 카메라가 로봇을 마주 보는 거울 배치면 `--pan-sign 1`일 가능성이 큽니다.
- 가운데 위치는 `--neutral-x/--neutral-y`, 가운데 허용 범위는 `--deadzone`, 반응성은 `--smooth`(클수록 빠름)로 조절합니다.
- 제스처는 hand_follow와 같습니다: 주먹=그리퍼 닫기, 폄=열기, 중지·약지 접고 새끼 폄=그 자리 정지(풀면 부드럽게 다시 따라감), 검지만 폄=기준 자세로 이동.
- 처음 손을 보면 팔은 현재 자세에서 매핑된 자세로 부드럽게 이동합니다. 손을 가운데 두고 시작하면 움직임이 가장 작습니다. 팔이 움직이는 범위를 비우고, 처음에는 `--dry-run`으로 확인하세요.
