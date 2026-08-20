# 현장용 단축 스크립트

7인치 터치스크린 + USB 키보드만으로 맵핑을 돌리기 위한 것들.
**네트워크가 필요 없다** — SLAM·텔레옵·지도 저장이 전부 온보드에서 돈다.

건물 WiFi 는 층마다 서브넷이 갈려 있어서 11층에 올라가면 SSH 가 끊긴다.
그때도 로봇 화면에서 그대로 작업할 수 있어야 한다.

## 현장 나가기 전에 (인터넷 되는 자리에서 한 번)

```bash
./fetch_models.sh          # SSDLite 가중치를 미리 받는다 (약 14MB)
./fetch_models.sh --check  # 받지 않고 있는지만 본다
```

`follow_service.py`(후면 사람 탐지)가 쓰는 torchvision 가중치는 **첫 실행 때
인터넷에서 받는다.** 저장소에도 컨테이너 이미지에도 없다 — 이 서비스는
컨테이너 밖(젯슨 호스트)에서 돌기 때문에 이미지에 굽는다고 해결되지 않는다.

건물 WiFi 는 층마다 서브넷이 갈려 있어 11층에서는 밖으로 못 나간다. 거기서
처음 띄우면 다운로드가 실패하고 서비스가 안 뜬다. 개발실에서 한 번 받아 두면
그 뒤로는 캐시(`~/.cache/torch`)에서 로드한다.

**`follow_service.py` 를 돌릴 그 계정으로 받을 것.** 캐시가 홈 밑이라
`sudo` 로 받으면 `/root` 에 들어가고, 정작 서비스는 못 찾는다.

가중치가 없으면 `follow_service.py` 는 **네트워크를 건드리기 전에 무엇을
해야 하는지 찍고 죽는다.** 받으러 나가게 두면 막힌 망에서 응답 없이 매달릴
수 있고(`torch.hub` 다운로드에는 타임아웃이 없다), 그러면 원인이 안 보인다.

## 설치 (젯슨에서 한 번)

```bash
cd ~/jongky_ws/src/jongky_bringup/scripts
./install.sh
```

`~/bin` 에 심링크를 걸고 `.bashrc` 에 PATH 를 추가한다.

## 쓰는 법

터미널 세 개를 띄운다 (터치스크린에서 탭 또는 `Ctrl+Alt+F2` 같은 가상 콘솔).

```bash
jmap 10f      # 1) SLAM + rosbag 기동. 층 이름을 준다
jdrive        # 2) 텔레옵. w 로 강의장 앞 waypoint 저장
jsave 10f     # 3) 지도 저장
```

`jmap` 이 뜬 뒤에 `jdrive` 를 띄운다. SLAM 이 먼저 올라와야 `w` 가 동작한다
(`map -> base_footprint` TF 를 읽기 때문).

## 확인

```bash
jcheck        # /scan /odom /map TF 가 다 나오는지 한 번에 본다
```

맵핑 시작 전에 이걸로 먼저 보는 게 좋다. 여기서 `/map` 이 없으면
주행해 봐야 지도가 안 쌓인다.

안내 주행(`jongky_guide`)까지 볼 거면 나가기 전에 하나 더:

```bash
./fetch_models.sh --check   # 탐지 가중치가 캐시에 있나
```
