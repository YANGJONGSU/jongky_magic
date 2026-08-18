# 현장용 단축 스크립트

7인치 터치스크린 + USB 키보드만으로 맵핑을 돌리기 위한 것들.
**네트워크가 필요 없다** — SLAM·텔레옵·지도 저장이 전부 온보드에서 돈다.

건물 WiFi 는 층마다 서브넷이 갈려 있어서 11층에 올라가면 SSH 가 끊긴다.
그때도 로봇 화면에서 그대로 작업할 수 있어야 한다.

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
