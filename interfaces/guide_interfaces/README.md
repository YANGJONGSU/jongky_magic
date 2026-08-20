# guide_interfaces

안내로봇의 계층 간 계약. 지금 문자열로 오가는 것들을 타입으로 못 박은 것이다.

```bash
colcon build --packages-select guide_interfaces
```

## 지금 무엇이 문제인가

오가는 게 전부 문자열이다.

| 지금 | 무엇이 없나 |
|---|---|
| `/guide/destination` — `std_msgs/String` | 수락/거절이 없다. 초기 위치가 없거나 이미 안내 중이면 `guide_node` 는 로그만 남기고 버린다. 던진 `voice_node` 는 갔는지 모른다 |
| `/guide/status` — `std_msgs/String` 에 JSON | 필드가 코드에 없다. 웹 UI 가 `json.parse` 로 풀고, 이름이 바뀌면 조용히 `undefined` 가 된다 |
| `GET /api/destinations` → 이름 배열 | 표시 이름도 종류도 층도 없다. 내부 코드(`10a` `ev1`)가 그대로 버튼에 뜬다 |
| `POST /api/cancel` | 어느 안내를 끊는지 모른다. "지금 뭐든 취소" 다 |

## 만든 것

### `action/Guide.action` — 목적지 안내

**액션이다.** 안내는 수십 초~수 분 걸리고, 중간에 취소되고, 남은 거리가 있다.
목표 수락/거절 + 취소 + 피드백은 액션이 하는 일이고, 위 표의 네 칸 중 세 칸이
이걸로 메워진다. 토픽으로 두면 거절도 취소 대응도 진행률도 계속 없다.

- goal: `string poi_id` — 발화나 표시 이름이 아니라 항상 id. 이름 해석(문자열
  매칭·LLM)은 요청하는 쪽에서 끝낸다
- result: `outcome` + `message`. 종료 다섯 가지(`ARRIVED` `CANCELED`
  `NAV_FAILED` `FOLLOWER_LOST` `ALERT`)는 전부 지금 `_guide_loop` 가 실제로
  도달하는 것들이다. 거절은 결과가 아니라 goal reject 로 처리한다
- feedback: `GuideStatus status` — 필드를 다시 나열하지 않는다. 두 경로가
  갈라져서 "화면에 뜬 것" 과 "요청자가 받은 것" 이 달라지면 안 된다

`/api/go` → goal, `/api/cancel` → 그 goal 의 cancel 로 대응된다.

### `msg/GuideStatus.msg` — 상태 보고

`GuideState.snapshot()` 이 실제로 담는 다섯 개(`status` `destination` `message`
`distance` `follower_m`) 를 그대로 옮기고, `/api/status` 응답에만 얹히던 둘
(`localized` `can_listen`) 을 더했다. 뒤의 둘은 지금 토픽에 안 실려서 웹 UI 만
볼 수 있는데, "초기 위치가 없으면 어떤 목적지도 가지 않는다" 는 조건이라
관제 쪽도 봐야 한다.

`status` 문자열은 `uint8` 상수로 바꿨다. `listening` 은 `GuideState` 주석의
목록에 없지만 `_listen_loop` 가 실제로 넣는 값이라 포함했다.

시각(stamp)은 **일부러 뺐다.** 이 토픽은 상태가 바뀔 때만 나가므로 "5분째 조용함"
이 정상이고, 하트비트 없이 stamp 만 두면 신선도를 판단할 수 있는 것처럼 보이기만
한다.

### `msg/Poi.msg` — POI 스키마

핵심은 **id 와 사람이 읽는 이름의 분리**다.

현장 YAML 의 키는 맵핑하던 사람이 그때그때 친 내부 코드다 — `10a` `ev1` `m1`.
이게 지금 세 군데에 그대로 쓰인다.

1. Nav2 목표를 고르는 식별자 (`guide_node._to_pose`)
2. 터치스크린 버튼 글자 (`web/index.html` 이 `/api/destinations` 를 그대로 그린다)
3. 발화 매칭 대상 (`_resolve` / `_match` 가 "이름이 발화에 통째로 들어 있는가" 로 찾는다)

방문객은 `10a` 라고 말하지 않고, 버튼에 `10a` 가 뜨면 어디인지 모른다.
그래서 좌표를 가리키는 불변 키(`id`)와 사람이 보고 말하는 것(`display_name`,
`aliases`)을 나눈다. 이름을 고쳐도 좌표 참조가 안 깨지는 게 요점이다.

`kind` 는 층 전환 때문에 필요하다. 엘리베이터를 이름 규칙(`ev` 로 시작)으로
찾으면 안 된다 — 실제 파일에 `wwwwwwwwwwwwwwwwwev2` 가 들어 있다. 종류는 다섯
가지(`강의장 / 엘리베이터 / 출입구 / 화장실 / 기타`)만 둔다. 이 이상은 지금
쓰는 데가 없다.

`floor` 도 층 전환용이다. waypoint 파일이 층별로 나뉘어 있어서(`waypoints_10f.yaml`)
항목 자체에는 층이 안 적혀 있는데, 목적지가 다른 층인지 알아야 승강기를 탄다.

### `srv/ListPois.srv`, `srv/SetStart.srv`

`/api/destinations` 와 `/api/start-here` 를 대신한다. 둘 다 즉시 끝나고 취소할
것도 진행률도 없어서 서비스다. `ListPois` 의 요청은 비어 있다 — 안내 노드는
waypoint 파일 하나(= 한 층)를 들고 있고 지금 걸러 낼 축이 없다.

`/api/listen` (PTT) 은 인자도 결과도 없어서 `std_srvs/Trigger` 로 충분하다.
같은 모양의 타입을 새로 만들지 않았다.

### 안 만든 것

`fleet/guide_vda5050` 와 `fleet/guide_rmf` 가 필요로 할 타입은 없다. 그쪽은
아직 README 3줄이고, 무엇을 주고받을지 정해지지 않은 채 만든 타입은 정해질 때
어차피 다시 만든다. 층 전환용 `Floor`/`MapSwitch` 류도 마찬가지다 — 층 전환에
지금 필요한 것은 "목적지가 몇 층인가"(`Poi.floor`)와 "어느 게 승강기인가"
(`Poi.kind`)뿐이고, 둘 다 `Poi.msg` 안에 있다.

## waypoint YAML 스키마와 하위 호환

디스크 포맷은 `schema/waypoints.example.yaml` 에 있다. 최상위는 지금과 똑같이
**이름을 키로 하는 매핑**이다. 리스트로 바꾸면 `self._waypoints[name]` 과
`list(self._waypoints)` 가 전부 깨진다.

새 필드는 전부 선택이다. 없으면 읽는 쪽이 이렇게 메운다.

| 필드 | 없을 때 |
|---|---|
| `display_name` | 키를 그대로 쓴다 |
| `kind` | `etc` (기타) |
| `aliases` | 없음 |
| `floor` | 파일 이름(`waypoints_10f.yaml`)에서 뽑고, 그것도 안 되면 0 |

그래서 오늘 현장에 있는 파일이 한 글자도 안 고치고 그대로 돈다. 정리는 나중에
해도 되고, 하는 동안에도 로봇은 움직인다.

**읽는 쪽은 절대 추론하지 않는다.** 이름에서 종류를 짐작하는 일은 이관 도구
[`robot/jongky_bringup/tools/waypoint_doctor.py`](../../robot/jongky_bringup/tools/waypoint_doctor.py)
가 한 번만 하고, 결과를 파일에 적어 사람이 고칠 수 있게 남긴다. 런타임이 매번
짐작하면 틀린 짐작을 아무도 고칠 수 없다.

```bash
# 검사 + 이관 (원본은 안 건드린다. 결과는 /tmp/waypoint_doctor)
python3 robot/jongky_bringup/tools/waypoint_doctor.py ~/waypoints_10f.yaml ~/waypoints_11f.yaml
```

## 바꿀 때

`docs/collaboration.md` 대로 A/B/C 3인 승인이 필요하다. 특히 `Poi.id` 는 지도
좌표와 사람이 부르는 이름을 잇는 유일한 고리라, 이름을 바꾸는 것보다 id 를 바꾸는
쪽이 훨씬 비싸다.
