# 안내 대상 지점 하나. teleop_key.py 가 찍어 둔 waypoint YAML 의 한 항목과 1:1 이다.
#
# [왜 id 와 display_name 을 나누는가]
# 현장 YAML 의 키는 맵핑하던 사람이 그때그때 친 내부 코드다 — '10a' 'ev1' 'm1'.
# 지금 이 키가 세 군데에 그대로 쓰인다.
#   1. Nav2 목표를 고르는 식별자 (guide_node._to_pose)
#   2. 터치스크린 버튼 글자     (web/index.html 이 /api/destinations 를 그대로 그린다)
#   3. 발화 매칭 대상           (guide_node._resolve / voice_node._match 가
#      "이름이 발화 안에 통째로 들어 있는가" 로 찾는다)
# 방문객은 '10a' 라고 말하지 않고, 버튼에 '10a' 가 뜨면 어디인지 모른다.
# 그래서 좌표를 가리키는 불변 키(id)와 사람이 보고 말하는 이름(display_name,
# aliases)을 분리한다. 이름을 고쳐도 좌표 참조가 깨지지 않는 것이 요점이다.

# YAML 키 그대로. 안내 요청·상태 보고가 가리키는 실제 식별자다. 비면 안 된다.
string id

# 화면 버튼과 TTS 에 쓰는 이름. **비어 있으면 받는 쪽이 id 를 쓴다.**
# 새 필드가 없는 옛 YAML 이 그대로 도는 이유가 이 규칙이다.
string display_name

# 발화 매칭용 다른 표현 ('에이강의장', '십층 에이'). 비어도 된다.
# 지금의 부분문자열 매칭은 방문객이 waypoint 이름을 글자 그대로 말해야
# 동작한다. 매칭 알고리즘을 바꾸지 않고도 말이 통하게 하는 최소 장치다.
string[] aliases

# 종류. 층 전환이 엘리베이터를 알아보려면 이름 규칙('ev' 로 시작)이 아니라
# 이 값을 봐야 한다 — 이름은 오염되고(실제 파일에 'wwwwwwwwwwwwwwwwwev2' 가
# 있다) 규칙은 층마다 사람마다 달라진다.
uint8 KIND_ETC=0        # 기타. 종류를 정하지 못한 것도 여기로 떨어진다
uint8 KIND_LECTURE=1    # 강의장
uint8 KIND_ELEVATOR=2   # 엘리베이터 — 층 전환의 진입/이탈 지점
uint8 KIND_ENTRANCE=3   # 출입구
uint8 KIND_RESTROOM=4   # 화장실
uint8 kind

# 층. 0 = 모름. waypoint 파일이 층마다 나뉘어 있어서(waypoints_10f.yaml)
# 항목 자체에는 층이 안 적혀 있다. 층 전환은 목적지가 다른 층인지 알아야 한다.
int32 floor

# YAML 의 frame_id / position / orientation 이 그대로 들어온다.
# header.frame_id 는 'map' 이어야 한다 — 다른 값이면 Nav2 가 변환하지 못한다.
geometry_msgs/PoseStamped pose
