# /guide/status 의 타입.
#
# 지금은 std_msgs/String 에 GuideState.snapshot() 을 JSON 으로 말아 보내고
# 받는 쪽이 파싱한다. 필드는 snapshot() 이 실제로 담는 다섯 개
# (status/destination/message/distance/follower_m) 에, HTTP /api/status 가
# 응답에만 얹어 주는 둘(localized/can_listen)을 더한 것이다. 뒤의 둘은 토픽에
# 안 실려서 지금은 웹 UI 만 볼 수 있는데, "초기 위치가 없으면 어떤 목적지도
# 가지 않는다"는 조건이라 관제 쪽도 봐야 한다.
#
# 시각(stamp)은 일부러 안 넣었다. 이 토픽은 상태가 바뀔 때만 나가므로
# "5분째 조용함" 이 정상이다. 하트비트가 없는 채로 stamp 만 두면 신선도를
# 판단할 수 있는 것처럼 보이기만 한다.

# status 문자열을 그대로 옮긴 것이다. 'listening' 은 GuideState 주석의 목록에는
# 없지만 _listen_loop 가 실제로 넣는 값이라 포함했다.
uint8 STATE_IDLE=0
uint8 STATE_NAVIGATING=1
uint8 STATE_WAITING=2      # 뒤처진 사람을 기다리는 중 (Nav2 는 취소된 상태)
uint8 STATE_ARRIVED=3
uint8 STATE_FAILED=4
uint8 STATE_ALERT=5        # VLM 이 위급으로 판단해 세웠다
uint8 STATE_LISTENING=6    # PTT 로 한 마디 듣는 중
uint8 state

# 지금 안내 중인 목적지의 Poi.id. 안내 중이 아니면 빈 문자열.
string destination

# 화면에 그대로 찍고 TTS 로 읽는 한국어 문장. 열거할 수 있는 값이 아니라
# 자유 문자열로 둔다 — 상태(state)와 달리 문면은 계속 바뀐다.
string message

# 남은 거리. Nav2 피드백의 distance_remaining 이고 안내 중이 아니면 0.
float32 distance_m

# 뒤따라오는 사람까지 거리. -1 = 모름 (탐지기가 없거나 죽었다).
float32 follower_m

# AMCL 초기 위치가 잡혔는가. false 면 어떤 안내 요청도 거절된다.
bool localized

# 마이크와 STT 가 준비됐는가. false 면 PTT 버튼을 눌러도 소용없다.
bool can_listen
