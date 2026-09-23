# 버거 로봇 주문판 (front)

햄버거 재료를 집어 쌓는 로봇 프로젝트용 웹 대시보드입니다.
게임 **Overcooked**의 HUD처럼 좌상단에 주문 티켓이 줄지어 나타나고, 로봇이 쌓기를 끝내면(지금은 데모용 **완료** 버튼) 티켓이 사라집니다. 우측 패널에서 어떤 버거를 만들지 고르거나 직접 조립해 주문을 넣습니다.

빌드 도구 없이 순수 HTML / CSS / JS(ES module)로 되어 있어 정적 서버만 있으면 바로 실행됩니다.

## 화면 구성

| 영역 | 내용 |
|---|---|
| 상단 헤더 | 대기 / 조리 중 / 완료 개수, 현재 시각 |
| 좌상단 **주문 티켓 밴드** | 주문마다 카드 1장: 타임바, 티켓 번호, 경과 시간, 완성 버거 썸네일, 재료 아이콘(아래→위 순), 상태, **완료 / 취소** 버튼. 많아지면 가로 스크롤 |
| 가운데 **지금 만들 버거** | 로봇이 다음에 만들 주문(조리 중 우선, 없으면 가장 오래된 대기 주문)의 큰 미리보기와 쌓는 순서, **조리 시작 / 완료** 버튼 |
| 우측 **주문 만들기** | 프리셋 레시피 6종(카드 클릭 → 조립판에 불러오기, **+** → 바로 주문), 커스텀 조립(패티/치즈/토마토 버튼, 되돌리기, 비우기, 주문 추가) |

타임바 색: 초록(여유) → 노랑(남은 시간 50% 이하) → 빨강(20% 이하, 카드가 한 번 흔들림). 기본 제한 시간은 3분이며 `js/data.js`의 `LIMITS`에서 바꿉니다.

## 실행

레포 루트에서:

```bash
cd front && python3 -m http.server 8000
```

브라우저에서 <http://localhost:8000> 을 엽니다.
`file://` 로 직접 열면 ES 모듈이 차단되므로 반드시 HTTP 서버를 사용하세요.

## 재료 이미지 준비

`assets/ingredients/` 에는 배경을 제거해 투명 PNG로 만든 재료 5장(하단 번, 패티, 치즈, 토마토, 상단 번)이 이미 들어 있습니다.
원본(흰 배경 1536×1024 PNG)을 바꾸거나 다시 만들려면:

```bash
python3 scripts/prepare_assets.py --src "/path/to/burger images" --out assets/ingredients --width 320 --preview
```

- 필요 패키지: Pillow, numpy, scipy
- 흰 배경과 회색 접촉 그림자를 제거하고, 이미지 테두리에 닿은 영역만 배경으로 취급해 빵 속살 같은 밝은 부분은 보존합니다.
- 하단 번의 물체 폭이 `--width`가 되도록 모든 재료를 같은 비율로 축소합니다(상대 크기 유지).
- `--preview` 를 주면 `assets/ingredients/_preview_stack.png` 에 쌓은 모습을 합성해 줍니다.
- 실행 후 출력되는 표의 `scale`, `h/w` 값을 `js/data.js` 의 `INGREDIENTS` 에 반영하세요. (`height = scale × h/w`)
- 원본 파일명은 `bottom_bun.png, patty.png, cheese.png, tomato.png, upper_bun.png` 이어야 하며 `upper_bun.png` 는 `top_bun.png` 로 저장됩니다.

원본 사진(약 8 MB)은 저장소에 넣지 않습니다. `assets/raw/` 는 `.gitignore` 되어 있으니 원하면 거기에 복사해 두세요.

## 로봇 연동 API

페이지가 뜨면 `window.BurgerUI` 가 노출됩니다. 티켓의 **완료** 버튼과 외부 API 호출은 **같은 함수**를 타므로, 나중에 로봇 서버가 생기면 그 신호로 `completeOrder` 만 불러 주면 됩니다.

| 함수 | 설명 |
|---|---|
| `addOrder(layers, name?)` | 주문 추가. `layers` 는 아래→위 순 재료 id 배열. 번은 없으면 자동으로 붙음. 주문 객체 반환 |
| `completeOrder(id)` | 로봇이 쌓기를 끝냈을 때. 티켓이 완료 애니메이션과 함께 사라짐. `boolean` 반환 |
| `cancelOrder(id)` | 주문 취소 |
| `setOrderStatus(id, 'in_progress' \| 'queued')` | 조리 중 표시 켜기/끄기 |
| `getOrders()` | 현재 대기·조리 중 주문 배열(복사본) |
| `onChange((state, event) => {})` | 상태 변경 구독. 해제 함수 반환. `event.type` 은 `add` / `complete` / `cancel` / `status` / `builder` |
| `INGREDIENTS`, `RECIPES`, `LIMITS`, `makeLayers` | 데이터 참조 |

재료 id: `bottom_bun`, `patty`, `cheese`, `tomato`, `top_bun`
각 재료의 `yolo` 필드는 로봇 트래커의 클래스 이름(`Bottom Bun`, `Patty`, `Cheese`, `Tomato`, `Bun Top`)입니다.

주문 객체:

```js
{
  id: 'k3f9-1',           // 세션 접두어 + 순번 (새로고침해도 겹치지 않음)
  seq: 1,                 // 티켓에 표시되는 번호
  name: '치즈버거',
  layers: ['bottom_bun', 'patty', 'cheese', 'top_bun'],   // 아래 → 위
  createdAt: 1727054400000,
  status: 'queued',       // 'queued' | 'in_progress' | 'done' | 'cancelled'
  completedAt: null
}
```

콘솔에서 바로 테스트:

```js
BurgerUI.addOrder(['patty', 'cheese']);                    // 치즈버거 주문
BurgerUI.setOrderStatus(BurgerUI.getOrders()[0].id, 'in_progress');
BurgerUI.completeOrder(BurgerUI.getOrders()[0].id);         // 완료 버튼과 동일
```

### 연동 훅 위치

`js/bridge.js` 의 `connectRobot()` 이 스텁으로 있습니다. WebSocket 이나 fetch 를 여기서 열고, 받은 메시지를 `bridge.handleMessage(msg)` 로 넘기면 됩니다. UI 코드는 손댈 필요 없습니다.

제안 메시지 형식:

| 방향 | 메시지 |
|---|---|
| UI → 로봇 | `{ type: 'order_added', order, yolo: ['Bottom Bun', 'Patty', ...] }` |
| UI → 로봇 | `{ type: 'order_cancelled', id }` |
| 로봇 → UI | `{ type: 'order_status', id, status: 'in_progress' }` |
| 로봇 → UI | `{ type: 'order_done', id }` |

```js
import { connectRobot } from './bridge.js';
const ws = new WebSocket('ws://robot:8765');
const bridge = connectRobot({ send: (msg) => ws.readyState === 1 && ws.send(JSON.stringify(msg)) });
ws.onmessage = (e) => bridge.handleMessage(JSON.parse(e.data));
```

## 데이터 수정

`js/data.js` 한 곳에서 관리합니다.

- **재료 추가**: `INGREDIENTS` 에 항목 추가 + `MIDDLE_IDS` 에 id 추가 + `assets/ingredients/<id>.png` 준비.
  - `scale`: 스택 폭 대비 이미지 폭, `height`: `scale × (이미지 h/w)`, `thickness`: 그 위에 다음 층을 얼마나 띄울지(스택 폭 비율). 미리보기가 어색하면 `thickness` 를 조절하세요.
- **레시피 추가**: `RECIPES` 에 `{ id, name, middle: [...] }` 추가. 번은 자동으로 붙습니다.
- **제한값**: `LIMITS` — 중간 재료 최대 수(5), 대기열 최대(12), 타임바 제한 시간(3분), 노랑/빨강 전환 비율.

## 폴더 구조

```
index.html               페이지 뼈대
css/styles.css           스타일 (토큰 / 레이아웃 / 스택 합성 / 티켓 / 주문 만들기 / 반응형)
js/data.js               재료·레시피·제한값
js/store.js              상태 저장소 (DOM 없음). 모든 상태 변경은 여기서만
js/stack.js              재료 이미지를 겹쳐 버거 미리보기 합성
js/tickets.js            주문 티켓 렌더러 (등장/퇴장 애니메이션, 타이머)
js/composer.js           주문 만들기 패널
js/bridge.js             window.BurgerUI 공개 API + 로봇 연동 훅
js/main.js               부트스트랩, 헤더 통계, "지금 만들 버거" 카드
assets/ingredients/      투명 배경 재료 PNG
scripts/prepare_assets.py 원본 사진 → 투명 PNG 변환
```

## 향후 계획

- `connectRobot()` 에 실제 WebSocket / REST 어댑터 연결
- 가운데 영역에 로봇 카메라 뷰 패널
- 새로고침 시 주문 상태 유지 (localStorage 또는 서버)
