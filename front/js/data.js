// 재료·레시피·제한값 정의. 스택 합성 수치는 scripts/prepare_assets.py 출력 표를 기준으로 한다.
//  scale     : 스택 폭(W) 대비 이미지 폭 비율
//  height    : 렌더된 이미지 높이 / W  (= scale × 이미지 h/w)
//  thickness : 이 층 위에 다음 층을 얼마나 띄울지 (W 비율). 상단 번은 위에 아무것도 없으므로 전체 높이.
//  yolo      : 로봇 트래커(YOLO)의 클래스 이름 — 로봇 연동 메시지에 사용
export const INGREDIENTS = {
  bottom_bun: { id: 'bottom_bun', ko: '하단 번', en: 'Bottom Bun', yolo: 'Bottom Bun', img: 'assets/ingredients/bottom_bun.png', scale: 1.00, height: 0.705, thickness: 0.16,  color: '#E8A94E' },
  patty:      { id: 'patty',      ko: '패티',    en: 'Patty',      yolo: 'Patty',      img: 'assets/ingredients/patty.png',      scale: 1.04, height: 0.659, thickness: 0.085, color: '#7A3F1D' },
  cheese:     { id: 'cheese',     ko: '치즈',    en: 'Cheese',     yolo: 'Cheese',     img: 'assets/ingredients/cheese.png',     scale: 1.10, height: 0.683, thickness: 0.03,  color: '#F5B301' },
  tomato:     { id: 'tomato',     ko: '토마토',  en: 'Tomato',     yolo: 'Tomato',     img: 'assets/ingredients/tomato.png',     scale: 1.04, height: 0.783, thickness: 0.075, color: '#E2452B' },
  top_bun:    { id: 'top_bun',    ko: '상단 번', en: 'Top Bun',    yolo: 'Bun Top',    img: 'assets/ingredients/top_bun.png',    scale: 1.00, height: 0.755, thickness: 0.755, color: '#D9822B' },
};

export const MIDDLE_IDS = ['patty', 'cheese', 'tomato'];

export const LIMITS = {
  MAX_MIDDLE: 5,                 // 번 사이에 넣을 수 있는 최대 층 수
  MAX_QUEUE: 12,                 // 동시에 대기 가능한 주문 수
  TICKET_TIME_LIMIT_MS: 180_000, // 티켓 타임바가 0이 되는 시간
  WARN_RATIO: 0.5,               // 남은 비율이 이 이하이면 노랑
  LATE_RATIO: 0.2,               // 남은 비율이 이 이하이면 빨강 + 흔들림
};

// 프리셋 레시피 (중간 층만 정의, 번은 자동으로 붙는다)
export const RECIPES = [
  { id: 'basic',         name: '기본 버거',        middle: ['patty'] },
  { id: 'cheese',        name: '치즈버거',         middle: ['patty', 'cheese'] },
  { id: 'tomato_cheese', name: '토마토 치즈버거',  middle: ['patty', 'cheese', 'tomato'] },
  { id: 'double_patty',  name: '더블 패티',        middle: ['patty', 'patty'] },
  { id: 'double_cheese', name: '더블 치즈버거',    middle: ['patty', 'cheese', 'patty', 'cheese'] },
  { id: 'big_stack',     name: '빅 스택',          middle: ['patty', 'cheese', 'tomato', 'patty', 'cheese'] },
];

/** 중간 층 배열 → 완전한 층 배열 (bottom → top) */
export function makeLayers(middle = []) {
  return ['bottom_bun', ...middle, 'top_bun'];
}

/** 층 배열에서 번을 제외한 중간 층만 */
export function middleOf(layers) {
  return layers.filter((id) => id !== 'bottom_bun' && id !== 'top_bun');
}

/** 층 배열에 맞는 이름: 프리셋과 일치하면 프리셋 이름, 아니면 "패티×2 · 치즈×1" */
export function nameFor(layers) {
  const middle = middleOf(layers);
  const preset = RECIPES.find(
    (r) => r.middle.length === middle.length && r.middle.every((id, i) => id === middle[i]),
  );
  if (preset) return preset.name;
  if (middle.length === 0) return '빈 버거';
  const counts = new Map();
  for (const id of middle) counts.set(id, (counts.get(id) || 0) + 1);
  return [...counts].map(([id, n]) => `${INGREDIENTS[id].ko}×${n}`).join(' · ');
}
