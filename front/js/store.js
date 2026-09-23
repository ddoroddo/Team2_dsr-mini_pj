// 순수 상태 저장소. DOM을 만지지 않는다. 모든 상태 변경은 여기서만 일어나고 onChange로 알린다.
import { INGREDIENTS, RECIPES, MIDDLE_IDS, LIMITS, makeLayers, middleOf, nameFor } from './data.js';

const SESSION = Date.now().toString(36).slice(-4); // 새로고침해도 주문 id가 겹치지 않도록

const state = {
  orders: [],           // 대기/조리 중인 주문만. 완료·취소되면 즉시 빠진다.
  nextSeq: 1,
  completedCount: 0,
  cancelledCount: 0,
  builder: { middle: [], presetId: null },
};

const listeners = new Set();

/** 상태 변경 구독. 콜백은 (state 스냅샷, event) 로 호출된다. 해제 함수를 반환. */
export function onChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

function emit(event) {
  const snapshot = getState();
  for (const fn of listeners) {
    try { fn(snapshot, event); } catch (err) { console.error('[store] listener error', err); }
  }
}

export function getState() { return structuredClone(state); }
export function getOrders() { return structuredClone(state.orders); }

function normalizeLayers(layers) {
  if (!Array.isArray(layers)) throw new Error('layers는 재료 id 배열이어야 합니다');
  const ids = layers.map(String);
  for (const id of ids) if (!INGREDIENTS[id]) throw new Error(`알 수 없는 재료: ${id}`);
  const middle = middleOf(ids);
  if (middle.length > LIMITS.MAX_MIDDLE) throw new Error(`중간 재료는 최대 ${LIMITS.MAX_MIDDLE}개까지 넣을 수 있습니다`);
  return makeLayers(middle);
}

/** 주문 추가. layers 는 bottom→top 순, 번은 없으면 자동으로 붙는다. 주문 객체(복사본)를 반환. */
export function addOrder(layers, name) {
  if (state.orders.length >= LIMITS.MAX_QUEUE) {
    throw new Error(`주문 대기열이 가득 찼습니다 (최대 ${LIMITS.MAX_QUEUE}개)`);
  }
  const normalized = normalizeLayers(layers);
  const order = {
    id: `${SESSION}-${state.nextSeq}`,
    seq: state.nextSeq++,
    name: name || nameFor(normalized),
    layers: normalized,
    createdAt: Date.now(),
    status: 'queued',
    completedAt: null,
  };
  state.orders.push(order);
  emit({ type: 'add', order: structuredClone(order) });
  return structuredClone(order);
}

function removeOrder(id, status, type) {
  const idx = state.orders.findIndex((o) => o.id === id);
  if (idx < 0) return false;
  const [order] = state.orders.splice(idx, 1);
  order.status = status;
  order.completedAt = Date.now();
  if (status === 'done') state.completedCount += 1; else state.cancelledCount += 1;
  emit({ type, order: structuredClone(order) });
  return true;
}

/** 로봇이 쌓기를 끝냈을 때 (또는 데모용 완료 버튼) */
export function completeOrder(id) { return removeOrder(id, 'done', 'complete'); }
export function cancelOrder(id) { return removeOrder(id, 'cancelled', 'cancel'); }

/** 'queued' | 'in_progress' 로 상태 변경 (완료/취소는 completeOrder/cancelOrder 사용) */
export function setOrderStatus(id, status) {
  if (!['queued', 'in_progress'].includes(status)) throw new Error(`지원하지 않는 상태: ${status}`);
  const order = state.orders.find((o) => o.id === id);
  if (!order) return false;
  if (order.status === status) return true;
  order.status = status;
  emit({ type: 'status', order: structuredClone(order) });
  return true;
}

// ---- 커스텀 조립(builder) ----
export function pushLayer(id) {
  if (!MIDDLE_IDS.includes(id)) throw new Error(`중간 재료가 아닙니다: ${id}`);
  if (state.builder.middle.length >= LIMITS.MAX_MIDDLE) return false;
  state.builder.middle.push(id);
  state.builder.presetId = null;
  emit({ type: 'builder' });
  return true;
}

export function popLayer() {
  if (state.builder.middle.length === 0) return false;
  state.builder.middle.pop();
  state.builder.presetId = null;
  emit({ type: 'builder' });
  return true;
}

export function clearBuilder() {
  state.builder.middle = [];
  state.builder.presetId = null;
  emit({ type: 'builder' });
}

export function loadPreset(recipeId) {
  const recipe = RECIPES.find((r) => r.id === recipeId);
  if (!recipe) throw new Error(`알 수 없는 레시피: ${recipeId}`);
  state.builder.middle = [...recipe.middle];
  state.builder.presetId = recipe.id;
  emit({ type: 'builder' });
}
