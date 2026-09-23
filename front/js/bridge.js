// 외부(로봇 서버, 콘솔, 다른 스크립트)에서 쓰는 공개 API 와 로봇 연동 훅.
import * as store from './store.js';
import { INGREDIENTS, RECIPES, LIMITS, makeLayers } from './data.js';

export const BurgerUI = {
  version: '0.1.0',
  addOrder: store.addOrder,             // (layers, name?) → order
  completeOrder: store.completeOrder,   // (id) → boolean   ← 로봇이 쌓기를 끝냈을 때 호출
  cancelOrder: store.cancelOrder,       // (id) → boolean
  setOrderStatus: store.setOrderStatus, // (id, 'queued'|'in_progress') → boolean
  getOrders: store.getOrders,           // () → order[]
  getState: store.getState,
  onChange: store.onChange,             // ((state, event) => void) → unsubscribe
  makeLayers,
  INGREDIENTS,
  RECIPES,
  LIMITS,
};
window.BurgerUI = BurgerUI;

// ============================================================
//  로봇 연동 훅 (스텁)
//  지금은 데모라 티켓의 [완료] 버튼이 store.completeOrder 를 직접 호출한다.
//  나중에 로봇 서버가 생기면 여기서 WebSocket/fetch 를 열고,
//  받은 메시지를 handleMessage 로 넘기기만 하면 된다. UI 코드는 손댈 필요 없음.
//
//  UI  → 로봇 : { type:'order_added', order, yolo:[...] } / { type:'order_cancelled', id }
//  로봇 → UI  : { type:'order_status', id, status:'in_progress'|'queued' } / { type:'order_done', id }
// ============================================================
export function connectRobot({ url = null, send = null } = {}) {
  // 예시 (WebSocket):
  //   const ws = new WebSocket(url);
  //   ws.onmessage = (e) => bridge.handleMessage(JSON.parse(e.data));
  //   send = (msg) => ws.readyState === 1 && ws.send(JSON.stringify(msg));
  const unsubscribe = store.onChange((_state, ev) => {
    if (!send) return;
    if (ev.type === 'add') {
      send({ type: 'order_added', order: ev.order, yolo: ev.order.layers.map((id) => INGREDIENTS[id].yolo) });
    } else if (ev.type === 'cancel') {
      send({ type: 'order_cancelled', id: ev.order.id });
    }
  });

  const bridge = {
    url,
    handleMessage(msg) {
      if (!msg || typeof msg !== 'object') return false;
      if (msg.type === 'order_done') return store.completeOrder(msg.id);
      if (msg.type === 'order_status') return store.setOrderStatus(msg.id, msg.status);
      console.warn('[bridge] 알 수 없는 메시지', msg);
      return false;
    },
    disconnect() { unsubscribe(); },
  };
  return bridge;
}
