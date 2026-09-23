import * as store from './store.js';
import * as tickets from './tickets.js';
import * as composer from './composer.js';
import { INGREDIENTS } from './data.js';
import { renderStack } from './stack.js';
import './bridge.js';

const $ = (sel, root = document) => root.querySelector(sel);

// ---- 헤더 통계 ----
function renderHeader(state) {
  const queued = state.orders.filter((o) => o.status === 'queued').length;
  const cooking = state.orders.filter((o) => o.status === 'in_progress').length;
  $('[data-stat="queued"]').textContent = queued;
  $('[data-stat="in_progress"]').textContent = cooking;
  $('[data-stat="completed"]').textContent = state.completedCount;
}

// ---- 가운데 "지금 만들 버거" 카드 ----
const nowEl = $('[data-now]');
let nowKey = null;

function renderNow(state) {
  const order = state.orders.find((o) => o.status === 'in_progress') || state.orders[0] || null;
  const key = order ? `${order.id}:${order.status}` : 'empty';
  if (key === nowKey) return;
  nowKey = key;

  if (!order) {
    nowEl.innerHTML = `<div class="now__empty">대기 중인 주문이 없습니다.<br><small>오른쪽 패널에서 버거를 골라 주문을 추가하면 여기에 첫 주문이 표시됩니다.</small></div>`;
    return;
  }
  const cooking = order.status === 'in_progress';
  nowEl.innerHTML = `
    <div class="now__card ${cooking ? 'now__card--cooking' : ''}">
      <div class="now__label">
        <span>지금 만들 버거</span>
        <span class="pill ${cooking ? 'pill--cooking' : ''}">${cooking ? '조리 중' : '대기 중'}</span>
      </div>
      <div class="now__body">
        <div class="now__thumb"><div class="stack"></div></div>
        <div class="now__info">
          <div class="now__num">#${order.seq}</div>
          <div class="now__name"></div>
          <ol class="now__steps"></ol>
          <div class="now__actions">
            <button class="btn btn--primary" type="button" data-action="start" ${cooking ? 'disabled' : ''}>조리 시작</button>
            <button class="btn btn--done" type="button" data-action="complete">완료</button>
          </div>
        </div>
      </div>
    </div>`;
  $('.now__name', nowEl).textContent = order.name;
  renderStack($('.stack', nowEl), order.layers, { w: 190, maxH: 225 });
  const steps = $('.now__steps', nowEl);
  order.layers.forEach((id) => {
    const ing = INGREDIENTS[id];
    const li = document.createElement('li');
    li.innerHTML = `<img alt=""><span></span><small></small>`;
    li.querySelector('img').src = ing.img;
    li.querySelector('span').textContent = ing.ko;
    li.querySelector('small').textContent = ing.en;
    steps.appendChild(li);
  });
  nowEl.dataset.orderId = order.id;
}

nowEl.addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-action]');
  const id = nowEl.dataset.orderId;
  if (!btn || !id) return;
  if (btn.dataset.action === 'start') store.setOrderStatus(id, 'in_progress');
  else if (btn.dataset.action === 'complete') store.completeOrder(id);
});

// ---- 시계 ----
function tickClock() {
  $('[data-clock]').textContent = new Date().toLocaleTimeString('ko-KR', { hour12: false });
}

// ---- 부트스트랩 ----
tickets.init($('[data-tickets]'));
composer.init($('[data-composer]'));

store.onChange((state, event) => {
  tickets.render(state, event);
  composer.render(state, event);
  renderHeader(state);
  renderNow(state);
});

const initial = store.getState();
tickets.render(initial, { type: 'init' });
composer.render(initial, { type: 'init' });
renderHeader(initial);
renderNow(initial);
tickets.startTimer();
tickClock();
setInterval(tickClock, 1000);
