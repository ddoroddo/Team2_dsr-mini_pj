// 좌상단 주문 티켓 밴드 렌더러. keyed reconcile + 등장/퇴장 애니메이션 + 1초 타이머.
import * as store from './store.js';
import { INGREDIENTS, LIMITS } from './data.js';
import { renderStack } from './stack.js';

const mounted = new Map(); // id -> { el, order }
let track;
let emptyEl;
let timerId = null;

const reduceMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;

export function init(root) {
  track = root.querySelector('[data-tickets-track]');
  emptyEl = root.querySelector('[data-tickets-empty]');
  track.addEventListener('click', onClick);
}

function onClick(e) {
  const btn = e.target.closest('button[data-action]');
  if (!btn) return;
  const id = btn.closest('.ticket')?.dataset.id;
  if (!id) return;
  // 완료 버튼과 로봇 API(window.BurgerUI.completeOrder)는 같은 store 함수를 호출한다.
  if (btn.dataset.action === 'complete') store.completeOrder(id);
  else if (btn.dataset.action === 'cancel') store.cancelOrder(id);
  else if (btn.dataset.action === 'start') store.setOrderStatus(id, 'in_progress');
}

export function render(state, event = {}) {
  const alive = new Set();
  for (const order of state.orders) {
    alive.add(order.id);
    if (mounted.has(order.id)) update(order);
    else mount(order);
  }
  for (const [id, m] of [...mounted]) {
    if (alive.has(id)) continue;
    mounted.delete(id);
    const kind = event.type === 'cancel' && event.order?.id === id ? 'cancel' : 'complete';
    unmount(m.el, kind);
  }
  emptyEl.hidden = state.orders.length > 0;
}

function mount(order) {
  const el = document.createElement('article');
  el.className = 'ticket ticket--fresh ticket--enter';
  el.dataset.id = order.id;
  el.style.setProperty('--progress', '1');
  el.innerHTML = `
    <div class="ticket__bar"></div>
    <header class="ticket__head">
      <span class="ticket__num">#${order.seq}</span>
      <time class="ticket__timer">00:00</time>
    </header>
    <div class="ticket__dish"><div class="stack"></div></div>
    <div class="ticket__name"></div>
    <ul class="ticket__chips" aria-label="재료 (아래부터 위로)"></ul>
    <div class="ticket__status"><span class="pill"></span></div>
    <footer class="ticket__actions">
      <button class="btn btn--done" type="button" data-action="complete">완료</button>
      <button class="btn btn--ghost" type="button" data-action="cancel" aria-label="주문 취소">취소</button>
    </footer>
    <div class="ticket__stamp" aria-hidden="true">완료!</div>`;

  el.querySelector('.ticket__name').textContent = order.name;
  renderStack(el.querySelector('.stack'), order.layers, { w: 84, maxH: 112 });

  const chips = el.querySelector('.ticket__chips');
  for (const id of order.layers) {
    const ing = INGREDIENTS[id];
    const li = document.createElement('li');
    li.className = 'chip';
    li.style.setProperty('--c', ing.color);
    li.title = `${ing.ko} (${ing.en})`;
    const img = document.createElement('img');
    img.src = ing.img;
    img.alt = ing.ko;
    li.appendChild(img);
    chips.appendChild(li);
  }

  el.addEventListener('animationend', (e) => {
    if (e.target === el && e.animationName === 'ticket-in') el.classList.remove('ticket--enter');
  });

  track.appendChild(el);
  mounted.set(order.id, { el, order });
  update(order);
  tick();
  track.scrollTo({ left: track.scrollWidth, behavior: reduceMotion() ? 'auto' : 'smooth' });
}

function update(order) {
  const m = mounted.get(order.id);
  m.order = order;
  const pill = m.el.querySelector('.pill');
  const cooking = order.status === 'in_progress';
  pill.textContent = cooking ? '조리 중' : '대기 중';
  pill.classList.toggle('pill--cooking', cooking);
  m.el.classList.toggle('ticket--cooking', cooking);
}

function unmount(el, kind) {
  el.classList.remove('ticket--enter');
  el.querySelector('.ticket__stamp').textContent = kind === 'cancel' ? '취소' : '완료!';
  el.classList.add(kind === 'cancel' ? 'ticket--cancelled' : 'ticket--done');
  el.querySelectorAll('button').forEach((b) => { b.disabled = true; });
  const hold = reduceMotion() ? 0 : kind === 'cancel' ? 250 : 550;
  setTimeout(() => collapse(el), hold);
}

function collapse(el) {
  if (reduceMotion()) { el.remove(); return; }
  const w = el.offsetWidth;
  el.style.width = `${w}px`;
  el.style.flexBasis = `${w}px`;
  void el.offsetHeight; // reflow 강제 → width 전환이 실제로 애니메이션됨
  el.classList.add('ticket--exit');
  let finished = false;
  const finish = () => { if (finished) return; finished = true; el.remove(); };
  el.addEventListener('transitionend', (e) => {
    if (e.target === el && e.propertyName === 'width') finish();
  });
  setTimeout(finish, 500); // 백그라운드 탭 등에서 transitionend가 안 올 때 대비
}

function fmt(ms) {
  const s = Math.max(0, Math.floor(ms / 1000));
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

function tick() {
  const now = Date.now();
  for (const { el, order } of mounted.values()) {
    const elapsed = now - order.createdAt;
    el.querySelector('.ticket__timer').textContent = fmt(elapsed);
    const remaining = Math.max(0, 1 - elapsed / LIMITS.TICKET_TIME_LIMIT_MS);
    el.style.setProperty('--progress', remaining.toFixed(3));
    const cls = remaining <= LIMITS.LATE_RATIO ? 'late' : remaining <= LIMITS.WARN_RATIO ? 'warn' : 'fresh';
    if (!el.classList.contains(`ticket--${cls}`)) {
      el.classList.remove('ticket--fresh', 'ticket--warn', 'ticket--late');
      el.classList.add(`ticket--${cls}`);
    }
  }
}

export function startTimer() {
  if (timerId) return;
  tick();
  timerId = setInterval(tick, 1000);
}
