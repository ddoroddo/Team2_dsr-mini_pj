// 우측 "주문 만들기" 패널: 프리셋 카드 + 커스텀 조립 + 주문 추가.
import * as store from './store.js';
import { INGREDIENTS, RECIPES, MIDDLE_IDS, LIMITS, makeLayers, nameFor } from './data.js';
import { renderStack } from './stack.js';

const refs = {};
let toastTimer = null;

export function init(root) {
  refs.grid = root.querySelector('[data-preset-grid]');
  refs.stack = root.querySelector('[data-builder-stack]');
  refs.name = root.querySelector('[data-builder-name]');
  refs.layers = root.querySelector('[data-builder-layers]');
  refs.ingredients = root.querySelector('[data-builder-ingredients]');
  refs.count = root.querySelector('[data-builder-count]');
  refs.toast = root.querySelector('[data-toast]');
  refs.pop = root.querySelector('[data-action="pop"]');
  refs.clear = root.querySelector('[data-action="clear"]');

  buildPresets();
  buildIngredientButtons();
  root.addEventListener('click', onClick);
  root.addEventListener('keydown', onKeydown);
}

function buildPresets() {
  const frag = document.createDocumentFragment();
  for (const r of RECIPES) {
    const layers = makeLayers(r.middle);
    const card = document.createElement('div');
    card.className = 'preset';
    card.dataset.preset = r.id;
    card.setAttribute('role', 'button');
    card.tabIndex = 0;
    card.title = `${r.name}: ${layers.map((id) => INGREDIENTS[id].ko).join(' → ')}`;
    card.innerHTML = `
      <div class="preset__thumb"><div class="stack"></div></div>
      <div class="preset__name"></div>
      <div class="preset__meta">${layers.length}층</div>
      <button class="preset__add" type="button" data-action="quick-add" data-preset="${r.id}" aria-label="${r.name} 바로 주문">+</button>`;
    card.querySelector('.preset__name').textContent = r.name;
    renderStack(card.querySelector('.stack'), layers, { w: 56, maxH: 70 });
    frag.appendChild(card);
  }
  refs.grid.replaceChildren(frag);
}

function buildIngredientButtons() {
  const frag = document.createDocumentFragment();
  for (const id of MIDDLE_IDS) {
    const ing = INGREDIENTS[id];
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'ingredient';
    btn.dataset.action = 'push';
    btn.dataset.ingredient = id;
    btn.style.setProperty('--c', ing.color);
    btn.innerHTML = `<img alt=""><span class="ingredient__label"></span><span class="ingredient__sub"></span>`;
    btn.querySelector('img').src = ing.img;
    btn.querySelector('.ingredient__label').textContent = ing.ko;
    btn.querySelector('.ingredient__sub').textContent = ing.en;
    frag.appendChild(btn);
  }
  refs.ingredients.replaceChildren(frag);
}

function onClick(e) {
  const btn = e.target.closest('button[data-action]');
  if (btn) {
    const action = btn.dataset.action;
    if (action === 'push') store.pushLayer(btn.dataset.ingredient);
    else if (action === 'pop') store.popLayer();
    else if (action === 'clear') store.clearBuilder();
    else if (action === 'add') submitBuilder();
    else if (action === 'quick-add') quickAdd(btn.dataset.preset);
    return;
  }
  const card = e.target.closest('[data-preset]');
  if (card) store.loadPreset(card.dataset.preset);
}

function onKeydown(e) {
  if (e.key !== 'Enter' && e.key !== ' ') return;
  const card = e.target.closest('[data-preset]');
  if (!card || e.target.tagName === 'BUTTON') return;
  e.preventDefault();
  store.loadPreset(card.dataset.preset);
}

function quickAdd(recipeId) {
  const r = RECIPES.find((x) => x.id === recipeId);
  if (!r) return;
  try {
    const order = store.addOrder(makeLayers(r.middle), r.name);
    toast(`#${order.seq} ${order.name} 주문 추가`, 'ok');
  } catch (err) { toast(err.message); }
}

function submitBuilder() {
  const { middle } = store.getState().builder;
  try {
    const layers = makeLayers(middle);
    const order = store.addOrder(layers, nameFor(layers));
    toast(`#${order.seq} ${order.name} 주문 추가`, 'ok');
  } catch (err) { toast(err.message); }
}

function toast(msg, kind = 'error') {
  refs.toast.textContent = msg;
  refs.toast.className = `toast toast--show${kind === 'ok' ? ' toast--ok' : ''}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => refs.toast.classList.remove('toast--show'), 2200);
}

export function render(state) {
  const { middle, presetId } = state.builder;
  const layers = makeLayers(middle);

  renderStack(refs.stack, layers, { w: 190, maxH: 225 });
  refs.name.textContent = nameFor(layers);
  refs.layers.textContent = layers.map((id) => INGREDIENTS[id].ko).join(' → ');
  refs.count.textContent = `${middle.length} / ${LIMITS.MAX_MIDDLE}`;

  const full = middle.length >= LIMITS.MAX_MIDDLE;
  refs.ingredients.querySelectorAll('button').forEach((b) => { b.disabled = full; });
  refs.pop.disabled = middle.length === 0;
  refs.clear.disabled = middle.length === 0;

  refs.grid.querySelectorAll('[data-preset]').forEach((card) => {
    card.classList.toggle('preset--active', card.dataset.preset === presetId);
  });
}
