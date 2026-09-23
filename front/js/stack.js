// 재료 이미지를 겹쳐 쌓은 버거 미리보기를 CSS 변수로 합성한다.
import { INGREDIENTS } from './data.js';

/**
 * @param {HTMLElement} el   .stack 요소
 * @param {string[]} layers  bottom → top 순 재료 id
 * @param {{w?: number, maxH?: number}} opts  w: 스택 폭(px), maxH: 이 높이를 넘으면 자동 축소
 */
export function renderStack(el, layers, { w = 84, maxH = Infinity } = {}) {
  const ings = layers.map((id) => INGREDIENTS[id]).filter(Boolean);
  if (ings.length === 0) {
    el.replaceChildren();
    el.style.setProperty('--w', `${w}px`);
    el.style.setProperty('--stack-h', '0');
    return;
  }
  const last = ings[ings.length - 1];
  const stackH = ings.slice(0, -1).reduce((sum, ing) => sum + ing.thickness, 0) + last.height;
  const width = Math.min(w, maxH / stackH);

  el.style.setProperty('--w', `${width}px`);
  el.style.setProperty('--stack-h', stackH.toFixed(3));

  const frag = document.createDocumentFragment();
  let y = 0;
  ings.forEach((ing, i) => {
    const layer = document.createElement('div');
    layer.className = 'stack__layer';
    layer.style.setProperty('--y', y.toFixed(3));
    layer.style.setProperty('--scale', ing.scale);
    layer.style.zIndex = i + 1;
    const img = document.createElement('img');
    img.src = ing.img;
    img.alt = ing.ko;
    img.draggable = false;
    layer.appendChild(img);
    frag.appendChild(layer);
    y += ing.thickness;
  });
  el.replaceChildren(frag);
}
