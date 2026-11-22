// 図解一覧の検索・表示切替・目次の現在位置。JS が無くても一覧と索引はそのまま読める。
(() => {
  "use strict";

  const $ = (selector, root = document) => root.querySelector(selector);
  const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
  const storage = {
    get(key) { try { return window.localStorage.getItem(key); } catch { return null; } },
    set(key, value) { try { window.localStorage.setItem(key, value); } catch { /* 保存できなくても動く */ } },
  };

  // 全角・半角、大文字・小文字、ひらがな・カタカナ、区切り記号の違いを無視して比べる
  const normalize = text => text.normalize("NFKC").toLowerCase()
    .replace(/[ぁ-ゖ]/g, c => String.fromCharCode(c.charCodeAt(0) + 0x60))
    .replace(/[\s\-–—_・/.,'’()[\]{}「」『』【】:;|]+/g, "");

  const toolbar = $(".toolbar");
  const input = $("#search");
  const count = $("#result-count");
  const catalog = $("#catalog");
  const empty = $("#empty");
  const az = $("#az-index");
  const sidebar = $(".sidebar");
  toolbar.hidden = false;

  const entries = $$(".entry").map(el => ({el, text: normalize(`${el.dataset.terms} ${el.textContent}`)}));
  const categories = $$(".category").map(el => ({
    el, entries: $$(".entry", el), badge: $(".category-head .count", el),
    nav: $(`.nav-link[href="#${el.id}"]`), shown: 0,
  }));
  const groups = $$(".group").map(el => ({
    el, categories: categories.filter(category => el.contains(category.el)),
    nav: $(`.nav-group-link[href="#${el.id}"]`),
  }));
  const total = entries.length;

  function setCount(link, shown) {
    if (!link) return;
    $(".count", link).textContent = shown;
    link.classList.toggle("is-empty", shown === 0);
    if (shown === 0) link.setAttribute("aria-disabled", "true"); else link.removeAttribute("aria-disabled");
  }

  function filter(query) {
    const terms = query.normalize("NFKC").split(/\s+/).map(normalize).filter(Boolean);
    let shown = 0;
    for (const entry of entries) {
      const match = terms.every(term => entry.text.includes(term));
      entry.el.hidden = !match;
      if (match) shown += 1;
    }
    for (const category of categories) {
      category.shown = category.entries.filter(el => !el.hidden).length;
      category.el.hidden = category.shown === 0;
      category.badge.textContent = category.shown;
      setCount(category.nav, category.shown);
    }
    for (const group of groups) {
      const shownInGroup = group.categories.reduce((sum, category) => sum + category.shown, 0);
      group.el.hidden = shownInGroup === 0;
      setCount(group.nav, shownInGroup);
    }
    const active = terms.length > 0;
    az.hidden = active;
    empty.hidden = shown > 0;
    count.innerHTML = active ? `<strong>${shown}</strong> / ${total} 件` : `全 <strong>${total}</strong> 件`;
    return active;
  }

  function syncURL(query) {
    try {
      const url = new URL(window.location.href);
      if (query) url.searchParams.set("q", query); else url.searchParams.delete("q");
      url.hash = "";
      window.history.replaceState(null, "", url);
    } catch { /* file:// で URL を書き換えられないブラウザでも検索は動く */ }
  }

  function search() {
    const active = filter(input.value);
    syncURL(input.value.trim());
    // 一覧の途中までスクロールしていたら、結果の先頭へ戻す
    if (active && catalog.getBoundingClientRect().top < 0) {
      window.scrollTo({top: window.scrollY + catalog.getBoundingClientRect().top - toolbar.offsetHeight - 12});
    }
  }

  input.addEventListener("input", search);
  input.addEventListener("keydown", event => {
    if (event.key === "Escape" && input.value) { input.value = ""; search(); event.stopPropagation(); }
    if (event.key === "Enter") { $(".entry:not([hidden]) .entry-open")?.focus(); event.preventDefault(); }
  });
  document.addEventListener("keydown", event => {
    if (event.key !== "/" || event.ctrlKey || event.metaKey || event.altKey) return;
    if (event.target.closest("input, textarea, select, [contenteditable]")) return;
    event.preventDefault();
    input.focus();
    input.select();
  });

  const initial = new URLSearchParams(window.location.search).get("q");
  if (initial) input.value = initial;
  filter(input.value);

  // カード／リスト表示
  const viewButtons = $$(".view-toggle button");
  function setView(view) {
    catalog.dataset.view = view;
    for (const button of viewButtons) button.setAttribute("aria-pressed", String(button.dataset.view === view));
  }
  for (const button of viewButtons) {
    button.addEventListener("click", () => { setView(button.dataset.view); storage.set("cp-visualizations-view", button.dataset.view); });
  }
  const savedView = storage.get("cp-visualizations-view");
  if (savedView === "list" || savedView === "cards") setView(savedView);

  // 分野の目次：広い画面では常に開き、狭い画面では閉じて置く
  const wide = window.matchMedia("(min-width: 960px)");
  const syncSidebar = () => { sidebar.open = wide.matches; };
  syncSidebar();
  wide.addEventListener("change", syncSidebar);
  sidebar.addEventListener("click", event => {
    if (event.target.closest("a") && !wide.matches) sidebar.open = false;
  });
  // 広い画面では目次を閉じさせない
  sidebar.addEventListener("toggle", () => { if (wide.matches && !sidebar.open) sidebar.open = true; });

  // 今見ている分野を目次で示す
  if ("IntersectionObserver" in window) {
    const visible = new Set();
    const links = $$(".nav-link");
    const observer = new IntersectionObserver(records => {
      for (const record of records) record.isIntersecting ? visible.add(record.target) : visible.delete(record.target);
      const current = categories.find(category => visible.has(category.el));
      for (const link of links) link.removeAttribute("aria-current");
      if (!current?.nav) return;
      current.nav.setAttribute("aria-current", "true");
      if (wide.matches) {
        const box = sidebar.getBoundingClientRect(), item = current.nav.getBoundingClientRect();
        // 目次だけを動かし、ページ自体はスクロールさせない
        if (item.top < box.top || item.bottom > box.bottom) sidebar.scrollTop += item.top - box.top - box.height / 2;
      }
    }, {rootMargin: "-90px 0px -65% 0px"});
    categories.forEach(category => observer.observe(category.el));
  }
})();
