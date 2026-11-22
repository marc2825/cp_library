// tools/build_docs_html.py が html/assets/docs.js へ複写する。編集はこのファイルで行う。
// コードのコピーボタンと、目次で今読んでいる見出しの強調だけを担う。JS が無くても本文は読める。
(() => {
  "use strict";

  function copyText(text) {
    if (navigator.clipboard && window.isSecureContext) return navigator.clipboard.writeText(text);
    return new Promise((resolve, reject) => {
      const area = document.createElement("textarea");
      area.value = text;
      area.setAttribute("readonly", "");
      area.style.position = "fixed";
      area.style.opacity = "0";
      document.body.appendChild(area);
      area.select();
      try { document.execCommand("copy") ? resolve() : reject(new Error("copy failed")); }
      catch (error) { reject(error); }
      finally { area.remove(); }
    });
  }

  for (const pre of document.querySelectorAll(".markdown-body pre")) {
    const code = pre.querySelector("code");
    if (!code) continue;
    const button = document.createElement("button");
    button.type = "button";
    button.className = "copy-button";
    button.textContent = "コピー";
    button.setAttribute("aria-label", "コードをコピー");
    button.addEventListener("click", () => {
      copyText(code.textContent).then(
        () => { button.textContent = "コピーしました"; button.dataset.state = "done"; },
        () => { button.textContent = "コピーできません"; button.dataset.state = "error"; },
      ).finally(() => setTimeout(() => { button.textContent = "コピー"; delete button.dataset.state; }, 1600));
    });
    pre.appendChild(button);
  }

  const links = [...document.querySelectorAll(".toc a[href^='#']")];
  if (!links.length || !("IntersectionObserver" in window)) return;
  const byId = new Map(links.map(link => [decodeURIComponent(link.hash.slice(1)), link]));
  const headings = [...byId.keys()].map(id => document.getElementById(id)).filter(Boolean);
  const visible = new Set();
  const observer = new IntersectionObserver(entries => {
    for (const entry of entries) entry.isIntersecting ? visible.add(entry.target) : visible.delete(entry.target);
    const current = headings.find(heading => visible.has(heading));
    if (!current) return;
    for (const link of links) link.removeAttribute("aria-current");
    byId.get(current.id)?.setAttribute("aria-current", "true");
  }, {rootMargin: "-72px 0px -60% 0px"});
  headings.forEach(heading => observer.observe(heading));
})();
