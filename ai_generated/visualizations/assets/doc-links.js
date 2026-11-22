// 図解ページ内の Markdown（.md）へのリンクを、tools/build_docs_html.py が html/ に生成した
// 同じ内容の HTML へ向け直す。file:// で開いたとき .md は生テキストで表示されるため。
// ページの JS が後から href を書き換えても追従する。html/ に対応するページが無い .md、
// ai_generated/ の外を指すリンクはそのまま残す（リンク先の存在はブラウザテストで検査する）。
(() => {
  "use strict";
  const script = document.currentScript;
  if (!script || !script.src) return;
  const root = new URL("../../", script.src).href;   // ai_generated/
  const mirror = `${root}html/`;

  function retarget(link) {
    const raw = link.getAttribute("href");
    if (!raw || !/\.md(?:[?#]|$)/i.test(raw)) return;
    let url;
    try { url = new URL(raw, document.baseURI); } catch { return; }
    if (!url.href.startsWith(root) || url.href.startsWith(mirror) || !/\.md$/i.test(url.pathname)) return;
    const relative = url.href.slice(root.length).replace(/[?#].*$/, "").replace(/\.md$/i, ".html");
    link.dataset.markdownHref = raw;
    link.setAttribute("href", `${mirror}${relative}${url.hash}`);
  }

  function scan(node) {
    if (node.nodeType !== Node.ELEMENT_NODE) return;
    if (node.matches("a[href]")) retarget(node);
    for (const link of node.querySelectorAll("a[href]")) retarget(link);
  }

  scan(document.documentElement);
  new MutationObserver(records => {
    for (const record of records) {
      if (record.type === "attributes") retarget(record.target);
      else record.addedNodes.forEach(scan);
    }
  }).observe(document.documentElement, {subtree: true, childList: true, attributes: true, attributeFilter: ["href"]});
})();
