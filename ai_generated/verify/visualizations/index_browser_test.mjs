// 図解一覧（visualizations/index.html）・.md リンクの付け替え・生成した html/ の動作を確認する。
// 追加パッケージ不要。Chromium と Node.js 22 以降で実行する。
// node verify/visualizations/index_browser_test.mjs --browser /path/to/chromium [--screenshot out.png]
import assert from "node:assert/strict";
import {spawn} from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import {fileURLToPath, pathToFileURL} from "node:url";

const browserIndex = process.argv.indexOf("--browser");
if (browserIndex < 0 || !process.argv[browserIndex + 1]) throw new Error("--browser /path/to/chromium is required");
const screenshotIndex = process.argv.indexOf("--screenshot");
const screenshotPath = screenshotIndex >= 0 ? process.argv[screenshotIndex + 1] : null;
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");
const total = JSON.parse(fs.readFileSync(path.join(root, "visualizations/entries.json"), "utf8")).entries.length;
const temporary = fs.mkdtempSync(path.join(os.tmpdir(), "cp-visualization-index-"));
const browser = spawn(process.argv[browserIndex + 1], ["--headless", "--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage", "--no-first-run", "--disable-breakpad", "--remote-debugging-port=0", `--user-data-dir=${temporary}`, "about:blank"], {stdio: ["ignore", "ignore", "pipe"]});
let socket, closeBrowser;
async function bounded(promise, milliseconds) {
  let timeout;
  try { return await Promise.race([promise, new Promise(resolve => { timeout = setTimeout(resolve, milliseconds); })]); }
  finally { clearTimeout(timeout); }
}
const fileURL = relative => pathToFileURL(path.join(root, relative)).href;

try {
  const endpoint = await new Promise((resolve, reject) => {
    let error = "";
    const timeout = setTimeout(() => reject(new Error(`Chromium startup timeout: ${error}`)), 10000);
    browser.on("error", reject);
    browser.on("exit", code => { clearTimeout(timeout); reject(new Error(`Chromium exited ${code}: ${error}`)); });
    browser.stderr.on("data", data => { error += data; const match = error.match(/DevTools listening on (ws:\/\/\S+)/); if (match) { clearTimeout(timeout); resolve(match[1]); } });
  });
  socket = new WebSocket(endpoint);
  await new Promise((resolve, reject) => { socket.addEventListener("open", resolve, {once: true}); socket.addEventListener("error", reject, {once: true}); });
  let id = 0;
  const pending = new Map(), errors = [];
  socket.addEventListener("message", event => {
    const message = JSON.parse(event.data);
    if (message.method === "Runtime.exceptionThrown") errors.push(message.params.exceptionDetails.exception?.description ?? message.params.exceptionDetails.text);
    if (message.id) { const request = pending.get(message.id); pending.delete(message.id); message.error ? request.reject(new Error(JSON.stringify(message.error))) : request.resolve(message.result); }
  });
  const send = (method, params = {}, sessionId) => new Promise((resolve, reject) => { const next = ++id; pending.set(next, {resolve, reject}); socket.send(JSON.stringify({id: next, method, params, ...(sessionId ? {sessionId} : {})})); });
  closeBrowser = () => send("Browser.close");
  const {targetId} = await send("Target.createTarget", {url: "about:blank"});
  const {sessionId} = await send("Target.attachToTarget", {targetId, flatten: true});
  const command = (method, params = {}) => send(method, params, sessionId);
  await command("Page.enable"); await command("Runtime.enable");
  const evaluate = async expression => {
    const result = await command("Runtime.evaluate", {expression, returnByValue: true, awaitPromise: true});
    if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  const check = async expression => assert.equal(await evaluate(expression), true, expression);
  const wait = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));
  async function load(url, ready) {
    await command("Page.navigate", {url});
    for (let i = 0; i < 100; ++i) { if (await evaluate(`document.readyState==='complete' && (${ready})`)) return; await wait(50); }
    throw new Error(`page did not initialize: ${url}`);
  }
  async function screenshot(suffix) {
    if (!screenshotPath) return;
    const shot = await command("Page.captureScreenshot", {format: "png"});
    fs.writeFileSync(screenshotPath.replace(/\.png$/, `${suffix}.png`), Buffer.from(shot.data, "base64"));
  }
  async function checkLocalLinks() {
    const links = await evaluate(`[...new Set([...document.querySelectorAll('a[href]')].map(a => a.href))]`);
    for (const href of links) {
      const url = new URL(href);
      if (url.protocol !== "file:") continue;
      assert.ok(fs.existsSync(fileURLToPath(url)), `broken local link: ${href}`);
    }
    await check(`[...document.querySelectorAll('a[href^="#"]')].every(a => a.getAttribute('href')==='#' || document.getElementById(decodeURIComponent(a.getAttribute('href').slice(1))))`);
    return links.length;
  }
  const search = async query => evaluate(`(() => { const input=document.getElementById('search'); input.value=${JSON.stringify(query)}; input.dispatchEvent(new Event('input',{bubbles:true})); return document.querySelectorAll('.entry:not([hidden])').length; })()`);

  // ---- 図解一覧（PC幅） ----------------------------------------------------
  await command("Emulation.setDeviceMetricsOverride", {width: 1280, height: 1000, deviceScaleFactor: 1, mobile: false});
  const index = fileURL("visualizations/index.html");
  await load(index, "!document.querySelector('.toolbar').hidden");
  await check(`document.querySelectorAll('.entry').length===${total}`);
  await check(`document.getElementById('result-count').textContent.includes('${total}')`);
  await check("document.querySelector('.sidebar').open");
  await check("document.querySelectorAll('.az-letter li').length>100");
  // 各図解は分野にちょうど1回ずつ現れ、全ての分野は目次から辿れる
  await check(`[...document.querySelectorAll('.category')].every(c => document.querySelector('.nav-link[href="#'+c.id+'"]'))`);
  const indexLinks = await checkLocalLinks();
  assert.ok(indexLinks > total, `too few links: ${indexLinks}`);
  await screenshot("");

  // 英略称・ひらがな・複数語・該当なし・Escape・/キー
  const dsu = await search("dsu");
  assert.ok(dsu >= 10, `dsu: ${dsu}`);
  await check("document.getElementById('az-index').hidden && document.getElementById('empty').hidden");
  await check(`document.getElementById('result-count').textContent.includes('${dsu}')`);
  await check("[...document.querySelectorAll('.category')].every(c => c.hidden === !c.querySelector('.entry:not([hidden])'))");
  await check("[...document.querySelectorAll('.nav-link')].some(a => a.classList.contains('is-empty'))");
  assert.ok(await search("せぐ木") >= 5, "hiragana query should match セグ木");
  assert.ok(await search("Ｆｅｎｗｉｃｋ") >= 2, "full-width query should match Fenwick");
  const both = await search("dijkstra 最短");
  assert.ok(both >= 1 && both < await search("dijkstra") + 1, `multi-term: ${both}`);
  assert.equal(await search("zzqqxxyy"), 0);
  await check("!document.getElementById('empty').hidden && [...document.querySelectorAll('.group')].every(g => g.hidden)");
  await evaluate("document.getElementById('search').dispatchEvent(new KeyboardEvent('keydown',{key:'Escape',bubbles:true}))");
  await check(`document.querySelectorAll('.entry:not([hidden])').length===${total} && !document.getElementById('az-index').hidden`);
  await evaluate("document.activeElement.blur(); document.body.dispatchEvent(new KeyboardEvent('keydown',{key:'/',bubbles:true}))");
  await check("document.activeElement===document.getElementById('search')");

  // 表示切替は保存され、再読み込み後も維持される
  await evaluate("document.querySelector('.view-toggle [data-view=list]').click()");
  await check("document.getElementById('catalog').dataset.view==='list' && document.querySelector('[data-view=list]').getAttribute('aria-pressed')==='true'");
  await screenshot("-list");
  await load(`${index}?q=fenwick`, "!document.querySelector('.toolbar').hidden");
  await check("document.getElementById('search').value==='fenwick'");
  await check(`document.querySelectorAll('.entry:not([hidden])').length>0 && document.querySelectorAll('.entry:not([hidden])').length<${total}`);
  await check("document.getElementById('catalog').dataset.view==='list'");
  await evaluate("document.querySelector('.view-toggle [data-view=cards]').click()");

  // ---- 図解一覧（スマホ幅） ------------------------------------------------
  await command("Emulation.setDeviceMetricsOverride", {width: 390, height: 844, deviceScaleFactor: 1, mobile: true});
  await load(index, "!document.querySelector('.toolbar').hidden");
  await check("window.innerWidth===390");
  await check("document.documentElement.scrollWidth<=window.innerWidth+1");
  await check("!document.querySelector('.sidebar').open");
  await evaluate("document.querySelector('.sidebar summary').click()");
  await check("document.querySelector('.sidebar').open");
  await evaluate("document.querySelector('.nav-link').click()");
  await check("!document.querySelector('.sidebar').open");
  await screenshot("-mobile");

  // ---- 図解ページの .md リンクは html/ の同じページへ ---------------------
  await command("Emulation.setDeviceMetricsOverride", {width: 1280, height: 1000, deviceScaleFactor: 1, mobile: false});
  await load(fileURL("visualizations/union_find/index.html"), "!!document.querySelector('#forest .node')");
  await check("document.getElementById('docs-link').href.endsWith('/html/docs/union_find/disjoint_set_union.html')");
  await check("document.getElementById('docs-link').dataset.markdownHref.endsWith('.md')");
  await checkLocalLinks();
  // JS が後から差し替えるリンクにも追従する
  await evaluate("document.getElementById('docs-link').setAttribute('href','../../docs/union_find/README.md')");
  await wait(50);
  await check("document.getElementById('docs-link').href.endsWith('/html/docs/union_find/README.html')");
  await load(fileURL("visualizations/a_priority/index.html"), "document.querySelectorAll('a[href]').length>2");
  await check("![...document.querySelectorAll('a[href]')].some(a => /\\.md([?#]|$)/.test(a.href))");
  await checkLocalLinks();

  // ---- 生成した html/ -------------------------------------------------------
  await load(fileURL("html/docs/union_find/disjoint_set_union.html"), "!!document.querySelector('.copy-button')");
  await check("document.title.includes('Union-Find')");
  await check("document.querySelectorAll('.toc a').length>=3 && document.querySelectorAll('.toc a').length===document.querySelectorAll('.markdown-body :is(h2,h3)[id]').length");
  await check("document.querySelector('.markdown-body a[href$=\"references.html\"]')!==null");
  await checkLocalLinks();
  await load(fileURL("html/README.html"), "!!document.querySelector('.markdown-body')");
  await check("![...document.querySelectorAll('.markdown-body a[href]')].some(a => a.protocol==='file:' && /\\.md([?#]|$)/.test(a.href))");
  await checkLocalLinks();
  await command("Emulation.setDeviceMetricsOverride", {width: 390, height: 844, deviceScaleFactor: 1, mobile: true});
  await load(fileURL("html/FEATURES.html"), "!!document.querySelector('.markdown-body table')");
  await check("document.documentElement.scrollWidth<=window.innerWidth+1");

  assert.deepEqual(errors, []);
  console.log(`browser: index (${total} entries) search/view/mobile, .md link retargeting, generated docs passed`);
} finally {
  const exited = new Promise(resolve => browser.once("exit", resolve));
  const running = () => browser.pid && browser.exitCode === null && browser.signalCode === null;
  if (running() && closeBrowser && socket?.readyState === WebSocket.OPEN) {
    await bounded(closeBrowser().catch(() => {}), 3000);
    if (running()) await bounded(exited, 3000);
  }
  if (running()) { browser.kill("SIGTERM"); await bounded(exited, 2000); }
  if (running()) { browser.kill("SIGKILL"); await exited; }
  if (socket) socket.close();
  await fs.promises.rm(temporary, {recursive: true, force: true, maxRetries: 10, retryDelay: 100});
}
