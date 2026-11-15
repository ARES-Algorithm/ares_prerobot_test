let token = null;
let online = false;
const $ = id => document.getElementById(id);
const vector = values => values.map(value => Number(value).toFixed(3)).join(' · ');
const notice = (message, error = false) => {
  $('notice').textContent = message;
  $('notice').classList.toggle('error', error);
};
async function api(action, payload = {}) {
  const response = await fetch('/api/' + action, {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload), cache: 'no-store'
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '请求失败');
  return result;
}
async function perform(action, payload = {}) {
  try {
    const result = await api(action, {...payload, token});
    if (action === 'claim') token = result.token;
    if (action === 'release') token = null;
    notice(action === 'command' ? '目标已更新。' : {
      claim: '已取得控制权。', stop: '正在受限减速。',
      halt: '已发送零目标。', release: '已释放控制，停止发送。'
    }[action] || '操作完成。');
    await refresh();
  } catch (error) { notice(error.message, true); }
}
async function refresh() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('状态读取失败');
    const state = await response.json();
    online = true;
    $('connection').textContent = '控制服务已连接';
    $('connection').className = 'connection online';
    if (token && !state.owner) { token = null; notice('控制权已失效，发送已停止。', true); }
    $('owner').textContent = state.owner ? `控制端：${state.owner}` : '未取得控制权';
    $('usb-state').textContent = state.usb_ready ? '已连接' : '未就绪';
    $('tx-state').textContent = state.transmitting ? '正在发送' : '未发送';
    $('mode-state').textContent = ({idle:'待命',tracking:'跟踪目标',stopping:'减速中',
      halting:'零目标',control_lost:'控制端失联',usb_lost:'USB 失联'})[state.mode] || state.mode;
    $('target-vector').textContent = vector(state.target);
    $('command-vector').textContent = vector(state.command);
    $('sent-vector').textContent = vector(state.sent);
    $('events').replaceChildren(...state.events.slice().reverse().map(event => {
      const item = document.createElement('li');
      const when = document.createElement('time'); when.textContent = event.time;
      const text = document.createElement('span'); text.textContent = event.text;
      item.append(when, text); return item;
    }));
    $('claim').disabled = Boolean(state.owner);
    for (const id of ['send','stop','halt','release']) $(id).disabled = !token;
  } catch (error) {
    online = false;
    $('connection').textContent = '控制服务失联';
    $('connection').className = 'connection offline';
    for (const id of ['claim','send','stop','halt','release']) $(id).disabled = true;
  }
}
$('claim').addEventListener('click', () => perform('claim', {client: 'Web GUI'}));
$('stop').addEventListener('click', () => perform('stop'));
$('halt').addEventListener('click', () => perform('halt'));
$('release').addEventListener('click', () => perform('release'));
$('command-form').addEventListener('submit', event => {
  event.preventDefault();
  const data = Object.fromEntries(new FormData(event.currentTarget));
  for (const key of Object.keys(data)) data[key] = Number(data[key]);
  perform('command', data);
});
setInterval(async () => {
  if (token && online) {
    try { await api('heartbeat', {token}); }
    catch { token = null; notice('控制权已失效，发送已停止。', true); }
  }
}, 400);
setInterval(refresh, 250);
refresh();
