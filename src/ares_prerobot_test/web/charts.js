(() => {
  const $ = id => document.getElementById(id);
  const rows = {command: [], wheel: [], current: [], current_mean: []};
  let cursor = 0, epoch = null, busy = false, wheel = 0, lastReceived = 0;
  const colors = {v:'#1767d2', a:'#df7d18', j:'#8236c9', Is:'#0f9f91', Id:'#d44a78'};
  const finite = x => typeof x === 'number' && Number.isFinite(x);
  const mode = $('telemetry-state');

  $('wheel-select').addEventListener('change', () => {
    wheel = Number($('wheel-select').value);
    cursor = 0;
    for (const key of Object.keys(rows)) rows[key] = [];
    draw();
  });

  async function poll() {
    if (busy) return;
    busy = true;
    try {
      const response = await fetch(`/api/telemetry?wheel=${wheel}&since=${cursor}`, {cache:'no-store'});
      if (!response.ok) throw new Error('遥测接口不可用');
      const data = await response.json();
      if (epoch !== null && epoch !== data.epoch) {
        rows.wheel = []; rows.current = []; rows.current_mean = [];
      }
      epoch = data.epoch;
      cursor = data.cursor;
      for (const key of Object.keys(rows)) rows[key].push(...data[key]);
      lastReceived = data.now;
      const cutoff = data.now - data.history;
      for (const key of Object.keys(rows)) rows[key] = rows[key].filter(r => r.t >= cutoff);
      const stale = data.sensor_age === null || data.sensor_age > .5;
      mode.textContent = data.error || (stale ? '等待电机遥测' : `遥测在线 · ${Math.round(data.sensor_age * 1000)} ms`);
      mode.classList.toggle('offline', stale || Boolean(data.error));
      draw(data.now, data.history);
    } catch (error) {
      mode.textContent = error.message;
      mode.classList.add('offline');
    } finally { busy = false; }
  }

  function setup(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    return {ctx, width:rect.width, height:rect.height};
  }

  function axes(ctx, box, lo, hi, label, digits=1) {
    ctx.strokeStyle = 'rgba(120,130,145,.18)'; ctx.lineWidth = 1;
    ctx.fillStyle = '#888d96'; ctx.font = '11px sans-serif';
    for (let k=0;k<=4;k++) {
      const y=box.y+box.h*k/4;
      ctx.beginPath();ctx.moveTo(box.x,y);ctx.lineTo(box.x+box.w,y);ctx.stroke();
      ctx.fillText((hi-(hi-lo)*k/4).toFixed(digits), 3, y+4);
    }
    for(let k=0;k<=6;k++) {
      const x=box.x+box.w*k/6;
      ctx.beginPath();ctx.moveTo(x,box.y);ctx.lineTo(x,box.y+box.h);ctx.stroke();
    }
    ctx.fillStyle = '#535b67';ctx.font='600 12px sans-serif';ctx.fillText(label,box.x+5,box.y+14);
  }

  function line(ctx, points, getY, box, start, end, lo, hi, color, dashed=false) {
    ctx.save();ctx.beginPath();ctx.rect(box.x,box.y,box.w,box.h);ctx.clip();
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dashed?[5,4]:[]);
    let started=false, previousTime=null;
    for (const p of points) {
      const y=getY(p), t=p.t;
      if (!finite(y) || !finite(t) || t<start || t>end) {started=false;previousTime=null;continue;}
      const x=box.x+(t-start)/(end-start)*box.w;
      const py=box.y+box.h-(y-lo)/(hi-lo)*box.h;
      // Break the line across missing samples instead of inventing a smooth transition.
      if (previousTime !== null && t-previousTime > .15) started=false;
      if (!started) {ctx.moveTo(x,py);started=true;} else ctx.lineTo(x,py);
      previousTime=t;
    }
    ctx.stroke();ctx.restore();
  }

  function bounds(a,b,zero=true) {
    const all=[...a,...b].filter(finite);
    if (zero) all.push(0);
    if (!all.length) return [-1,1];
    let lo=Math.min(...all),hi=Math.max(...all);
    const pad=Math.max((hi-lo)*.15,.05);
    if (lo===hi) {lo-=pad;hi+=pad;} else {lo-=pad;hi+=pad;}
    return [lo,hi];
  }

  function draw(now=lastReceived || performance.now()/1000, history=30) {
    const start=now-history;
    const motion=setup($('motion-chart'));
    if (motion) {
      const {ctx,width,height}=motion;
      ctx.clearRect(0,0,width,height);
      const left=49,right=13,top=4,bottom=25,gap=8;
      const band=(height-top-bottom-gap*2)/3;
      [['v','速度 (m/s)'],['a','加速度 (m/s²)'],['j','Jerk (m/s³)']].forEach(([key,label],index)=>{
        const box={x:left,y:top+index*(band+gap),w:width-left-right,h:band};
        const command=rows.command.map(r=>key==='v'?r.v:r.derived?.[key]);
        const measured=rows.wheel.map(r=>key==='v'?r.v:r.derived?.[key]);
        const [lo,hi]=bounds(command,measured);
        axes(ctx,box,lo,hi,label);
        const commanded=key==='v'?rows.command:rows.command.filter(r=>r.derived).map(r=>({...r,t:r.derived.t}));
        const measuredRows=key==='v'?rows.wheel:rows.wheel.filter(r=>r.derived).map(r=>({...r,t:r.derived.t}));
        line(ctx,commanded,r=>key==='v'?r.v:r.derived?.[key],box,start,now,lo,hi,colors[key]);
        line(ctx,measuredRows,r=>key==='v'?r.v:r.derived?.[key],box,start,now,lo,hi,colors[key],true);
      });
      ctx.fillStyle='#888d96';ctx.font='11px sans-serif';
      for(let k=0;k<=6;k++) ctx.fillText(`${Math.round(-history+history*k/6)}s`,left+(width-left-right)*k/6-9,height-5);
    }
    function drawCurrent(canvasId, samples) {
      const current=setup($(canvasId));
      if (!current) return;
      const {ctx,width,height}=current;
      ctx.clearRect(0,0,width,height);
      const box={x:49,y:7,w:width-62,h:height-34};
      const [lo,hi]=bounds(samples.map(r=>r.Is),samples.map(r=>r.Id));
      axes(ctx,box,lo,hi,'电流 (A)');
      line(ctx,samples,r=>r.Is,box,start,now,lo,hi,colors.Is);
      line(ctx,samples,r=>r.Id,box,start,now,lo,hi,colors.Id);
      ctx.fillStyle='#888d96';ctx.font='11px sans-serif';
      for(let k=0;k<=6;k++) ctx.fillText(`${Math.round(-history+history*k/6)}s`,box.x+box.w*k/6-9,height-5);
    }
    drawCurrent('current-chart', rows.current);
    drawCurrent('current-mean-chart', rows.current_mean);
  }

  window.addEventListener('resize',()=>draw());
  setInterval(poll,100);
  poll();
})();
