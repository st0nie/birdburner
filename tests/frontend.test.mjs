// Dependency-free regression of the embedded page's state and request handling.
// Real layout and native form validation are additionally checked in the external browser demo.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import fs from 'node:fs';

const html = fs.readFileSync(new URL('../src/index.html', import.meta.url), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const normal = {
  target_c: 25, temperature_c: 24.87, last_good_temperature_c: 24.87,
  commanded_duty_pct: 18, pid_output_pct: 18.2, max_output_pct: 100,
  max_temperature_c: 35, pid_kp: 10, pid_ki: 0.1, pid_kd: 0, pid_d_filter_s: 30,
  mode: 'pid', pid_enabled: true, desired_enabled: true, fault: null,
  sensor: 'ok', sensor_reset_attempt: 0, sensor_power_cycles: 25,
  sample_hz: 1.5, window_ok_reads: 8, window_failed_reads: 2,
  storage_ok: true, settings_pending: false, ssr_command: 'Open', ip: '127.0.0.1',
};
const flush = () => new Promise(resolve => setImmediate(resolve));
function harness() {
  const elements = new Map();
  const timers = new Map();
  let nextTimer = 0;
  const document = { activeElement: null, hidden: false, addEventListener() {} };
  for (const [,id] of html.matchAll(/\bid="([^"]+)"/g)) {
    const el = {
      id, value: '', textContent: '', className: '', disabled: false, hidden: false,
      style: {}, listeners: {}, attributes: {},
      addEventListener(name,fn) { this.listeners[name] = fn; },
      setAttribute(name,value) { this.attributes[name] = value; },
      blur() { if (document.activeElement === this) document.activeElement = null; },
    };
    elements.set(id, el);
  }
  document.getElementById = id => elements.get(id);
  document.body = { classList: { add() {}, remove() {} } };
  const context = vm.createContext({ document, console, AbortController, Date,
    setTimeout(fn,ms) { const id = ++nextTimer; timers.set(id, {fn,ms}); return id; },
    clearTimeout(id) { timers.delete(id); },
    fetch: async () => ({ok: true, json: async () => ({...normal})}),
  });
  vm.runInContext(script, context);
  const run = expression => vm.runInContext(expression, context);
  const fire = ms => { for (const [id,t] of [...timers]) if (t.ms === ms) { timers.delete(id); t.fn(); } };
  return {context, elements, timers, document, run, fire, el: id => elements.get(id)};
}

test('reset and post-reset average wait do not become a fault; true failures do', async () => {
  const h = harness(); await flush();
  for (const sensor of ['resetting','recovering']) {
    h.context.input = {...normal, sensor, temperature_c: null, sensor_reset_attempt: sensor==='resetting'?1:0};
    h.run('render(input)');
    assert.equal(h.el('mode').textContent, 'AUTO / HEATING');
    assert.equal(h.el('alarm').hidden, true);
    assert.equal(h.el('temp').textContent, '24.87*');
  }
  h.context.input = {...normal, fault: 'sensor_error', mode:'fault', sensor:'error', commanded_duty_pct:0, temperature_c:null};
  h.run('render(input)');
  assert.equal(h.el('alarm').hidden, false);
  assert.equal(h.el('mode').textContent, 'FAULT');
  h.context.input = {...normal}; h.run('render(input)');
  assert.equal(h.el('alarm').hidden, true);
  assert.equal(h.el('mode').textContent, 'AUTO / HEATING');
});

test('PID feedback is local, confirms flash, then disappears; edits survive polling', async () => {
  const h = harness(); await flush();
  h.el('kp').value = '12.345'; h.el('kp').listeners.input();
  h.context.input = {...normal}; h.run('render(input)');
  assert.equal(h.el('kp').value, '12.345');
  const applied = {...normal, pid_kp:12.345, settings_pending:true};
  h.context.fetch = async () => ({ok:true, json: async () => applied});
  await h.run("act('/api/pid',{kp:12.345,ki:0.1,kd:0},['kp','ki','kd'],'PID parameters')");
  assert.equal(h.el('pidmsg').textContent, 'Applied · saving to flash…');
  assert.equal(h.el('limitsmsg').textContent, '');
  assert.equal(h.el('msg').textContent, '');
  h.context.input = {...applied, settings_pending:false}; h.run('render(input)');
  assert.equal(h.el('pidmsg').textContent, 'PID parameters saved.');
  h.fire(4000);
  assert.equal(h.el('pidmsg').textContent, '');
});

test('flash failure is not reported as saved and a later successful write clears it', async () => {
  const h = harness(); await flush();
  h.context.fetch = async () => ({ok:true, json: async () => ({...normal, settings_pending:true})});
  await h.run("act('/api/pid',{kp:10,ki:0.1,kd:0},['kp','ki','kd'],'PID parameters')");
  h.context.input = {...normal, storage_ok:false, settings_pending:true}; h.run('render(input)');
  assert.match(h.el('pidmsg').textContent, /Flash write failed/);
  assert.equal(h.el('alarm').hidden, false);
  h.context.input = {...normal, storage_ok:true, settings_pending:false}; h.run('render(input)');
  assert.equal(h.el('pidmsg').textContent, 'PID parameters saved.');
  assert.equal(h.el('alarm').hidden, true);
});

test('rejected PID does not erase a draft; defaults are draft-only', async () => {
  const h = harness(); await flush();
  h.el('kp').value = '101'; h.el('kp').listeners.input();
  let calls = 0;
  h.context.fetch = async () => {calls++; return {ok:false, json:async()=>({error:'pid_gains_out_of_range_kp_0_100_ki_0_2_kd_0_200'})};};
  await h.run("act('/api/pid',{kp:101,ki:0.1,kd:0},['kp','ki','kd'],'PID parameters')");
  assert.equal(calls,1,'a validation rejection is not a transport retry');
  assert.equal(h.el('kp').value,'101');
  assert.equal(h.el('pidmsg').className,'local-feedback error');
  h.el('defaults').onclick();
  assert.equal(calls,1,'defaults do not apply without explicit confirmation');
  assert.equal(h.el('kp').value,20);
  assert.equal(h.el('ki').value,0.02);
  assert.equal(h.el('kd').value,120);
  assert.equal(h.el('tf').value,30);
});

test('offline values are marked not-live; reconnect restores normal state', async () => {
  const h = harness(); await flush();
  h.run('offline()');
  assert.equal(h.el('conn').textContent,'OFFLINE');
  assert.equal(h.el('stop').disabled,true);
  assert.match(h.el('alarm').textContent,/not live/);
  h.context.input={...normal}; h.run('render(input)');
  assert.equal(h.el('conn').textContent,'ONLINE');
  assert.equal(h.el('alarm').hidden,true);
  assert.equal(h.el('stop').disabled,false);
});

test('transport retry has a new deadline and AbortController after a timed-out read', async () => {
  const h = harness(); await flush();
  let calls = 0, firstSignal;
  h.context.fetch = async (_path,options) => {
    calls++;
    if(calls===1) {
      firstSignal=options.signal;
      return {ok:true,json:()=>new Promise((_resolve,reject)=>options.signal.addEventListener('abort',()=>reject(new Error('aborted'))))};
    }
    assert.notEqual(options.signal,firstSignal);
    assert.equal(options.signal.aborted,false);
    return {ok:true,json:async()=>({...normal})};
  };
  const request = h.run("call('/api/status')");
  await flush(); h.fire(4000); await flush(); h.fire(300);
  await request;
  assert.equal(calls,2);
});

test('Stop cancels an in-flight Start and prevents a late Start retry', async () => {
  const h = harness(); await flush();
  let starts=0;
  h.context.fetch = async(path,options) => {
    if(path==='/api/start') {
      starts++;
      return new Promise((_resolve,reject)=>options.signal.addEventListener('abort',()=>reject(new Error('aborted'))));
    }
    return {ok:true,json:async()=>({...normal,mode:'stopped',pid_enabled:false,desired_enabled:false,commanded_duty_pct:0})};
  };
  const start = h.run("act('/api/start',null,[],'Heating enabled')");
  await flush();
  await h.run("act('/api/stop',null,[],'Heating stopped')");
  await start;
  h.fire(300); await flush();
  assert.equal(starts,1);
  assert.equal(h.el('mode').textContent,'STOPPED');
  assert.equal(h.el('duty').textContent,'0');
});

test('D filter is edited with the PID form and sent with the gains', async () => {
  const h = harness(); await flush();
  h.context.input = {...normal}; h.run('render(input)');
  assert.equal(h.el('tf').value, 30);
  h.el('tf').value = '45'; h.el('tf').listeners.input();
  assert.equal(h.el('pidsave').disabled, false);
  let sent = null;
  h.context.fetch = async (url, opts) => { sent = {url, body: JSON.parse(opts.body)}; return {ok:true, json: async () => ({...normal, pid_d_filter_s:45, settings_pending:true})}; };
  h.el('pidform').listeners.submit({preventDefault() {}});
  await flush(); await flush();
  assert.equal(sent.url, '/api/pid');
  assert.equal(sent.body.d_filter_s, 45);
});
