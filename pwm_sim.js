/**
 * 理论PWM曲线仿真 v2 - 精调电机模型 + SVG矢量图
 * 控制架构: 位置PID → S曲线限速 → 速度PID → PWM → 直流电机
 */
const fs = require('fs');

// ============== 用户参数 ==============
const force     = 800;     // F:  位置环输出限幅 (= 最大速度设定)
const force_F   = 3500;    // R:  最终PWM限幅 (3500/3599≈97.2%)
const time_c    = 140;     // V:  匀速进孔速度
const pulse_buf = 30000;   // BS: S曲线范围
const time_buf  = 450;     // BV: S曲线起点限速
const target    = 39200;   // 单工位脉冲数

// PID
const KP = 0.150, KI = 0.005, KD = 0.015;
const SKP = 1.300, SKI = 0.850, SKD = 0.000;

// 电机模型 (一阶惯性)
// 关键: 速度设定(单位=PWM等效) 应与 实际速度(counts/5ms) 匹配
// 稳态时 speed ≈ SetPoint_S, 速度PID积分项自动补偿差异
// K_motor=0.30 → PWM=2667产生speed=800, 速度环有充分调节余量
const K_motor = 0.30;     // 稳态增益: speed_ss = K_motor * PWM
const tau_s  = 0.040;     // 时间常数 ~40ms (电气+机械)
const dt     = 0.025;     // PID周期

// ============== 状态初始化 ==============
let pos = 0, speed = 0, speed_f = 0, speed_f_prev = 0;
let SumE_P = 0, LstE_P = 0, PrvE_P = 0;
let AccS = 0, LstE_S = 0, PrvE_S = 0;
let sp_buf = new Array(10).fill(0), sp_idx = 0;
const recs = [];

for (let step = 0; step < 800; step++) {
    const t = step * dt;
    if (pos >= target && t > 0.1) break;

    // === 位置PID ===
    const ep = target - pos;
    SumE_P += ep;
    SumE_P = Math.max(-40000, Math.min(40000, SumE_P));
    const ap = KP*ep + KI*SumE_P + KD*(ep - 2*LstE_P + PrvE_P);
    PrvE_P = LstE_P; LstE_P = ep;

    let po = Math.round(ap);
    po = po >= 0 ? (po >= force ? force : Math.max(po, 100))
                 : (po <= -force ? -force : Math.min(po, -100));

    // === S曲线限速 ===
    const dr = target - pos;
    let sc = null;
    if (dr < pulse_buf && pulse_buf > 0) {
        const ts = Math.max(0, dr / pulse_buf);
        const sm = ts*ts*(3 - 2*ts); // smoothstep
        sc = time_c + (time_buf - time_c)*sm;
        const lim = Math.round(sc);
        if (po > lim) po = lim;
        if (po < -lim) po = -lim;
    }
    const ssp = po; // SetPoint_S

    // === 速度PID (增量式) ===
    const es = ssp - speed_f;
    AccS += SKP*(es - LstE_S) + SKI*es + SKD*(es - 2*LstE_S + PrvE_S);
    AccS = Math.max(-3000, Math.min(3000, AccS));
    PrvE_S = LstE_S; LstE_S = es;

    let pwm = Math.round(AccS);
    pwm = Math.max(-force_F, Math.min(force_F, pwm));

    // === 电机物理模型 ===
    const ss = K_motor * pwm;
    const alpha = Math.min(1, dt/tau_s);
    speed = speed + alpha*(ss - speed);

    // === 速度测量滤波 ===
    sp_buf[sp_idx] = speed;
    sp_idx = (sp_idx + 1) % 10;
    if (sp_idx === 0) {
        const srt = [...sp_buf].sort((a,b)=>a-b);
        const tmp = srt.slice(2,8).reduce((a,b)=>a+b,0)/6;
        speed_f = 0.48*tmp + 0.52*speed_f_prev;
        speed_f_prev = speed_f;
    }
    if (Math.abs(speed_f) < 1e-9) speed_f = speed;

    // === 位置更新 (speed单位=counts/5ms, dt=25ms=5个周期) ===
    pos += speed * 5.0;

    recs.push({t:t*1000, pwm, po, speed, ssp, pos, sc, ep, es});
}

const N = recs.length, T = recs[N-1].t;
const maxPWM = Math.max(...recs.map(r=>r.pwm));

// ============== 控制台输出 ==============
console.log('╔══════════════════════════════════════════════════════════╗');
console.log('║    理论PWM曲线 — 串级PID + S曲线限速 — 仿真结果        ║');
console.log('╠══════════════════════════════════════════════════════════╣');
console.log(`║  总步数: ${N}  (${(T/1000).toFixed(2)}s)                                  `);
console.log(`║  PWM: ${Math.min(...recs.map(r=>r.pwm))} ~ ${maxPWM}                                     `);
console.log(`║  速度: ${Math.min(...recs.map(r=>r.speed)).toFixed(0)} ~ ${Math.max(...recs.map(r=>r.speed)).toFixed(0)} counts/5ms`);
console.log(`║  位置: 0 → ${recs[N-1].pos.toFixed(0)} (目标=${target})`);
console.log('╚══════════════════════════════════════════════════════════╝');

// 关键阶段分析
console.log('\n=== 运动阶段分析 ===');
const phases = [
    {label:'① 加速段', cond: r => r.pwm < 1000, desc: '速度PID积分爬升，PWM快速增长'},
    {label:'② 全速段', cond: r => r.sc === null, desc: '位置PID饱和输出800，S曲线未触发'},
    {label:'③ S曲线起点', cond: r => r.sc !== null, desc: `dist=${pulse_buf}，speed_limit=${time_buf}`},
    {label:'④ S曲线中段', cond: r => r.sc !== null && r.sc < 300, desc: 'limit降至300以下'},
    {label:'⑤ S曲线末端', cond: r => r.sc !== null && r.sc < 165, desc: 'limit接近time_c=140'},
    {label:'⑥ 到达目标', cond: r => r.pos >= target-5, desc: '运动完成'},
];

let lastPhase = -1;
for (const r of recs) {
    for (let pi = lastPhase+1; pi < phases.length; pi++) {
        if (phases[pi].cond(r)) {
            console.log(`  ${r.t.toFixed(0).padStart(4)}ms | PWM=${String(r.pwm).padStart(4)} | Sp=${r.ssp.toFixed(0).padStart(4)} | spd=${r.speed.toFixed(0).padStart(4)} | pos=${r.pos.toFixed(0).padStart(5)} | ${phases[pi].label}: ${phases[pi].desc}`);
            lastPhase = pi;
            break;
        }
    }
    if (lastPhase >= phases.length-1) break;
}

// ASCII曲线
console.log('\n=== PWM vs 时间 (每个字符=1个PID周期25ms, 背景▓=S曲线区域) ===');
for (let i = 0; i < N; i++) {
    const r = recs[i];
    const bar = Math.round(r.pwm / maxPWM * 50);
    const bg = r.sc !== null ? '\x1b[43m \x1b[0m' : ' ';
    const barStr = '\x1b[36m' + '█'.repeat(Math.max(0, bar)) + '\x1b[0m';
    console.log(`${r.t.toFixed(0).padStart(4)}ms ${barStr}${' '.repeat(Math.max(0, 50-bar))}${bg} PWM=${String(r.pwm).padStart(4)} Sp=${r.ssp.toFixed(0).padStart(4)} pos=${r.pos.toFixed(0).padStart(5)}`);
}

// ============== CSV ==============
let csv = 'time_ms,pwm,pwm_pos,spd_sp,spd_actual,pos,s_curve,err_pos,err_spd\n';
for (const r of recs)
    csv += `${r.t.toFixed(1)},${r.pwm},${r.po},${r.ssp.toFixed(1)},${r.speed.toFixed(1)},${r.pos.toFixed(1)},${r.sc!==null?r.sc.toFixed(1):''},${r.ep.toFixed(1)},${r.es.toFixed(1)}\n`;
fs.writeFileSync('pwm_data.csv', csv);

// ============== SVG图 ==============
const W = 1400, H = 1000;
const ML = 105, MR = 55, MT = 55, MB = 60;
const PW = W - ML - MR;
const ROWS = 5, RH = (H - MT - MB) / ROWS;

function xc(t) { return ML + (t/T)*PW; }
function yv(row, v, vmax) { return MT + row*RH + RH*(1 - v/vmax); }

let S = [];
const A = (s) => S.push(s);

A(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" font-family="Consolas,monospace">`);
A(`<rect width="${W}" height="${H}" fill="#0d1117"/>`);
A(`<text x="${W/2}" y="28" fill="#e6edf3" text-anchor="middle" font-size="14" font-weight="bold">理论PWM曲线 — 串级PID + S曲线限速 &nbsp; [F=800 R=3500 V=140 BS=30000 BV=450] &nbsp; PID: KP=0.150 KI=0.005 KD=0.015 SKP=1.300 SKI=0.850</text>`);

// 网格工具
function grid(r, vmax, yTicks) {
    for (const yt of yTicks) {
        const yi = yv(r, yt.v, vmax);
        A(`<line x1="${ML}" y1="${yi.toFixed(1)}" x2="${W-MR}" y2="${yi.toFixed(1)}" stroke="#21262d" stroke-width="0.5"/>`);
        A(`<text x="${ML-8}" y="${yi.toFixed(1)+4}" fill="#8b949e" font-size="10" text-anchor="end">${yt.l || yt.v}</text>`);
    }
    for (let tt = 0; tt <= T + 50; tt += 100) {
        A(`<line x1="${xc(tt).toFixed(1)}" y1="${MT+r*RH}" x2="${xc(tt).toFixed(1)}" y2="${MT+(r+1)*RH}" stroke="#21262d" stroke-width="0.5"/>`);
        A(`<text x="${xc(tt).toFixed(1)}" y="${MT+(r+1)*RH+14}" fill="#8b949e" font-size="10" text-anchor="middle">${tt}</text>`);
    }
}

function line(r, fn, vmax, color, w, dash, filterFn) {
    let d = '', first = true;
    for (const rec of recs) {
        const v = filterFn ? filterFn(rec) : fn(rec);
        if (v === null || v === undefined || isNaN(v)) { first = true; continue; }
        const yy = yv(r, v, vmax);
        d += first ? `M${xc(rec.t).toFixed(1)},${yy.toFixed(1)}` : `L${xc(rec.t).toFixed(1)},${yy.toFixed(1)}`;
        first = false;
    }
    A(`<path d="${d}" stroke="${color}" stroke-width="${w}" fill="none"${dash ? ` stroke-dasharray="${dash}"` : ''}/>`);
}

function hline(r, val, vmax, color, dash, label) {
    const yi = yv(r, val, vmax);
    A(`<line x1="${ML}" y1="${yi.toFixed(1)}" x2="${W-MR}" y2="${yi.toFixed(1)}" stroke="${color}" stroke-width="1" stroke-dasharray="${dash}"/>`);
    if (label) A(`<text x="${W-MR-5}" y="${yi.toFixed(1)-6}" fill="${color}" font-size="10" text-anchor="end">${label}</text>`);
}

function label(r, xoff, text, color, sz) {
    A(`<text x="${ML+xoff}" y="${MT+r*RH+16}" fill="${color || '#e6edf3'}" font-size="${sz || 12}">${text}</text>`);
}

// Row 0: PWM输出
let row = 0;
A(`<text x="8" y="${yv(row,0,4000)+RH/2}" fill="#e6edf3" font-size="12" text-anchor="middle" transform="rotate(-90,8,${yv(row,0,4000)+RH/2})">PWM输出</text>`);
grid(row, 5000, [{v:0,l:'0'},{v:1000},{v:2000},{v:3000,l:'3000(速度环限幅)'},{v:3500,l:'force_F=3500'},{v:4000}]);
hline(row, 3500, 5000, '#f85149', '6,3', 'force_F=3500');
hline(row, 3000, 5000, '#d2991d', '4,4', '速度环限幅3000');
line(row, r=>r.pwm, 5000, '#58a6ff', 1.8);
label(row, 8, '━━ motor_pwm (最终输出)', '#58a6ff');

// Row 1: 速度设定 vs 实际
row = 1;
A(`<text x="8" y="${yv(row,0,1000)+RH/2}" fill="#e6edf3" font-size="12" text-anchor="middle" transform="rotate(-90,8,${yv(row,0,1000)+RH/2})">速度</text>`);
grid(row, 1000, [{v:0},{v:200},{v:400},{v:600},{v:800,l:'force=800'}]);
hline(row, 800, 1000, '#3fb950', '6,3', 'force=800');
line(row, r=>r.ssp, 1000, '#f0883e', 1.8);
line(row, r=>r.speed, 1000, '#79c0ff', 1.3);
label(row, 8, '━━ SetPoint_S(速度设定)', '#f0883e');
label(row, 230, '━━ 实际速度 (counts/5ms)', '#79c0ff');

// Row 2: S曲线限速值
row = 2;
A(`<text x="8" y="${yv(row,0,600)+RH/2}" fill="#e6edf3" font-size="12" text-anchor="middle" transform="rotate(-90,8,${yv(row,0,600)+RH/2})">S曲线</text>`);
grid(row, 600, [{v:0},{v:140,l:'time_c=140'},{v:200},{v:300},{v:450,l:'time_buf=450'},{v:600}]);
hline(row, 450, 600, '#d2991d', '4,4', 'time_buf=450');
hline(row, 140, 600, '#a371f7', '4,4', 'time_c=140');
line(row, r=>r.sc, 600, '#d2991d', 2.2, null, r=>r.sc);
const scStart = recs.find(r=>r.sc!==null);
if (scStart) A(`<rect x="${xc(scStart.t).toFixed(1)}" y="${MT+row*RH}" width="${(xc(T)-xc(scStart.t)).toFixed(1)}" height="${RH}" fill="#d2991d" opacity="0.06"/>`);
label(row, 8, '━━ S-Curve Limit = 140 + 310 × smoothstep(dist/30000)', '#d2991d');

// Row 3: 位置
row = 3;
A(`<text x="8" y="${yv(row,0,target)+RH/2}" fill="#e6edf3" font-size="12" text-anchor="middle" transform="rotate(-90,8,${yv(row,0,target)+RH/2})">位置</text>`);
grid(row, target, [{v:0},{v:9200,l:'9200'},{v:20000},{v:30000},{v:39200,l:'目标39200'}]);
hline(row, target, target, '#3fb950', '6,3', '目标=39200');
hline(row, 9200, target, '#d2991d', '3,3');
if (scStart) A(`<text x="${ML+5}" y="${yv(row,9200,target).toFixed(1)-4}" fill="#d2991d" font-size="9">dist=30000(S曲线起点)</text>`);
line(row, r=>r.pos, target, '#7ee787', 1.8);

// Row 4: 位置误差
row = 4;
A(`<text x="8" y="${yv(row,0,target)+RH/2}" fill="#e6edf3" font-size="12" text-anchor="middle" transform="rotate(-90,8,${yv(row,0,target)+RH/2})">误差</text>`);
grid(row, target, [{v:0},{v:9200},{v:20000},{v:30000,l:'30000(S起点)'},{v:39200}]);
line(row, r=>r.ep, target, '#d2a8ff', 1.3);
label(row, 8, '━━ 位置误差 = SetPoint_P - location', '#d2a8ff');

// X轴
A(`<text x="${W/2}" y="${H-5}" fill="#8b949e" font-size="12" text-anchor="middle">时间 (ms) &nbsp;—&nbsp; PID采样周期 = 25ms &nbsp;|&nbsp; 总运动时间 ≈ ${T.toFixed(0)}ms</text>`);

// 阶段标注 (在PWM图上)
const pStarts = [];
{
    let ri = 0;
    while (ri < N && recs[ri].pwm < 500) ri++;
    if (ri < N) pStarts.push({t:recs[ri].t, label:'加速'});
    while (ri < N && recs[ri].sc === null) ri++;
    if (ri < N) pStarts.push({t:recs[ri].t, label:'S曲线减速开始'});
    ri = N - 1;
    pStarts.push({t:recs[ri].t, label:'停止'});
}
for (const ps of pStarts) {
    A(`<line x1="${xc(ps.t).toFixed(1)}" y1="${MT}" x2="${xc(ps.t).toFixed(1)}" y2="${MT+ROWS*RH}" stroke="#f0f6fc" stroke-width="0.6" stroke-dasharray="3,3" opacity="0.4"/>`);
    A(`<text x="${xc(ps.t).toFixed(1)+4}" y="${MT+12}" fill="#f0f6fc" font-size="10" opacity="0.7">${ps.label}</text>`);
}

A('</svg>');
fs.writeFileSync('pwm_curve.svg', S.join('\n'));
console.log('\n✅ 已保存: pwm_data.csv , pwm_curve.svg');
