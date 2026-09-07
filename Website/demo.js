(function () {
  const canvas = document.getElementById('canvas');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;

  // =====================================================================
  // Physical parameters — copied directly from pendsys.m
  // =====================================================================
  const params = {
    g: -9.81,             // gravitational acceleration (m/s^2), sign as in pendsys.m
    L: 0.15395,            // pivot-to-mass distance (m)
    m: 0.012,               // pendulum mass (kg)
    M: 0.061,               // cart mass (kg)
    W: 0.0381,              // cart width (m)
    H: 0.05,                // cart height (m)
    wr: 0.015,              // wheel radius (m)
    mr: 0.015,              // mass "radius" (m)
    d: 5e-4,                // cart damping coefficient
    b: 1e-6,                // pendulum pivot/bearing friction
    cd_cart: 1e-4,           // cart aerodynamic drag, quadratic in v
    cd_pend: 1e-6,           // pendulum aerodynamic drag, quadratic in w
    u_max: 2.55,             // motor force ceiling (N)
    v_max: (2 * Math.PI * 1000 / 60) * 0.006, // max cart speed at rated RPM (m/s), ~0.628
    x_min: -0.48,
    x_max: 0.48,
    x_margin: 0.08,
    k_wall: 10,
    c_wall: 2,
  };

  // =====================================================================
  // Nonlinear plant dynamics — direct port of cartpend.m
  // state y = [x, xdot, theta, thetadot]
  // =====================================================================
  function cartpend(y, u) {
    const g = params.g, L = params.L, m = params.m, M = params.M;
    const d = params.d, b = params.b, cd_cart = params.cd_cart, cd_pend = params.cd_pend;
    const Sy = Math.sin(y[2]);
    const Cy = Math.cos(y[2]);
    const J = (4 / 3) * m * L * L;                       // uniform-rod inertia about pivot
    const D = (M + m) * J - m * m * L * L * Cy * Cy;

    const F_cart = m * L * y[3] * y[3] * Sy - d * y[1] - cd_cart * y[1] * Math.abs(y[1]);
    const F_pivot = b * y[3] + cd_pend * y[3] * Math.abs(y[3]);

    return [
      y[1],
      (1 / D) * (-m * m * L * L * g * Cy * Sy + J * F_cart + m * L * Cy * F_pivot) + J * (1 / D) * u,
      y[3],
      (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * F_cart - (m + M) * F_pivot) - m * L * Cy * (1 / D) * u,
    ];
  }

  // RK4 integrator — matches henry_lqgsim.m's rk4_step
  function rk4Step(state, u, dt) {
    const add = (a, b) => a.map((v, i) => v + b[i]);
    const scale = (a, s) => a.map(v => v * s);
    const k1 = cartpend(state, u);
    const k2 = cartpend(add(state, scale(k1, dt / 2)), u);
    const k3 = cartpend(add(state, scale(k2, dt / 2)), u);
    const k4 = cartpend(add(state, scale(k3, dt)), u);
    const next = new Array(4);
    for (let i = 0; i < 4; i++) next[i] = state[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    return next;
  }

  // matches MATLAB's wrapToPi
  function wrapToPi(theta) {
    let t = (theta + Math.PI) % (2 * Math.PI);
    if (t < 0) t += 2 * Math.PI;
    return t - Math.PI;
  }

  // soft-wall bumper force — direct port of track_limit_force() in hybrid_controller_lqg.m
  function trackLimitForce(x, dx) {
    const margin = params.x_margin;
    let u_wall = 0;
    if (x > params.x_max - margin) {
      const pen = x - (params.x_max - margin);
      u_wall = -params.k_wall * pen - params.c_wall * Math.max(dx, 0);
    } else if (x < params.x_min + margin) {
      const pen = x - (params.x_min + margin);
      u_wall = -params.k_wall * pen - params.c_wall * Math.min(dx, 0);
    }
    return u_wall;
  }

  // =====================================================================
  // LQR gain K — solved offline from the exact same A, B, Q, R matrices
  // built in henry_lqgsim.m (continuous-time algebraic Riccati equation,
  // the same computation MATLAB's lqr() performs). The plant is fixed,
  // so there's no reason to re-solve a Riccati equation in the browser
  // on every load; it's baked in here, the same way you'd flash a fixed
  // gain onto a real embedded controller.
  //   K = [-141.4213562372, -101.2602502096, 356.345579351, 23.7175346685]
  // =====================================================================
  const K = [-141.4213562372, -101.2602502096, 356.345579351, 23.7175346685];

  // =====================================================================
  // Hybrid swing-up / LQR controller — port of hybrid_controller_lqg.m
  //
  // The MATLAB version drives this off a Kalman-filtered state estimate
  // (xhat) because the real rig only has noisy position/angle encoders.
  // This browser simulation already has the true state, so it feeds
  // that in directly instead of running the estimator — perfect-state
  // feedback rather than noisy-sensor feedback. The control law itself
  // (energy-shaping swing-up, the LQR gain K above, the motor speed
  // droop limit, and the soft-wall force) is unchanged.
  // =====================================================================
  function hybridController(state) {
    const theta_hat = state[2];
    const theta_wrapped = wrapToPi(theta_hat);
    const theta_target = Math.sign(theta_wrapped) * Math.PI;
    const theta_target_global = theta_hat - theta_wrapped + theta_target;

    let u;
    if (Math.abs(theta_wrapped) > (Math.PI - 0.5)) {
      // near upright — balance with the LQR gain
      const e = [state[0] - 0, state[1] - 0, state[2] - theta_target_global, state[3] - 0];
      u = -(K[0] * e[0] + K[1] * e[1] + K[2] * e[2] + K[3] * e[3]);
    } else {
      const J = (4 / 3) * params.m * params.L * params.L;
      if (Math.abs(theta_wrapped) < 0.05 && Math.abs(state[3]) < 0.1) {
        u = params.u_max; // initial kick to get it moving
      } else {
        const g_mag = Math.abs(params.g);
        const E = 0.5 * J * state[3] * state[3] - params.m * g_mag * params.L * Math.cos(theta_hat);
        const E_target = params.m * g_mag * params.L;
        const ke = 24.0, kv = 0.2;
        u = ke * (E - E_target) * state[3] * Math.cos(theta_hat) - kv * state[1];
      }
    }

    // motor speed-droop limit
    let u_max_eff;
    if (state[1] !== 0 && Math.sign(u) === Math.sign(state[1])) {
      u_max_eff = params.u_max * Math.max(0, 1 - Math.abs(state[1]) / params.v_max);
    } else {
      u_max_eff = params.u_max;
    }
    u = Math.max(Math.min(u, u_max_eff), -u_max_eff);

    // soft-wall bumper + final hard clamp
    u += trackLimitForce(state[0], state[1]);
    u = Math.max(Math.min(u, params.u_max), -params.u_max);
    return u;
  }

  // =====================================================================
  // Simulation state & stepping
  // =====================================================================
  let state = [0, 0, 0, 0];   // [x, xdot, theta, thetadot] — hanging straight down,
                                // matches y_true(1,:) = [0 0 0 0] in henry_lqgsim.m
  let controllerOn = true;
  let userForce = 0;           // external push from dragging the cart (not motor-limited)
  let lastForce = 0;           // total force applied last substep, for the arrow/readout

  let draggingCart = false;
  let draggingPendulum = false;

  const SIM_DT = 0.002;        // matches henry_lqgsim.m's integration step
  const MAX_SUBSTEPS = 30;
  const HARD_LIMIT = 0.6;      // extra safety rail beyond x_max/x_min for the browser demo,
                                 // not present in the MATLAB files, which rely on the soft wall alone

  const USER_FORCE_MAX = .6;     // a hand pushing the cart isn't limited by the motor rating
  const FORCE_SENSITIVITY = 0.001;

  function stepPhysics(frameDt) {
    if (draggingPendulum) { lastForce = 0; return; }

    const substeps = Math.min(MAX_SUBSTEPS, Math.max(1, Math.round(frameDt / SIM_DT)));
    for (let i = 0; i < substeps; i++) {
      let u_control;
      if (controllerOn) {
        u_control = hybridController(state);
      } else {
        // controller off, but the physical bumper on the rail is still there
        const u_wall = trackLimitForce(state[0], state[1]);
        u_control = Math.max(Math.min(u_wall, params.u_max), -params.u_max);
      }
      const u_total = u_control + userForce;
      lastForce = u_total;
      state = rk4Step(state, u_total, SIM_DT);

      if (state[0] > HARD_LIMIT) { state[0] = HARD_LIMIT; state[1] = 0; }
      if (state[0] < -HARD_LIMIT) { state[0] = -HARD_LIMIT; state[1] = 0; }
    }
  }

  // =====================================================================
  // Rendering
  // =====================================================================
  const pxPerMeter = 550;
  const railY = H / 2;
  const poleLenPx = params.L * pxPerMeter;   // to-scale with draw_cartpend.m (pivot-to-mass = L, not 2L)
  const cartWpx = params.W * pxPerMeter;      // to-scale
  const cartHpx = params.H * pxPerMeter;      // to-scale
  const wheelRpx = Math.max(3, (params.wr * pxPerMeter) / 2);
  const BOB_VISUAL_R = 10;                    // drawn a bit larger than true scale (~4px) for visibility
  const BOB_GRAB_R = 24;                      // and an even larger invisible grab target, for mouse/touch

  function currentPivot() {
    return { x: W / 2 + state[0] * pxPerMeter, y: railY };
  }
  function currentBob() {
    const p = currentPivot();
    return { x: p.x + Math.sin(state[2]) * poleLenPx, y: p.y + Math.cos(state[2]) * poleLenPx };
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);

    const xMaxPx = W / 2 + params.x_max * pxPerMeter;
    const xMinPx = W / 2 + params.x_min * pxPerMeter;
    const marginPx = params.x_margin * pxPerMeter;

    // rail
    ctx.strokeStyle = '#2a3348';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(xMinPx - 30, railY);
    ctx.lineTo(xMaxPx + 30, railY);
    ctx.stroke();

    // soft-wall zones (x_margin), matching the amber patches in draw_cartpend.m
    ctx.fillStyle = 'rgba(245,160,92,0.15)';
    ctx.fillRect(xMaxPx - marginPx, railY - 60, marginPx, 120);
    ctx.fillRect(xMinPx, railY - 60, marginPx, 120);

    // hard limits
    ctx.strokeStyle = 'rgba(230,90,90,0.85)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(xMaxPx, railY - 60); ctx.lineTo(xMaxPx, railY + 60);
    ctx.moveTo(xMinPx, railY - 60); ctx.lineTo(xMinPx, railY + 60);
    ctx.stroke();

    const pivot = currentPivot();
    const bob = currentBob();

    // rod
    ctx.strokeStyle = '#f5a05c';
    ctx.lineWidth = 5;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(pivot.x, pivot.y);
    ctx.lineTo(bob.x, bob.y);
    ctx.stroke();

    // bob
    ctx.fillStyle = '#f5a05c';
    ctx.beginPath();
    ctx.arc(bob.x, bob.y, BOB_VISUAL_R, 0, Math.PI * 2);
    ctx.fill();
    if (draggingPendulum) {
      ctx.strokeStyle = 'rgba(94,230,200,0.9)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(bob.x, bob.y, BOB_VISUAL_R + 6, 0, Math.PI * 2);
      ctx.stroke();
    }

    // cart
    const grad = ctx.createLinearGradient(0, pivot.y - cartHpx / 2, 0, pivot.y + cartHpx / 2);
    grad.addColorStop(0, '#7cefd6');
    grad.addColorStop(1, '#3fa892');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.roundRect(pivot.x - cartWpx / 2, pivot.y - cartHpx / 2, cartWpx, cartHpx, 4);
    ctx.fill();

    // wheels
    ctx.fillStyle = '#0a1018';
    ctx.beginPath();
    ctx.arc(pivot.x - cartWpx / 3, pivot.y + cartHpx / 2, wheelRpx, 0, Math.PI * 2);
    ctx.arc(pivot.x + cartWpx / 3, pivot.y + cartHpx / 2, wheelRpx, 0, Math.PI * 2);
    ctx.fill();

    // pivot dot
    ctx.fillStyle = '#0a1018';
    ctx.beginPath();
    ctx.arc(pivot.x, pivot.y, 3, 0, Math.PI * 2);
    ctx.fill();

    // force arrow
    if (!draggingPendulum && Math.abs(lastForce) > 0.05) {
      const dir = Math.sign(lastForce);
      const len = Math.min(Math.abs(lastForce), 5) * 16;
      ctx.strokeStyle = '#5ee6c8';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(pivot.x, pivot.y);
      ctx.lineTo(pivot.x + dir * len, pivot.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(pivot.x + dir * len, pivot.y);
      ctx.lineTo(pivot.x + dir * (len - 8), pivot.y - 5);
      ctx.lineTo(pivot.x + dir * (len - 8), pivot.y + 5);
      ctx.closePath();
      ctx.fillStyle = '#5ee6c8';
      ctx.fill();
    }

    // balanced glow
    const thetaW = wrapToPi(state[2]);
    if (Math.abs(Math.abs(thetaW) - Math.PI) < 0.05) {
      ctx.fillStyle = 'rgba(94,230,200,0.06)';
      ctx.fillRect(0, 0, W, H);
    }
  }

  function updateStats() {
    const thetaW = wrapToPi(state[2]);
    document.getElementById('angleStat').textContent = (thetaW * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('posStat').textContent = state[0].toFixed(3) + ' m';
    document.getElementById('forceStat').textContent = lastForce.toFixed(2) + ' N';
  }
// =====================================================================
  // State Graphing
  // =====================================================================
  const graphCanvas = document.getElementById('graphCanvas');
  const gCtx = graphCanvas.getContext('2d');
  const GW = graphCanvas.width, GH = graphCanvas.height;
  
  let history = []; 
  let simTime = 0;

  function drawGraph() {
    gCtx.clearRect(0, 0, GW, GH);
    
    // Draw grid & center line
    gCtx.strokeStyle = '#333';
    gCtx.lineWidth = 1;
    gCtx.beginPath();
    gCtx.moveTo(0, GH / 2); gCtx.lineTo(GW, GH / 2);
    gCtx.stroke();

    if (history.length < 2) return;

    // Viewport: y-axis [-20, 20], dynamic x-axis (rolling 10 seconds)
    const yMax = 20; 
    const tMax = Math.max(10, history[history.length - 1].t); 
    const tMin = Math.max(0, tMax - 10); 

    const mapX = (t) => ((t - tMin) / 10) * GW;
    const mapY = (val) => GH / 2 - (val / yMax) * (GH / 2);

    const colors = ['cyan', 'yellow', 'magenta', '#00ff00']; 

    for (let s = 0; s < 4; s++) {
      gCtx.strokeStyle = colors[s];
      gCtx.lineWidth = 1.5;
      gCtx.beginPath();
      
      let started = false;
      for (let i = 0; i < history.length; i++) {
        const pt = history[i];
        if (pt.t < tMin) continue; // Skip off-screen points
        
        const px = mapX(pt.t);
        const py = mapY(pt.state[s]);
        
        if (!started) { gCtx.moveTo(px, py); started = true; } 
        else { gCtx.lineTo(px, py); }
      }
      gCtx.stroke();
    }
  }
  // =====================================================================
  // Main loop
  // =====================================================================
  let lastTime = null;
  function loop(t) {
    if (lastTime === null) lastTime = t;
    const dt = Math.min((t - lastTime) / 1000, 0.05);
    lastTime = t;
    
    simTime += dt;
    stepPhysics(dt);
    
    // Record history for the graph
    history.push({ t: simTime, state: [...state] });
    // Keep array from growing infinitely (caps at ~2000 frames)
    if (history.length > 2000) history.splice(0, 500); 

    draw();
    drawGraph(); // <--- This runs the graph!
    updateStats();
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);

  // =====================================================================
  // Interaction: drag the bob to set the angle, drag the cart to push it
  // =====================================================================
  function getCanvasPoint(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: (clientX - rect.left) * (canvas.width / rect.width),
      y: (clientY - rect.top) * (canvas.height / rect.height),
    };
  }

  function updatePendulumFromPoint(pt) {
    const pivot = currentPivot();
    state[2] = Math.atan2(pt.x - pivot.x, pt.y - pivot.y); // 0 = down, ±π = upright
    state[3] = 0;
  }

  function updateForceFromPoint(pt) {
    const pivot = currentPivot();
    const diff = pt.x - pivot.x;
    userForce = Math.max(-USER_FORCE_MAX, Math.min(USER_FORCE_MAX, diff * FORCE_SENSITIVITY));
  }

  function handlePointerDown(clientX, clientY) {
    const pt = getCanvasPoint(clientX, clientY);
    const bob = currentBob();
    const dx = pt.x - bob.x, dy = pt.y - bob.y;
    if (Math.sqrt(dx * dx + dy * dy) <= BOB_GRAB_R) {
      draggingPendulum = true;
      draggingCart = false;
      updatePendulumFromPoint(pt);
    } else {
      draggingCart = true;
      draggingPendulum = false;
      updateForceFromPoint(pt);
    }
  }

  function handlePointerUp() {
    if (draggingPendulum) state[3] = 0; // released at rest — controller takes over from here
    draggingPendulum = false;
    draggingCart = false;
    userForce = 0;
  }

  canvas.addEventListener('mousedown', (e) => handlePointerDown(e.clientX, e.clientY));
  window.addEventListener('mousemove', (e) => {
    if (draggingPendulum) updatePendulumFromPoint(getCanvasPoint(e.clientX, e.clientY));
    else if (draggingCart) updateForceFromPoint(getCanvasPoint(e.clientX, e.clientY));
  });
  window.addEventListener('mouseup', handlePointerUp);

  canvas.addEventListener('touchstart', (e) => {
    const t = e.touches[0];
    handlePointerDown(t.clientX, t.clientY);
  }, { passive: true });
  canvas.addEventListener('touchmove', (e) => {
    const t = e.touches[0];
    if (draggingPendulum) updatePendulumFromPoint(getCanvasPoint(t.clientX, t.clientY));
    else if (draggingCart) updateForceFromPoint(getCanvasPoint(t.clientX, t.clientY));
    e.preventDefault();
  }, { passive: false });
  window.addEventListener('touchend', handlePointerUp);

  document.getElementById('disturbBtn').addEventListener('click', () => {
    state[3] += (Math.random() - 0.5) * 3.0;   // rad/s nudge
    state[1] += (Math.random() - 0.5) * 0.25;  // m/s nudge
  });

  document.getElementById('resetBtn').addEventListener('click', () => {
    state = [0, 0, 0, 0]; 
    userForce = 0;
    draggingPendulum = false;
    draggingCart = false;
    
    // Reset graph data
    history = [];
    simTime = 0;
  });

  const controllerBtn = document.getElementById('controllerBtn');
  controllerBtn.addEventListener('click', () => {
    controllerOn = !controllerOn;
    controllerBtn.textContent = 'Controller: ' + (controllerOn ? 'ON' : 'OFF');
  });
})();