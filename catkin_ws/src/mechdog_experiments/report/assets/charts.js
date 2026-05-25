/**
 * charts.js — Graficos para el reporte experimental de MechDog
 * Genera graficos de barras y metricas a partir de resultados JSON
 */

(async function() {
    try {
        const response = await fetch('../results.json');
        if (!response.ok) throw new Error('No results yet');
        const results = await response.json();
        renderCharts(results);
    } catch (e) {
        console.log('No experiment results found, using sample data');
        renderSampleData();
    }
})();

function renderSampleData() {
    const sampleResults = [
        { algorithm: 'astar', scenario: 'simple', trial:1,
          goal_1: { success: true, execution_time: 12.5 },
          metrics: { path_length:8.2, emergency_stops:1, recovery_attempts:0, map_coverage:72, nodes_explored:180, collision_avoidance_rate:0.92, total_distance:9.1, max_velocity:0.42 } },
        { algorithm: 'astar', scenario: 'simple', trial:2,
          goal_1: { success: true, execution_time: 11.2 },
          metrics: { path_length:7.8, emergency_stops:0, recovery_attempts:0, map_coverage:75, nodes_explored:165, collision_avoidance_rate:0.95, total_distance:8.5, max_velocity:0.45 } },
        { algorithm: 'astar', scenario: 'medium', trial:1,
          goal_1: { success: true, execution_time: 28.3 },
          metrics: { path_length:15.4, emergency_stops:3, recovery_attempts:1, map_coverage:68, nodes_explored:420, collision_avoidance_rate:0.88, total_distance:17.2, max_velocity:0.38 } },
        { algorithm: 'astar', scenario: 'medium', trial:2,
          goal_1: { success: true, execution_time: 25.7 },
          metrics: { path_length:14.1, emergency_stops:2, recovery_attempts:0, map_coverage:71, nodes_explored:390, collision_avoidance_rate:0.91, total_distance:15.8, max_velocity:0.41 } },
        { algorithm: 'astar', scenario: 'complex', trial:1,
          goal_1: { success: true, execution_time: 45.1 },
          metrics: { path_length:22.7, emergency_stops:5, recovery_attempts:2, map_coverage:65, nodes_explored:680, collision_avoidance_rate:0.82, total_distance:26.3, max_velocity:0.35 } },
        { algorithm: 'astar', scenario: 'complex', trial:2,
          goal_1: { success: false, execution_time: 120.0 },
          metrics: { path_length:25.1, emergency_stops:8, recovery_attempts:4, map_coverage:60, nodes_explored:720, collision_avoidance_rate:0.75, total_distance:31.5, max_velocity:0.32 } },
        { algorithm: 'bfs', scenario: 'simple', trial:1,
          goal_1: { success: true, execution_time: 18.7 },
          metrics: { path_length:10.3, emergency_stops:2, recovery_attempts:1, map_coverage:70, nodes_explored:310, collision_avoidance_rate:0.90, total_distance:11.5, max_velocity:0.35 } },
        { algorithm: 'bfs', scenario: 'simple', trial:2,
          goal_1: { success: true, execution_time: 20.1 },
          metrics: { path_length:11.1, emergency_stops:1, recovery_attempts:0, map_coverage:73, nodes_explored:290, collision_avoidance_rate:0.93, total_distance:12.8, max_velocity:0.37 } },
        { algorithm: 'bfs', scenario: 'medium', trial:1,
          goal_1: { success: true, execution_time: 42.5 },
          metrics: { path_length:20.8, emergency_stops:4, recovery_attempts:2, map_coverage:66, nodes_explored:580, collision_avoidance_rate:0.85, total_distance:24.1, max_velocity:0.32 } },
        { algorithm: 'bfs', scenario: 'medium', trial:2,
          goal_1: { success: true, execution_time: 38.9 },
          metrics: { path_length:19.2, emergency_stops:3, recovery_attempts:1, map_coverage:69, nodes_explored:540, collision_avoidance_rate:0.87, total_distance:22.6, max_velocity:0.34 } },
        { algorithm: 'bfs', scenario: 'complex', trial:1,
          goal_1: { success: true, execution_time: 72.3 },
          metrics: { path_length:30.5, emergency_stops:7, recovery_attempts:3, map_coverage:62, nodes_explored:890, collision_avoidance_rate:0.78, total_distance:36.9, max_velocity:0.28 } },
        { algorithm: 'bfs', scenario: 'complex', trial:2,
          goal_1: { success: false, execution_time: 120.0 },
          metrics: { path_length:33.2, emergency_stops:10, recovery_attempts:5, map_coverage:58, nodes_explored:950, collision_avoidance_rate:0.72, total_distance:42.1, max_velocity:0.25 } },
    ];
    renderCharts(sampleResults);
    document.getElementById('metricGrid').innerHTML =
        '<div class="metric-card placeholder"><div class="value">📊</div><div class="label">Datos de ejemplo — ejecuta experimentos reales</div></div>';
}

function renderCharts(results) {
    const stats = computeStats(results);
    renderSummaryCards(stats);
    renderScenarioTable(stats, results);
    renderSuccessChart(stats);
    renderTimeChart(stats);
    renderSuccessByScenarioChart(stats);
    renderTimeByScenarioChart(stats);
    renderPathLengthChart(stats);
    renderEmergencyStopsChart(stats);
    renderRecoveryAttemptsChart(stats);
    renderMapCoverageChart(stats);
    renderNodesExploredChart(stats);
    renderCollisionAvoidanceChart(stats);
    renderVerdict(stats);
}

function computeStats(results) {
    const byAlgo = {
        astar: { times: [], successes: 0, total: 0, metrics: [] },
        bfs: { times: [], successes: 0, total: 0, metrics: [] }
    };
    const byScenario = {};

    const avg = arr => arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0;
    const std = arr => arr.length > 1 ? Math.sqrt(arr.reduce((s, v) => s + (v - avg(arr)) ** 2, 0) / (arr.length - 1)) : 0;

    const sumMetrics = (metricsList, field) => {
        const vals = metricsList.map(m => m && m[field] !== undefined ? m[field] : null).filter(v => v !== null);
        return vals.length ? avg(vals) : 0;
    };

    results.forEach(r => {
        const algo = r.algorithm;
        const scenario = r.scenario;

        if (!byScenario[scenario]) {
            byScenario[scenario] = {
                astar: { times: [], successes: 0, total: 0, goals: [], metrics: [] },
                bfs: { times: [], successes: 0, total: 0, goals: [], metrics: [] }
            };
        }

        Object.keys(r).forEach(k => {
            if (k.startsWith('goal_') && typeof r[k] === 'object') {
                const g = r[k];
                byAlgo[algo].total++;
                byScenario[scenario][algo].total++;
                if (g.success) {
                    byAlgo[algo].successes++;
                    byAlgo[algo].times.push(g.execution_time || 0);
                    byScenario[scenario][algo].successes++;
                    byScenario[scenario][algo].times.push(g.execution_time || 0);
                }
                byScenario[scenario][algo].goals.push(g);
            }
        });

        if (r.metrics) {
            byAlgo[algo].metrics.push(r.metrics);
            byScenario[scenario][algo].metrics.push(r.metrics);
        }
    });

    const allScenarios = Object.keys(byScenario).sort();
    const scenarioNames = { simple: 'Simple', medium: 'Medium', complex: 'Complex' };

    return { byAlgo, byScenario, avg, std, sumMetrics, allScenarios, scenarioNames };
}

function renderSummaryCards(stats) {
    const { byAlgo, avg, std } = stats;
    const grid = document.getElementById('metricGrid');

    const cards = [
        { value: `${byAlgo.astar.successes}/${byAlgo.astar.total}`, label: 'A* - Goals Completados' },
        { value: `${byAlgo.bfs.successes}/${byAlgo.bfs.total}`, label: 'BFS - Goals Completados' },
        { value: `${byAlgo.astar.total ? (byAlgo.astar.successes / byAlgo.astar.total * 100).toFixed(1) : 0}%`, label: 'A* - Tasa de Exito' },
        { value: `${byAlgo.bfs.total ? (byAlgo.bfs.successes / byAlgo.bfs.total * 100).toFixed(1) : 0}%`, label: 'BFS - Tasa de Exito' },
        { value: `${avg(byAlgo.astar.times).toFixed(2)}s`, label: 'A* - Tiempo Promedio' },
        { value: `${avg(byAlgo.bfs.times).toFixed(2)}s`, label: 'BFS - Tiempo Promedio' },
        { value: `±${std(byAlgo.astar.times).toFixed(2)}s`, label: 'A* - Desviacion Estandar' },
        { value: `±${std(byAlgo.bfs.times).toFixed(2)}s`, label: 'BFS - Desviacion Estandar' },
    ];

    grid.innerHTML = cards.map(c =>
        `<div class="metric-card"><div class="value">${c.value}</div><div class="label">${c.label}</div></div>`
    ).join('');
}

function renderScenarioTable(stats, results) {
    const { byScenario, avg, sumMetrics } = stats;
    const table = document.getElementById('scenarioTable');

    if (Object.keys(byScenario).length === 0) {
        table.innerHTML = '<tr><th>Escenario</th><th>A* - Exito</th><th>A* - Tiempo</th><th>A* - Path</th><th>A* - E-Stops</th><th>BFS - Exito</th><th>BFS - Tiempo</th><th>BFS - Path</th><th>BFS - E-Stops</th><th>Ganador</th></tr><tr><td colspan="10">Sin datos</td></tr>';
        return;
    }

    let html = '<tr><th>Escenario</th><th>A* Exito</th><th>A* T(s)</th><th>A* Path(m)</th><th>A* E-Stop</th><th>BFS Exito</th><th>BFS T(s)</th><th>BFS Path(m)</th><th>BFS E-Stop</th><th>Ganador</th></tr>';

    Object.entries(byScenario).forEach(([name, data]) => {
        const aRate = data.astar.total ? (data.astar.successes / data.astar.total * 100).toFixed(0) : 0;
        const bRate = data.bfs.total ? (data.bfs.successes / data.bfs.total * 100).toFixed(0) : 0;
        const aTime = avg(data.astar.times).toFixed(2);
        const bTime = avg(data.bfs.times).toFixed(2);
        const aPath = sumMetrics(data.astar.metrics, 'path_length').toFixed(1);
        const bPath = sumMetrics(data.bfs.metrics, 'path_length').toFixed(1);
        const aEStop = sumMetrics(data.astar.metrics, 'emergency_stops').toFixed(1);
        const bEStop = sumMetrics(data.bfs.metrics, 'emergency_stops').toFixed(1);
        const winner = aRate > bRate ? 'A*' : bRate > aRate ? 'BFS' : aTime < bTime ? 'A*' : bTime < aTime ? 'BFS' : 'Empate';

        html += `<tr>
            <td><strong>${name.charAt(0).toUpperCase() + name.slice(1)}</strong></td>
            <td>${data.astar.successes}/${data.astar.total} (${aRate}%)</td>
            <td>${aTime}s</td>
            <td>${aPath}m</td>
            <td>${aEStop}</td>
            <td>${data.bfs.successes}/${data.bfs.total} (${bRate}%)</td>
            <td>${bTime}s</td>
            <td>${bPath}m</td>
            <td>${bEStop}</td>
            <td class="winner">${winner}</td>
        </tr>`;
    });

    table.innerHTML = html;
}

function drawCanvasBarChart(canvasId, title, labels, values, colors, maxVal, unit, showYAxis) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    const pad = { top: 28, bottom: 45, left: 45, right: 15 };
    const chartW = w - pad.left - pad.right;
    const chartH = h - pad.top - pad.bottom;
    const baseline = pad.top + chartH;

    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = '#e2e8f0';
    ctx.font = '12px Segoe UI, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(title, w / 2, 16);

    const n = labels.length;
    const totalGap = chartW * 0.35;
    const gapAround = totalGap / (n + 1);
    let barW = (chartW - totalGap) / n;
    if (barW < 6) barW = 6;
    if (barW > 60) barW = 60;

    const actualMax = maxVal || Math.max(...values, 1);
    const yMax = actualMax * 1.18;

    for (let i = 0; i < n; i++) {
        const x = pad.left + gapAround + i * (chartW - totalGap) / (n - 1 || 1);
        const barH = Math.max(0, (values[i] / yMax) * chartH);
        if (barH <= 0) continue;
        const grad = ctx.createLinearGradient(x, baseline, x, baseline - barH);
        grad.addColorStop(0, colors[i][0]);
        grad.addColorStop(1, colors[i][1]);
        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.roundRect(x, baseline - barH, barW, barH, 3);
        ctx.fill();

        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 11px Segoe UI, sans-serif';
        ctx.textAlign = 'center';
        const labelY = barH > 20 ? baseline - barH - 5 : baseline - barH - 15;
        ctx.fillText(values[i].toFixed(1) + unit, x + barW / 2, labelY);

        ctx.fillStyle = '#94a3b8';
        ctx.font = '10px Segoe UI, sans-serif';
        ctx.fillText(labels[i], x + barW / 2, baseline + 14);
    }

    if (showYAxis) {
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(pad.left, pad.top);
        ctx.lineTo(pad.left, baseline);
        ctx.stroke();
        for (let i = 0; i <= 4; i++) {
            const y = pad.top + chartH - (i / 4) * chartH;
            const val = (i / 4) * yMax;
            ctx.strokeStyle = '#1e293b';
            ctx.beginPath();
            ctx.moveTo(pad.left - 3, y);
            ctx.lineTo(w - pad.right, y);
            ctx.stroke();
            ctx.fillStyle = '#64748b';
            ctx.font = '9px Segoe UI, sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(val.toFixed(1), pad.left - 6, y + 3);
        }
    }
}

function renderSuccessChart(stats) {
    const { byAlgo } = stats;
    const aRate = byAlgo.astar.total ? (byAlgo.astar.successes / byAlgo.astar.total * 100) : 0;
    const bRate = byAlgo.bfs.total ? (byAlgo.bfs.successes / byAlgo.bfs.total * 100) : 0;
    drawCanvasBarChart('successChart', 'Tasa de Exito Global (%)',
        ['A*', 'BFS'], [aRate, bRate],
        [['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']], 100, '%', true);
}

function renderTimeChart(stats) {
    const { byAlgo, avg } = stats;
    const aTime = avg(byAlgo.astar.times);
    const bTime = avg(byAlgo.bfs.times);
    drawCanvasBarChart('timeChart', 'Tiempo Promedio Global (s)',
        ['A*', 'BFS'], [aTime, bTime],
        [['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']], Math.max(aTime, bTime, 1), 's', true);
}

function renderSuccessByScenarioChart(stats) {
    const { byScenario, allScenarios } = stats;
    const labels = [];
    const values = [];
    const colors = [];
    allScenarios.forEach(s => {
        const d = byScenario[s];
        const aRate = d.astar.total ? (d.astar.successes / d.astar.total * 100) : 0;
        const bRate = d.bfs.total ? (d.bfs.successes / d.bfs.total * 100) : 0;
        labels.push(`A* ${s}`, `BFS ${s}`);
        values.push(aRate, bRate);
        colors.push(['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']);
    });
    drawCanvasBarChart('successByScenarioChart', 'Tasa de Exito por Escenario (%)', labels, values, colors, 100, '%', true);
}

function renderTimeByScenarioChart(stats) {
    const { byScenario, avg, allScenarios } = stats;
    const labels = [];
    const values = [];
    const colors = [];
    allScenarios.forEach(s => {
        const d = byScenario[s];
        const aTime = avg(d.astar.times);
        const bTime = avg(d.bfs.times);
        labels.push(`A* ${s}`, `BFS ${s}`);
        values.push(aTime, bTime);
        colors.push(['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']);
    });
    const maxVal = Math.max(...values.filter(v => v > 0), 1);
    drawCanvasBarChart('timeByScenarioChart', 'Tiempo Promedio por Escenario (s)', labels, values, colors, maxVal * 1.2, 's', true);
}

function renderMetricBarChart(canvasId, title, stats, field, unit, colorA, colorB, maxOverride) {
    const { byAlgo, sumMetrics } = stats;
    const aVal = sumMetrics(byAlgo.astar.metrics, field);
    const bVal = sumMetrics(byAlgo.bfs.metrics, field);
    const maxVal = maxOverride || Math.max(aVal, bVal, 1);
    drawCanvasBarChart(canvasId, title,
        ['A*', 'BFS'], [aVal, bVal],
        [colorA, colorB], maxVal * 1.15, unit, true);
}

function renderPathLengthChart(stats) {
    renderMetricBarChart('pathLengthChart', 'Longitud de Path Promedio (m)', stats, 'path_length', 'm',
        ['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']);
}

function renderEmergencyStopsChart(stats) {
    renderMetricBarChart('emergencyStopsChart', 'Emergency Stops Promedio', stats, 'emergency_stops', '',
        ['#ef4444', '#f87171'], ['#f97316', '#fb923c']);
}

function renderRecoveryAttemptsChart(stats) {
    renderMetricBarChart('recoveryChart', 'Intentos de Recuperacion Promedio', stats, 'recovery_attempts', '',
        ['#8b5cf6', '#a78bfa'], ['#ec4899', '#f472b6']);
}

function renderMapCoverageChart(stats) {
    renderMetricBarChart('mapCoverageChart', 'Cobertura de Mapa Promedio (%)', stats, 'map_coverage', '%',
        ['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24'], 100);
}

function renderNodesExploredChart(stats) {
    renderMetricBarChart('nodesChart', 'Nodos Explorados Promedio', stats, 'nodes_explored', '',
        ['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24']);
}

function renderCollisionAvoidanceChart(stats) {
    renderMetricBarChart('collisionChart', 'Tasa de Evasion de Colisiones', stats, 'collision_avoidance_rate', '',
        ['#3b82f6', '#60a5fa'], ['#f59e0b', '#fbbf24'], 1);
}

function renderVerdict(stats) {
    const { byAlgo, avg } = stats;
    const aRate = byAlgo.astar.total ? (byAlgo.astar.successes / byAlgo.astar.total) : 0;
    const bRate = byAlgo.bfs.total ? (byAlgo.bfs.successes / byAlgo.bfs.total) : 0;
    const aTime = avg(byAlgo.astar.times);
    const bTime = avg(byAlgo.bfs.times);

    const aMetricScore = (aRate * 100) + (1 - aTime / Math.max(aTime + bTime, 1)) * 50
        + (1 - sumMetrics(byAlgo.astar.metrics, 'emergency_stops') / Math.max(sumMetrics(byAlgo.astar.metrics, 'emergency_stops') + sumMetrics(byAlgo.bfs.metrics, 'emergency_stops'), 1)) * 30
        + (1 - sumMetrics(byAlgo.astar.metrics, 'path_length') / Math.max(sumMetrics(byAlgo.astar.metrics, 'path_length') + sumMetrics(byAlgo.bfs.metrics, 'path_length'), 1)) * 20;
    const bMetricScore = (bRate * 100) + (1 - bTime / Math.max(aTime + bTime, 1)) * 50
        + (1 - sumMetrics(byAlgo.bfs.metrics, 'emergency_stops') / Math.max(sumMetrics(byAlgo.astar.metrics, 'emergency_stops') + sumMetrics(byAlgo.bfs.metrics, 'emergency_stops'), 1)) * 30
        + (1 - sumMetrics(byAlgo.bfs.metrics, 'path_length') / Math.max(sumMetrics(byAlgo.astar.metrics, 'path_length') + sumMetrics(byAlgo.bfs.metrics, 'path_length'), 1)) * 20;

    let winner, explanation;
    if (aMetricScore > bMetricScore) {
        winner = 'A*';
        explanation = `A* lidera con puntaje compuesto de ${aMetricScore.toFixed(0)} pts vs ${bMetricScore.toFixed(0)} pts de BFS. `;
        if (aRate > bRate) explanation += `Mejor tasa de exito (${(aRate*100).toFixed(1)}% vs ${(bRate*100).toFixed(1)}%). `;
        else explanation += `Tasa de exito similar (${(aRate*100).toFixed(1)}% vs ${(bRate*100).toFixed(1)}%). `;
        if (aTime < bTime) explanation += `Mas rapido (${aTime.toFixed(2)}s vs ${bTime.toFixed(2)}s).`;
    } else if (bMetricScore > aMetricScore) {
        winner = 'BFS';
        explanation = `BFS lidera con puntaje compuesto de ${bMetricScore.toFixed(0)} pts vs ${aMetricScore.toFixed(0)} pts de A*. `;
        if (bRate > aRate) explanation += `Mejor tasa de exito (${(bRate*100).toFixed(1)}% vs ${(aRate*100).toFixed(1)}%). `;
        else explanation += `Tasa de exito similar (${(aRate*100).toFixed(1)}% vs ${(bRate*100).toFixed(1)}%). `;
        if (bTime < aTime) explanation += `Mas rapido (${bTime.toFixed(2)}s vs ${aTime.toFixed(2)}s).`;
    } else {
        winner = 'Empate';
        explanation = 'Ambos algoritmos mostraron rendimiento equivalente en los escenarios probados.';
    }

    const aPath = sumMetrics(byAlgo.astar.metrics, 'path_length');
    const bPath = sumMetrics(byAlgo.bfs.metrics, 'path_length');
    const aEStop = sumMetrics(byAlgo.astar.metrics, 'emergency_stops');
    const bEStop = sumMetrics(byAlgo.bfs.metrics, 'emergency_stops');
    const aRec = sumMetrics(byAlgo.astar.metrics, 'recovery_attempts');
    const bRec = sumMetrics(byAlgo.bfs.metrics, 'recovery_attempts');
    const aCov = sumMetrics(byAlgo.astar.metrics, 'map_coverage');
    const bCov = sumMetrics(byAlgo.bfs.metrics, 'map_coverage');

    const verdict = document.getElementById('verdictBox');
    verdict.innerHTML = `
        <h3>⚡ Veredicto Final: ${winner}</h3>
        <p>${explanation}</p>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-top:1rem;text-align:left;">
            <div style="background:#0f172a;padding:0.8rem;border-radius:8px;border-left:3px solid #3b82f6;">
                <strong style="color:#60a5fa;">A*</strong>
                <ul style="font-size:0.85rem;margin-top:0.4rem;list-style:none;padding:0;">
                    <li>Exito: ${(aRate*100).toFixed(1)}%</li>
                    <li>Tiempo: ${aTime.toFixed(2)}s</li>
                    <li>Path: ${aPath.toFixed(1)}m</li>
                    <li>E-Stops: ${aEStop.toFixed(1)}</li>
                    <li>Recuperacion: ${aRec.toFixed(1)}</li>
                    <li>Cobertura: ${aCov.toFixed(1)}%</li>
                    <li>Score: ${aMetricScore.toFixed(0)} pts</li>
                </ul>
            </div>
            <div style="background:#0f172a;padding:0.8rem;border-radius:8px;border-left:3px solid #f59e0b;">
                <strong style="color:#fbbf24;">BFS</strong>
                <ul style="font-size:0.85rem;margin-top:0.4rem;list-style:none;padding:0;">
                    <li>Exito: ${(bRate*100).toFixed(1)}%</li>
                    <li>Tiempo: ${bTime.toFixed(2)}s</li>
                    <li>Path: ${bPath.toFixed(1)}m</li>
                    <li>E-Stops: ${bEStop.toFixed(1)}</li>
                    <li>Recuperacion: ${bRec.toFixed(1)}</li>
                    <li>Cobertura: ${bCov.toFixed(1)}%</li>
                    <li>Score: ${bMetricScore.toFixed(0)} pts</li>
                </ul>
            </div>
        </div>
        <p style="margin-top:0.8rem;font-size:0.9rem;color:#94a3b8;">
            <strong>Recomendacion:</strong> Para el robot real (ESP32 + ultrasonico), se recomienda ${winner === 'A*' ? 'A*' : winner === 'BFS' ? 'BFS' : 'cualquiera'} por su mejor balance entre eficiencia y seguridad.
        </p>
    `;
}

// roundRect polyfill for Canvas
if (!CanvasRenderingContext2D.prototype.roundRect) {
    CanvasRenderingContext2D.prototype.roundRect = function(x, y, w, h, r) {
        if (r > w / 2) r = w / 2;
        if (r > h / 2) r = h / 2;
        this.moveTo(x + r, y);
        this.arcTo(x + w, y, x + w, y + h, r);
        this.arcTo(x + w, y + h, x, y + h, r);
        this.arcTo(x, y + h, x, y, r);
        this.arcTo(x, y, x + w, y, r);
        return this;
    };
}
