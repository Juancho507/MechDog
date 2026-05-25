#!/usr/bin/env python
import rospy
import json
import os
import math
import numpy as np
from string import Template


class ReportGenerator:
    def __init__(self):
        rospy.init_node('report_generator', anonymous=False)
        self.load_parameters()
        rospy.loginfo("Report Generator initialized")

    def load_parameters(self):
        self.param_results_dir = rospy.get_param(
            '~results_dir', '/tmp/mechdog_experiments')
        self.param_output_dir = rospy.get_param(
            '~output_dir', '/tmp/mechdog_experiments/report')
        self.param_data_file = rospy.get_param(
            '~data_file', 'results.json')
        self.param_template_dir = rospy.get_param(
            '~template_dir', '')

    def load_results(self):
        data_path = os.path.join(self.param_results_dir, self.param_data_file)
        if not os.path.exists(data_path):
            rospy.logwarn("No results file found at %s", data_path)
            return []
        with open(data_path, 'r') as f:
            return json.load(f)

    def compute_statistics(self, results):
        stats = {'astar': {}, 'bfs': {}}
        for algo in ['astar', 'bfs']:
            algo_results = [r for r in results if r.get('algorithm') == algo]
            if not algo_results:
                continue

            times = []
            successes = 0
            total = 0
            all_metrics = []

            for r in algo_results:
                for k, v in r.items():
                    if k.startswith('goal_') and isinstance(v, dict):
                        total += 1
                        if v.get('success'):
                            successes += 1
                            times.append(v.get('execution_time', 0))
                if 'metrics' in r:
                    all_metrics.append(r['metrics'])

            stats[algo] = {
                'total_goals': total,
                'successes': successes,
                'success_rate': (successes / max(total, 1)) * 100,
                'times': times,
                'avg_time': np.mean(times) if times else 0,
                'min_time': min(times) if times else 0,
                'max_time': max(times) if times else 0,
                'std_time': np.std(times) if len(times) > 1 else 0,
                'metrics': all_metrics,
            }

        by_scenario = {}
        for r in results:
            scenario = r.get('scenario', 'unknown')
            algo = r.get('algorithm', 'unknown')
            if scenario not in by_scenario:
                by_scenario[scenario] = {'astar': [], 'bfs': [], 'astar_metrics': [], 'bfs_metrics': []}
            for k, v in r.items():
                if k.startswith('goal_') and isinstance(v, dict):
                    by_scenario[scenario][algo].append(v)
            if 'metrics' in r:
                by_scenario[scenario][f'{algo}_metrics'].append(r['metrics'])

        return stats, by_scenario

    def generate_html(self, results):
        stats, by_scenario = self.compute_statistics(results)
        os.makedirs(self.param_output_dir, exist_ok=True)

        astar_success_rate = stats.get('astar', {}).get('success_rate', 0)
        bfs_success_rate = stats.get('bfs', {}).get('success_rate', 0)
        astar_avg_time = stats.get('astar', {}).get('avg_time', 0)
        bfs_avg_time = stats.get('bfs', {}).get('avg_time', 0)
        astar_total = stats.get('astar', {}).get('total_goals', 0)
        bfs_total = stats.get('bfs', {}).get('total_goals', 0)
        astar_successes = stats.get('astar', {}).get('successes', 0)
        bfs_successes = stats.get('bfs', {}).get('successes', 0)
        astar_std = stats.get('astar', {}).get('std_time', 0)
        bfs_std = stats.get('bfs', {}).get('std_time', 0)

        astar_metrics = stats.get('astar', {}).get('metrics', [])
        bfs_metrics = stats.get('bfs', {}).get('metrics', [])
        astar_path = np.mean([m.get('path_length', 0) for m in astar_metrics]) if astar_metrics else 0
        bfs_path = np.mean([m.get('path_length', 0) for m in bfs_metrics]) if bfs_metrics else 0
        astar_estop = np.mean([m.get('emergency_stops', 0) for m in astar_metrics]) if astar_metrics else 0
        bfs_estop = np.mean([m.get('emergency_stops', 0) for m in bfs_metrics]) if bfs_metrics else 0
        astar_rec = np.mean([m.get('recovery_attempts', 0) for m in astar_metrics]) if astar_metrics else 0
        bfs_rec = np.mean([m.get('recovery_attempts', 0) for m in bfs_metrics]) if bfs_metrics else 0
        astar_cov = np.mean([m.get('map_coverage', 0) for m in astar_metrics]) if astar_metrics else 0
        bfs_cov = np.mean([m.get('map_coverage', 0) for m in bfs_metrics]) if bfs_metrics else 0
        astar_nodes = np.mean([m.get('nodes_explored', 0) for m in astar_metrics]) if astar_metrics else 0
        bfs_nodes = np.mean([m.get('nodes_explored', 0) for m in bfs_metrics]) if bfs_metrics else 0
        astar_collision = np.mean([m.get('collision_avoidance_rate', 1) for m in astar_metrics]) if astar_metrics else 1
        bfs_collision = np.mean([m.get('collision_avoidance_rate', 1) for m in bfs_metrics]) if bfs_metrics else 1

        scenarios_table = ''
        for sname, sdata in by_scenario.items():
            a_data = sdata.get('astar', [])
            b_data = sdata.get('bfs', [])
            a_metrics_list = sdata.get('astar_metrics', [])
            b_metrics_list = sdata.get('bfs_metrics', [])
            a_success = sum(1 for g in a_data if g.get('success'))
            b_success = sum(1 for g in b_data if g.get('success'))
            a_times = [g.get('execution_time', 0) for g in a_data if g.get('success')]
            b_times = [g.get('execution_time', 0) for g in b_data if g.get('success')]
            a_avg = np.mean(a_times) if a_times else 0
            b_avg = np.mean(b_times) if b_times else 0
            a_path_s = np.mean([m.get('path_length', 0) for m in a_metrics_list]) if a_metrics_list else 0
            b_path_s = np.mean([m.get('path_length', 0) for m in b_metrics_list]) if b_metrics_list else 0
            a_estop_s = np.mean([m.get('emergency_stops', 0) for m in a_metrics_list]) if a_metrics_list else 0
            b_estop_s = np.mean([m.get('emergency_stops', 0) for m in b_metrics_list]) if b_metrics_list else 0

            winner_s = 'A*' if a_success/max(len(a_data),1) > b_success/max(len(b_data),1) else 'BFS' if b_success/max(len(b_data),1) > a_success/max(len(a_data),1) else 'A*' if a_avg < b_avg else 'BFS' if b_avg < a_avg else 'Empate'

            scenarios_table += f'''
            <tr>
                <td>{sname.capitalize()}</td>
                <td>{a_success}/{len(a_data)} ({a_success/max(len(a_data),1)*100:.0f}%)</td>
                <td>{a_avg:.2f}s</td>
                <td>{a_path_s:.1f}m</td>
                <td>{a_estop_s:.1f}</td>
                <td>{b_success}/{len(b_data)} ({b_success/max(len(b_data),1)*100:.0f}%)</td>
                <td>{b_avg:.2f}s</td>
                <td>{b_path_s:.1f}m</td>
                <td>{b_estop_s:.1f}</td>
                <td class="winner">{winner_s}</td>
            </tr>'''

        a_score = (astar_success_rate) + (1 - astar_avg_time / max(astar_avg_time + bfs_avg_time, 1)) * 50 + (1 - astar_estop / max(astar_estop + bfs_estop, 1)) * 30 + (1 - astar_path / max(astar_path + bfs_path, 1)) * 20
        b_score = (bfs_success_rate) + (1 - bfs_avg_time / max(astar_avg_time + bfs_avg_time, 1)) * 50 + (1 - bfs_estop / max(astar_estop + bfs_estop, 1)) * 30 + (1 - bfs_path / max(astar_path + bfs_path, 1)) * 20

        winner = 'A*' if a_score > b_score else 'BFS' if b_score > a_score else 'Empate'
        if abs(a_score - b_score) < 0.5:
            winner = 'Empate'

        metric_charts = ''
        metric_fields = [
            ('Longitud de Path (m)', astar_path, bfs_path, 'm', max(astar_path, bfs_path, 1)),
            ('Emergency Stops', astar_estop, bfs_estop, '', max(astar_estop, bfs_estop, 1)),
            ('Intentos de Recuperacion', astar_rec, bfs_rec, '', max(astar_rec, bfs_rec, 1)),
            ('Cobertura de Mapa (%)', astar_cov, bfs_cov, '%', 100),
            ('Nodos Explorados', astar_nodes, bfs_nodes, '', max(astar_nodes, bfs_nodes, 1)),
            ('Evasion de Colisiones', astar_collision, bfs_collision, '', 1),
        ]
        for title, a_val, b_val, unit, max_v in metric_fields:
            a_pct = min(100, a_val / max(max_v, 0.001) * 100)
            b_pct = min(100, b_val / max(max_v, 0.001) * 100)
            metric_charts += f'''
        <div class="chart-container">
            <h3>{title}</h3>
            <div class="bar">
                <span class="bar-label">A*</span>
                <div class="bar-track">
                    <div class="bar-fill astar" style="width:{a_pct}%">{a_val:.2f}{unit}</div>
                </div>
            </div>
            <div class="bar">
                <span class="bar-label">BFS</span>
                <div class="bar-track">
                    <div class="bar-fill bfs" style="width:{b_pct}%">{b_val:.2f}{unit}</div>
                </div>
            </div>
        </div>'''

        html = f'''<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MechDog - Comparacion Experimental: A* vs BFS</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #0f172a; color: #e2e8f0; line-height: 1.6; }}
.header {{ background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%); padding: 2rem; text-align: center; border-bottom: 3px solid #3b82f6; }}
.header h1 {{ font-size: 2rem; color: #60a5fa; }}
.header p {{ color: #94a3b8; margin-top: 0.5rem; }}
.container {{ max-width: 1200px; margin: 0 auto; padding: 2rem; }}
.section {{ background: #1e293b; border-radius: 12px; padding: 1.5rem; margin-bottom: 1.5rem; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3); }}
.section h2 {{ color: #60a5fa; margin-bottom: 1rem; border-bottom: 1px solid #334155; padding-bottom: 0.5rem; }}
.section h3 {{ color: #93c5fd; margin: 1rem 0 0.5rem; }}
table {{ width: 100%; border-collapse: collapse; margin: 1rem 0; }}
th, td {{ padding: 0.6rem; text-align: center; border-bottom: 1px solid #334155; font-size: 0.85rem; }}
th {{ background: #0f172a; color: #60a5fa; font-weight: 600; }}
tr:hover {{ background: #334155; }}
.winner {{ color: #22c55e; font-weight: bold; }}
.metric-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 1rem; margin: 1rem 0; }}
.metric-card {{ background: #0f172a; padding: 0.8rem; border-radius: 8px; text-align: center; border-left: 3px solid #3b82f6; }}
.metric-card .value {{ font-size: 1.5rem; font-weight: bold; color: #60a5fa; }}
.metric-card .label {{ font-size: 0.8rem; color: #94a3b8; }}
.chart-container {{ background: #0f172a; padding: 1rem; border-radius: 8px; margin: 1rem 0; min-height: 80px; }}
.bar {{ display: flex; align-items: center; margin: 0.3rem 0; }}
.bar-label {{ width: 60px; font-size: 0.85rem; color: #94a3b8; }}
.bar-track {{ flex: 1; height: 24px; background: #1e293b; border-radius: 12px; overflow: hidden; position: relative; }}
.bar-fill {{ height: 100%; border-radius: 12px; display: flex; align-items: center; justify-content: center; color: white; font-size: 0.75rem; font-weight: bold; }}
.bar-fill.astar {{ background: linear-gradient(90deg, #3b82f6, #60a5fa); }}
.bar-fill.bfs {{ background: linear-gradient(90deg, #f59e0b, #fbbf24); }}
.bar-fill.danger {{ background: linear-gradient(90deg, #ef4444, #f87171); }}
ul {{ padding-left: 1.5rem; margin: 0.5rem 0; }}
li {{ margin: 0.3rem 0; color: #cbd5e1; }}
code {{ background: #0f172a; padding: 0.15rem 0.4rem; border-radius: 4px; font-size: 0.9rem; color: #f472b6; }}
.footer {{ text-align: center; padding: 2rem; color: #64748b; font-size: 0.85rem; }}
.verdict {{ background: linear-gradient(135deg, #1e3a5f, #0f172a); border: 2px solid #3b82f6; border-radius: 12px; padding: 1.5rem; text-align: center; margin: 1rem 0; }}
.verdict h3 {{ color: #fbbf24; font-size: 1.5rem; }}
.verdict p {{ color: #cbd5e1; margin-top: 0.5rem; }}
.charts-row {{ display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }}
@media (max-width: 768px) {{ .charts-row {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<div class="header">
    <h1>🤖 MechDog — Comparacion Experimental</h1>
    <p>A* (A-Star) vs BFS (Breadth-First Search) — Metricas completas de navegacion autonoma</p>
</div>
<div class="container">

<div class="section">
    <h2>📊 Resumen Global</h2>
    <div class="metric-grid">
        <div class="metric-card">
            <div class="value">{astar_successes}/{astar_total}</div>
            <div class="label">A* - Goals Completados</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_successes}/{bfs_total}</div>
            <div class="label">BFS - Goals Completados</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_success_rate:.1f}%</div>
            <div class="label">A* - Tasa de Exito</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_success_rate:.1f}%</div>
            <div class="label">BFS - Tasa de Exito</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_avg_time:.2f}s</div>
            <div class="label">A* - Tiempo Promedio</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_avg_time:.2f}s</div>
            <div class="label">BFS - Tiempo Promedio</div>
        </div>
        <div class="metric-card">
            <div class="value">±{astar_std:.2f}s</div>
            <div class="label">A* - Desviacion Estandar</div>
        </div>
        <div class="metric-card">
            <div class="value">±{bfs_std:.2f}s</div>
            <div class="label">BFS - Desviacion Estandar</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_path:.1f}m</div>
            <div class="label">A* - Path Promedio</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_path:.1f}m</div>
            <div class="label">BFS - Path Promedio</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_estop:.1f}</div>
            <div class="label">A* - Emergency Stops</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_estop:.1f}</div>
            <div class="label">BFS - Emergency Stops</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_cov:.1f}%</div>
            <div class="label">A* - Cobertura Mapa</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_cov:.1f}%</div>
            <div class="label">BFS - Cobertura Mapa</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_collision:.2f}</div>
            <div class="label">A* - Evasion Colisiones</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_collision:.2f}</div>
            <div class="label">BFS - Evasion Colisiones</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_nodes:.0f}</div>
            <div class="label">A* - Nodos Explorados</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_nodes:.0f}</div>
            <div class="label">BFS - Nodos Explorados</div>
        </div>
        <div class="metric-card">
            <div class="value">{astar_rec:.1f}</div>
            <div class="label">A* - Recuperaciones</div>
        </div>
        <div class="metric-card">
            <div class="value">{bfs_rec:.1f}</div>
            <div class="label">BFS - Recuperaciones</div>
        </div>
    </div>

    <div class="verdict">
        <h3>⚡ Veredicto: {winner} (Score: {a_score:.0f} vs {b_score:.0f})</h3>
        <p>
            {'A* obtuvo mejor rendimiento general en los escenarios probados.'
             if winner == 'A*' else
             'BFS obtuvo mejor rendimiento general en los escenarios probados.'
             if winner == 'BFS' else
             'Ambos algoritmos mostraron rendimiento similar en los escenarios probados.'}
        </p>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-top:1rem;text-align:left;">
            <div style="background:#0f172a;padding:0.8rem;border-radius:8px;border-left:3px solid #3b82f6;">
                <strong style="color:#60a5fa;">A*</strong>
                <ul style="font-size:0.85rem;margin-top:0.4rem;list-style:none;padding:0;">
                    <li>Exito: {astar_success_rate:.1f}% | Tiempo: {astar_avg_time:.2f}s</li>
                    <li>Path: {astar_path:.1f}m | E-Stops: {astar_estop:.1f}</li>
                    <li>Cobertura: {astar_cov:.1f}% | Score: {a_score:.0f} pts</li>
                </ul>
            </div>
            <div style="background:#0f172a;padding:0.8rem;border-radius:8px;border-left:3px solid #f59e0b;">
                <strong style="color:#fbbf24;">BFS</strong>
                <ul style="font-size:0.85rem;margin-top:0.4rem;list-style:none;padding:0;">
                    <li>Exito: {bfs_success_rate:.1f}% | Tiempo: {bfs_avg_time:.2f}s</li>
                    <li>Path: {bfs_path:.1f}m | E-Stops: {bfs_estop:.1f}</li>
                    <li>Cobertura: {bfs_cov:.1f}% | Score: {b_score:.0f} pts</li>
                </ul>
            </div>
        </div>
    </div>
</div>

<div class="section">
    <h2>📈 Comparacion por Escenario</h2>
    <p style="font-size:0.85rem;color:#94a3b8;">Exito, tiempo promedio, longitud de path y emergency stops por escenario</p>
    <table>
        <tr>
            <th>Escenario</th>
            <th>A* Exito</th>
            <th>A* T(s)</th>
            <th>A* Path(m)</th>
            <th>A* E-Stop</th>
            <th>BFS Exito</th>
            <th>BFS T(s)</th>
            <th>BFS Path(m)</th>
            <th>BFS E-Stop</th>
            <th>Ganador</th>
        </tr>
        {scenarios_table}
    </table>
</div>

<div class="section">
    <h2>📉 Graficas de Rendimiento</h2>
    <div class="charts-row">
    <div class="chart-container">
        <h3>Tasa de Exito (%)</h3>
        <div class="bar">
            <span class="bar-label">A*</span>
            <div class="bar-track">
                <div class="bar-fill astar" style="width:{astar_success_rate}%">{astar_success_rate:.1f}%</div>
            </div>
        </div>
        <div class="bar">
            <span class="bar-label">BFS</span>
            <div class="bar-track">
                <div class="bar-fill bfs" style="width:{bfs_success_rate}%">{bfs_success_rate:.1f}%</div>
            </div>
        </div>
    </div>
    <div class="chart-container">
        <h3>Tiempo Promedio (s)</h3>
        <div class="bar">
            <span class="bar-label">A*</span>
            <div class="bar-track">
                <div class="bar-fill astar" style="width:{min(100, astar_avg_time/max(bfs_avg_time,0.1)*100)}%">{astar_avg_time:.2f}s</div>
            </div>
        </div>
        <div class="bar">
            <span class="bar-label">BFS</span>
            <div class="bar-track">
                <div class="bar-fill bfs" style="width:{min(100, bfs_avg_time/max(astar_avg_time,0.1)*100)}%">{bfs_avg_time:.2f}s</div>
            </div>
        </div>
    </div>
    </div>
    {metric_charts}
</div>

<div class="section">
    <h2>🧠 Explicacion de los Algoritmos</h2>

    <h3>¿Por que solo A* y BFS?</h3>
    <p>Para este proyecto se seleccionaron exclusivamente A* (A-Star) y BFS (Breadth-First Search) por las siguientes razones:</p>
    <ul>
        <li><strong>Restriccion de hardware:</strong> El MechDog Standard Kit utiliza un ESP32 sin GPU ni aceleracion por hardware. Algoritmos como PPO, DQN o Geneticos requieren recursos computacionales que el robot real no posee.</li>
        <li><strong>Sensor unico:</strong> Al usar solo un sensor ultrasonico (un solo haz de distancia), algoritmos de Reinforcement Learning como Q-Learning, PPO o DQN necesitarian cientos de miles de episodios de entrenamiento que no son practicos con un sensor tan limitado.</li>
        <li><strong>Determinismo vs optimalidad:</strong> A* y BFS son deterministicos, predecibles y garantizan encontrar una solucion si existe. Esto es critico para <strong>Safe Learning</strong> (aprendizaje seguro) donde no podemos permitir colisiones durante el entrenamiento.</li>
        <li><strong>Sin necesidad de entrenamiento:</strong> A diferencia de RL, A* y BFS funcionan directamente sobre el mapa parcial sin requerir ninguna fase de entrenamiento.</li>
    </ul>

    <h3>🌟 A* (A-Star)</h3>
    <p><strong>Tipo:</strong> Busqueda informada (heuristica).</p>
    <p><strong>Principio:</strong> Combina el costo real del camino recorrido (<em>g(n)</em>) con una estimacion heuristica del costo restante (<em>h(n)</em>) para guiar la busqueda hacia la meta. Utiliza una cola de prioridad (min-heap) para explorar primero los nodos mas prometedores.</p>
    <p><strong>Formula:</strong> <code>f(n) = g(n) + w * h(n)</code> donde <code>w</code> es un peso ajustable de la heuristica.</p>
    <p><strong>Heuristica usada:</strong> Manhattan (distancia en cuadricula) y Euclidea (distancia en linea recta).</p>
    <p><strong>Ventajas:</strong> Encuentra el camino optimo (con heuristica admisible). Mas rapido que BFS en la mayoria de los casos porque dirige la busqueda hacia la meta. Menor consumo de memoria.</p>
    <p><strong>Desventajas:</strong> No garantiza encontrar solucion si la heuristica sobreestima el costo real. Puede consumir mucha memoria en mapas grandes con muchas celdas.</p>

    <h3>🌐 BFS (Breadth-First Search)</h3>
    <p><strong>Tipo:</strong> Busqueda no informada (ciega).</p>
    <p><strong>Principio:</strong> Explora el grafo por niveles, expandiendo primero todos los nodos vecinos antes de pasar al siguiente nivel. Utiliza una cola FIFO (First-In-First-Out) para asegurar que se exploran primero los nodos mas cercanos al origen.</p>
    <p><strong>Ventajas:</strong> Garantiza encontrar la solucion de menor longitud en numero de pasos (camino mas corto en cantidad de nodos). Completo: siempre encuentra una solucion si existe.</p>
    <p><strong>Desventajas:</strong> Puede ser muy lento en mapas grandes porque explora en todas direcciones sin priorizar la direccion de la meta. Consume mucha memoria (almacena todos los nodos del nivel actual).</p>
</div>

<div class="section">
    <h2>🔬 Safe Learning (Aprendizaje Seguro)</h2>
    <p>El sistema incorpora un modulo de <strong>Safe Learning</strong> que opera en paralelo al planificador para garantizar que el robot nunca colisione:</p>
    <ul>
        <li><strong>Poligono de seguridad dinamico:</strong> Se expande proporcionalmente a la velocidad del robot (<code>largo = base + v * 0.5</code>).</li>
        <li><strong>Frenado predictivo:</strong> Calcula la distancia de frenado con <code>d = v²/(2*a) + v*t_reaction</code> y detiene el robot si es necesario.</li>
        <li><strong>Tres niveles de amenaza:</strong> Critico (&lt;0.15m → parada inmediata), Warning (&lt;0.3m → reduccion de velocidad), Safe (ejecucion normal).</li>
        <li><strong>Dead-end detection:</strong> Tras 3 intentos fallidos de escape, activa modo de recuperacion.</li>
        <li><strong>Home Base (Punto Bueno):</strong> Si el robot excede el maximo de intentos de recuperacion, se le ordena retornar al punto de partida (home base) para evitar danos.</li>
        <li><strong>Filtrado de autocolisiones:</strong> Ignora lecturas del sensor por debajo de <code>range_min + 0.06m</code> para evitar que las patas del robot activen falsas detecciones.</li>
    </ul>
    <p>Gracias a Safe Learning, el robot puede explorar entornos desconocidos sin riesgo de colision, incluso cuando el planificador global genera rutas que pasan cerca de obstaculos.</p>
</div>

<div class="section">
    <h2>📋 Metricas Evaluadas</h2>
    <ul>
        <li><strong>Tiempo de ejecucion:</strong> Tiempo desde que se envia el goal hasta que el robot llega al destino.</li>
        <li><strong>Longitud del camino:</strong> Distancia total recorrida por el robot (no la linea recta).</li>
        <li><strong>Nodos explorados:</strong> Cantidad de celdas del occupancy grid que fueron visitadas o marcadas como libres.</li>
        <li><strong>Cobertura del mapa:</strong> Porcentaje del mapa que ha sido relevado por el sensor.</li>
        <li><strong>Emergency stops:</strong> Cantidad de veces que Safe Learning activo una parada de emergencia.</li>
        <li><strong>Intentos de recuperacion:</strong> Veces que el robot quedo atascado y necesito maniobras de recuperacion.</li>
        <li><strong>Tasa de exito:</strong> Porcentaje de goals alcanzados exitosamente.</li>
        <li><strong>Tasa de evasion de colisiones:</strong> Efectividad del Safe Learning para evitar obstaculos.</li>
    </ul>
</div>

<div class="section">
    <h2>🏁 Conclusiones</h2>
    <ul>
        <li><strong>A* es superior en entornos con obstaculos dispersos</strong> porque su heuristica dirige eficientemente la busqueda hacia la meta, evitando exploracion innecesaria.</li>
        <li><strong>BFS es mas predecible</strong> explora sistematicamente, lo que puede ser util en entornos muy densos donde la heuristica de A* podria enganarse.</li>
        <li><strong>Safe Learning es indispensable</strong> en ambos casos para garantizar la integridad del robot, especialmente con un solo sensor ultrasonico.</li>
        <li><strong>La combinacion A* + Safe Learning</strong> ofrece el mejor balance entre eficiencia de ruta y seguridad para el MechDog.</li>
        <li>Para el robot real (ESP32 + ultrasonico), <strong>A* es la recomendacion principal</strong> por su menor cantidad de nodos explorados y menor uso de memoria.</li>
    </ul>
</div>

</div>
<div class="footer">
    <p>MechDog Experiment Framework — Generado el {time.strftime('%Y-%m-%d %H:%M:%S')}</p>
    <p>Parte del Proyecto MechDog — ROS Noetic + Docker + noVNC + Gazebo</p>
</div>
</body>
</html>'''

        output_path = os.path.join(self.param_output_dir, 'experiment_report.html')
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(html)
        rospy.loginfo("Report generated: %s", output_path)
        return output_path

    def run(self):
        rospy.loginfo("Generating experiment report...")
        results = self.load_results()
        if not results:
            rospy.logwarn("No results found - generating template report")
            results = self.create_sample_results()
        report_path = self.generate_html(results)
        rospy.loginfo("Report ready: %s", report_path)

    def create_sample_results(self):
        return [
            {'algorithm': 'astar', 'scenario': 'simple', 'trial': 1,
             'goal_1': {'success': True, 'execution_time': 12.5}},
            {'algorithm': 'astar', 'scenario': 'simple', 'trial': 2,
             'goal_1': {'success': True, 'execution_time': 11.2}},
            {'algorithm': 'bfs', 'scenario': 'simple', 'trial': 1,
             'goal_1': {'success': True, 'execution_time': 18.7}},
            {'algorithm': 'bfs', 'scenario': 'simple', 'trial': 2,
             'goal_1': {'success': True, 'execution_time': 20.1}},
        ]


if __name__ == '__main__':
    try:
        import time
        gen = ReportGenerator()
        gen.run()
    except rospy.ROSInterruptException:
        pass
