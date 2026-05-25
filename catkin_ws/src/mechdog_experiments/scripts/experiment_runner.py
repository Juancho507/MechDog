#!/usr/bin/env python
import rospy
import os
import sys
import json
import time
import math
import subprocess
import signal
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty


class ExperimentRunner:
    def __init__(self):
        rospy.init_node('experiment_runner', anonymous=False)
        self.load_parameters()

        self.results = {}
        self.goal_pub = rospy.Publisher(
            self.param_goal_topic, PoseStamped, queue_size=1, latch=True)
        self.status_sub = rospy.Subscriber(
            self.param_status_topic, String, self.status_callback)

        self.current_status = "idle"
        self.all_results = []
        self.scenarios_config = self.load_scenarios()
        self.processes = []

        rospy.loginfo("=" * 60)
        rospy.loginfo("MechDog Experiment Runner")
        rospy.loginfo("Algoritmos: A* (A-Star) vs BFS (Breadth-First Search)")
        rospy.loginfo("Escenarios: %s", self.param_scenarios)
        rospy.loginfo("Trials por escenario: %d", self.param_trials)
        rospy.loginfo("=" * 60)

    def load_parameters(self):
        self.param_goal_topic = rospy.get_param(
            '~topics/goal', '/mechdog/goal')
        self.param_status_topic = rospy.get_param(
            '~topics/status', '/mechdog/navigation_status')
        self.param_goal_tolerance = rospy.get_param(
            '~goal_tolerance', 0.3)
        self.param_max_timeout = rospy.get_param(
            '~max_timeout', 120.0)
        self.param_trials = rospy.get_param(
            '~trials_per_scenario', 3)
        self.param_algorithms = rospy.get_param(
            '~algorithms', ['astar', 'bfs'])
        self.param_scenarios = rospy.get_param(
            '~scenarios', ['simple', 'medium', 'complex'])
        self.param_output_dir = rospy.get_param(
            '~output_dir', '/tmp/mechdog_experiments')
        self.param_config_path = rospy.get_param(
            '~config_path', '')
        self.param_home_enabled = rospy.get_param(
            '~home_base/enabled', True)
        self.param_home_return = rospy.get_param(
            '~home_base/return_on_failure', True)

    def load_scenarios(self):
        config_paths = [
            self.param_config_path,
            os.path.join(rospy.get_param('~pkg_path',
                         '/app/catkin_ws/src/mechdog_experiments'),
                         'config', 'scenarios.yaml'),
            '/app/catkin_ws/src/mechdog_experiments/config/scenarios.yaml'
        ]
        for path in config_paths:
            if path and os.path.exists(path):
                with open(path, 'r') as f:
                    return yaml.safe_load(f).get('scenarios', {})
        rospy.logwarn("No scenarios config found, using defaults")
        return {}

    def status_callback(self, msg):
        self.current_status = msg.data

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo("Goal sent: (%.2f, %.2f)", x, y)

    def wait_for_status(self, target_status, timeout=60.0, check_interval=1.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.current_status == target_status:
                return True
            if rospy.is_shutdown():
                return False
            time.sleep(check_interval)
        return False

    def wait_for_goal_completion(self, timeout=120.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.current_status == "goal_reached":
                return True, "goal_reached"
            if self.current_status == "error":
                return False, "error"
            if self.current_status == "recovery":
                rospy.logwarn("In recovery - waiting...")
            if rospy.is_shutdown():
                return False, "shutdown"
            time.sleep(0.5)
        return False, "timeout"

    def set_planner_algorithm(self, algorithm):
        if algorithm == 'astar':
            rospy.set_param('~global_planner/algorithm', 'astar')
            rospy.loginfo("Algorithm set to: A* (A-Star)")
        elif algorithm == 'bfs':
            rospy.set_param('~global_planner/algorithm', 'bfs')
            rospy.loginfo("Algorithm set to: BFS (Breadth-First Search)")
        else:
            rospy.logerr("Unknown algorithm: %s", algorithm)
            return False
        return True

    def spawn_simulation(self, world_file):
        rospy.loginfo("Spawning simulation with world: %s", world_file)
        try:
            subprocess.Popen([
                'roslaunch', 'mechdog_sim', 'simulation.launch',
                f'world_file:={world_file}',
                'gui:=false', 'rviz:=false'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            rospy.logerr("Failed to spawn simulation: %s", e)
            return False
        return True

    def run_experiment(self, algorithm, scenario, trial):
        rospy.loginfo("-" * 50)
        rospy.loginfo("EXPERIMENT: %s | %s | Trial %d/%d",
                      algorithm.upper(), scenario.upper(),
                      trial + 1, self.param_trials)
        rospy.loginfo("-" * 50)

        self.set_planner_algorithm(algorithm)
        rospy.sleep(1.0)

        scenario_cfg = self.scenarios_config.get(scenario, {})
        goals = scenario_cfg.get('goals', [[5.0, 0.0]])
        home_pos = scenario_cfg.get('home_base', [0.0, 0.0])

        metrics_file = os.path.join(
            self.param_output_dir,
            f"metrics_{algorithm}_{scenario}_trial{trial+1}.json"
        )
        os.makedirs(self.param_output_dir, exist_ok=True)

        experiment_data = {
            'algorithm': algorithm,
            'scenario': scenario,
            'trial': trial + 1,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'metrics': {}
        }

        for goal_idx, goal_pos in enumerate(goals):
            rospy.loginfo("Goal %d/%d: (%.2f, %.2f)",
                          goal_idx + 1, len(goals),
                          goal_pos[0], goal_pos[1])

            rospy.sleep(2.0)
            self.send_goal(goal_pos[0], goal_pos[1])

            start_time = time.time()
            success, reason = self.wait_for_goal_completion(self.param_max_timeout)
            elapsed = time.time() - start_time

            goal_data = {
                'goal_position': goal_pos,
                'success': success,
                'reason': reason,
                'execution_time': elapsed,
            }
            experiment_data[f'goal_{goal_idx+1}'] = goal_data

            if success:
                rospy.loginfo("Goal reached in %.2f seconds", elapsed)
            else:
                rospy.logwarn("Goal failed: %s (%.2fs)", reason, elapsed)
                if self.param_home_return and self.param_home_enabled:
                    rospy.loginfo("Returning to home base (%.2f, %.2f)",
                                  home_pos[0], home_pos[1])
                    self.send_goal(home_pos[0], home_pos[1])
                    self.wait_for_goal_completion(60.0)
                    rospy.sleep(2.0)

        # Merge metrics from aggregator
        if os.path.exists(metrics_file):
            try:
                with open(metrics_file, 'r') as f:
                    metrics_data = json.load(f)
                metrics_data.pop('algorithm', None)
                metrics_data.pop('scenario', None)
                metrics_data.pop('trial', None)
                experiment_data['metrics'] = metrics_data
            except Exception as e:
                rospy.logwarn("Could not load metrics file: %s", e)

        self.all_results.append(experiment_data)
        self.save_intermediate_results()
        return experiment_data

    def save_intermediate_results(self):
        data_file = os.path.join(self.param_output_dir, 'results.json')
        with open(data_file, 'w') as f:
            json.dump(self.all_results, f, indent=2)

    def run_all_experiments(self):
        for scenario in self.param_scenarios:
            for algorithm in self.param_algorithms:
                for trial in range(self.param_trials):
                    rospy.loginfo("")
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("Escenario: %s | Algoritmo: %s | Trial: %d/%d",
                                  scenario, algorithm, trial + 1, self.param_trials)
                    rospy.loginfo("=" * 60)
                    try:
                        self.run_experiment(algorithm, scenario, trial)
                    except rospy.ROSException as e:
                        rospy.logerr("Experiment failed: %s", e)
                        continue

        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("ALL EXPERIMENTS COMPLETED")
        rospy.loginfo("Results saved to: %s", self.param_output_dir)
        rospy.loginfo("=" * 60)
        self.print_summary()

    def print_summary(self):
        if not self.all_results:
            rospy.logwarn("No results to summarize")
            return

        print("\n" + "=" * 70)
        print("RESUMEN DE EXPERIMENTOS")
        print("=" * 70)

        for algorithm in self.param_algorithms:
            algo_results = [r for r in self.all_results
                            if r['algorithm'] == algorithm]
            if not algo_results:
                continue

            successes = sum(1 for r in algo_results
                            for k, v in r.items()
                            if k.startswith('goal_') and v.get('success'))
            total_goals = sum(1 for r in algo_results
                              for k, v in r.items()
                              if k.startswith('goal_'))

            times = []
            for r in algo_results:
                for k, v in r.items():
                    if k.startswith('goal_') and v.get('success'):
                        times.append(v.get('execution_time', 0))

            print(f"\n{algorithm.upper()}:")
            print(f"  Tasa de exito: {successes}/{total_goals} ({successes/max(total_goals,1)*100:.1f}%)")
            if times:
                print(f"  Tiempo promedio: {sum(times)/len(times):.2f}s")
                print(f"  Tiempo minimo: {min(times):.2f}s")
                print(f"  Tiempo maximo: {max(times):.2f}s")
            print("  -" * 20)

        print("=" * 70)

    def run(self):
        rospy.loginfo("Experiment Runner ready")
        rospy.sleep(2.0)
        self.run_all_experiments()
        rospy.loginfo("Experiment Runner finished")


if __name__ == '__main__':
    try:
        runner = ExperimentRunner()
        runner.run()
    except rospy.ROSInterruptException:
        pass
