import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
MAX_LINEAR_VELOCITY = 0.5  # Maximum linear velocity (m/s)
MAX_ANGULAR_VELOCITY = 1.0  # Maximum angular velocity (rad/s)
ROBOT_RADIUS = 0.2  # Robot radius (m)
DT = 0.1  # Time step (s)

# Other parameters
TIME_HORIZON = 2.0  # Time horizon for trajectory simulation (s)
NUM_TRAJECTORY_SAMPLES = 100  # Number of trajectory samples

def dynamic_window_approach(robot_pose, goal_pose, obstacles):
    # Calculate dynamic window based on robot constraints
    min_linear_velocity = 0.0
    max_linear_velocity = min(MAX_LINEAR_VELOCITY, goal_distance(robot_pose, goal_pose) / TIME_HORIZON)
    min_angular_velocity = -MAX_ANGULAR_VELOCITY
    max_angular_velocity = MAX_ANGULAR_VELOCITY

    # Generate linear and angular velocity candidates
    linear_velocities = np.linspace(min_linear_velocity, max_linear_velocity, NUM_TRAJECTORY_SAMPLES)
    angular_velocities = np.linspace(min_angular_velocity, max_angular_velocity, NUM_TRAJECTORY_SAMPLES)

    # Evaluate each trajectory and find the best one
    best_trajectory = None
    best_trajectory_score = float('-inf')

    for linear_velocity in linear_velocities:
        for angular_velocity in angular_velocities:
            trajectory = simulate_trajectory(robot_pose, linear_velocity, angular_velocity)
            trajectory_score = evaluate_trajectory(trajectory, goal_pose, obstacles)

            if trajectory_score > best_trajectory_score:
                best_trajectory = trajectory
                best_trajectory_score = trajectory_score

    return best_trajectory

def goal_distance(pose1, pose2):
    # Calculate Euclidean distance between two poses
    dx = pose2[0] - pose1[0]
    dy = pose2[1] - pose1[1]
    return np.sqrt(dx**2 + dy**2)

def simulate_trajectory(robot_pose, linear_velocity, angular_velocity):
    # Simulate robot trajectory based on given velocities
    trajectory = []
    current_pose = robot_pose

    for _ in range(int(TIME_HORIZON / DT)):
        current_pose = update_pose(current_pose, linear_velocity, angular_velocity)
        trajectory.append(current_pose)

    return trajectory

def update_pose(pose, linear_velocity, angular_velocity):
    # Update robot pose based on given velocities
    x, y, theta = pose
    new_x = x + linear_velocity * np.cos(theta) * DT
    new_y = y + linear_velocity * np.sin(theta) * DT
    new_theta = theta + angular_velocity * DT

    return np.array([new_x, new_y, new_theta])

def evaluate_trajectory(trajectory, goal_pose, obstacles):
    # Evaluate trajectory based on scoring function
    score = 0.0

    for pose in trajectory:
        distance_to_goal = goal_distance(pose, goal_pose)
        obstacle_cost = calculate_obstacle_cost(pose, obstacles)

        # Define your scoring function based on desired criteria
        score += 1.0 / distance_to_goal - obstacle_cost

    return score

def calculate_obstacle_cost(pose, obstacles):
    # Calculate cost based on proximity to obstacles
    cost = 0.0

    for obstacle in obstacles:
        distance = goal_distance(pose, obstacle)

        if distance < ROBOT_RADIUS:
            cost += 1.0 / distance

    return cost

# Visualization functions
def plot_robot(robot_pose):
    plt.plot(robot_pose[0], robot_pose[1], 'bo', markersize=10)
    robot_circle = plt.Circle((robot_pose[0], robot_pose[1]), ROBOT_RADIUS, color='b', fill=False)
    plt.gca().add_patch(robot_circle)

def plot_goal(goal_pose):
    plt.plot(goal_pose[0], goal_pose[1], 'go', markersize=10)

def plot_obstacles(obstacles):
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1], 'ro', markersize=10)

def plot_trajectory(trajectory):
    trajectory_x = [pose[0] for pose in trajectory]
    trajectory_y = [pose[1] for pose in trajectory]
    plt.plot(trajectory_x, trajectory_y, 'r-')

# Main
def main():
    # Environment setup
    robot_pose = np.array([0.0, 0.0, 0.0])  # Initial robot pose (x, y, theta)
    goal_pose = np.array([5.0, 5.0])  # Goal pose (x, y)
    obstacles = [np.array([2.0, 2.0]), np.array([3.0, 3.0])]  # List of obstacle poses

    # Dynamic Window Approach
    best_trajectory = dynamic_window_approach(robot_pose, goal_pose, obstacles)

    # Plotting
    plt.figure()
    plot_robot(robot_pose)
    plot_goal(goal_pose)
    plot_obstacles(obstacles)
    plot_trajectory(best_trajectory)
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    main()
