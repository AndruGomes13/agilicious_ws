from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
from typing import List, Tuple

import numpy as np
from agiros_msgs.msg import QuadState, Telemetry, Command
from std_msgs.msg import Bool
import rospy
from collections import deque
import time
from protocol.messages import InferenceMsg, InferenceReply, parse_message
    
@dataclass
class DroneState:
    time:float
    position: np.ndarray
    velocity: np.ndarray
    orientation: np.ndarray
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    gyro_bias: np.ndarray

@dataclass
class BallState:
    time:float
    position: np.ndarray
    velocity: np.ndarray
    

class MLPPilot:
    def __init__(self, policy_sampling_frequency: float, jax_policy_path: Path):
        self.policy_sampling_frequency = policy_sampling_frequency
        self.policy = self.load_jax_policy(jax_policy_path)
        self.inference_server = self.start_inference_server(jax_policy_path.parent, (1, 10))  # TODO: Adjust obs shape
        self.check_inference_time()
        
        self.run_policy = False
        self.history_is_initialized = False

        DRONE_BUFFER_SIZE = 100
        BALL_BUFFER_SIZE = 100
        DRONE_HISTORY_SIZE = 10
        BALL_HISTORY_SIZE = 10

        self.drone_state_buffer: deque[DroneState] = deque(maxlen=DRONE_BUFFER_SIZE)
        self.ball_state_buffer: deque[BallState] = deque(maxlen=BALL_BUFFER_SIZE)
        self.drone_state_history: deque[DroneState] = deque(maxlen=DRONE_HISTORY_SIZE)
        self.ball_state_history: deque[BallState] = deque(maxlen=BALL_HISTORY_SIZE)

        self.position_bounds = (
            np.array([-10.0, -10.0, -10.0]),
            np.array([10.0, 10.0, 10.0])
        )

        self.init_subscriptions()
        self.init_publishers()
    
    def init_subscriptions(self):
        self.state_sub = rospy.Subscriber("agiros_pilot/state", QuadState, self.callback_drone_state)
        self.telemetery_sub = rospy.Subscriber(
            "agiros_pilot/telemetry",
            Telemetry,
            queue_size=1,
        )
        self.start_signal_sub = rospy.Subscriber(
            "/start_policy", Bool, self.callback_start_signal, queue_size=1
        )
        
    def init_publishers(self):
        self.command_pub = rospy.Publisher("agiros_pilot/feedthrough_command", Command, queue_size=1, tcp_nodelay=True)
        
    def callback_drone_state(self, msg: QuadState):
        # Process the state message
        drone_state = self.parse_state_msg(msg)
        
        if drone_state.time < self.drone_state_buffer[-1].time if self.drone_state_buffer else 0:
            rospy.logwarn("Received drone state with time less than the last recorded state. Ignoring.")
            return
        
        self.drone_state_buffer.appendleft(drone_state)
        
        # Check if the drone state is within the defined bounds
        self.check_drone_state(drone_state)
            
    def callback_telemetry(self, msg):
        # Process the telemetry message
        rospy.loginfo("Received telemetry data: %s", msg.data)
        
    def callback_start_signal(self, msg: Bool):
        # Start or stop the policy execution based on the received signal and start conditions
        candidate_start = msg.data
        
        if candidate_start and not self.validate_pre_start_conditions():
            rospy.logwarn("Pre-start conditions not met. Cannot start policy.")
            return    
        
        self.run_policy = msg.data
        if self.run_policy:
            rospy.loginfo("Start signal received, initiating policy execution.")
        else:
            rospy.loginfo("Stop signal received, halting policy execution.")
       
    def check_drone_state(self, drone_state: DroneState):
        # Check if the state is within the acceptable range
        if not self.is_within_bounds(drone_state.position, self.position_bounds):
            rospy.logwarn("Drone position out of bounds: %s", drone_state.position)
            self.stop_policy()

    def is_within_bounds(self, value: np.ndarray, bounds: Tuple[np.ndarray, np.ndarray]) -> bool:
        return np.all(value >= bounds[0]) and np.all(value <= bounds[1])

    def parse_state_msg(self, msg: QuadState) -> DroneState:
        # Parse the state message to extract relevant information
        time = msg.header.stamp.to_sec()
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        velocity = np.array([msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z])
        orientation_wxyz = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        angular_velocity = np.array([msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z])
        angular_acceleration = np.array([msg.acceleration.angular.x, msg.acceleration.angular.y, msg.acceleration.angular.z])
        gyro_bias = np.array([msg.gyr_bias.x, msg.gyr_bias.y, msg.gyr_bias.z])
        

        return DroneState(
            time=time,
            position=position,
            velocity=velocity,
            orientation=orientation_wxyz,
            angular_velocity=angular_velocity,
            angular_acceleration=angular_acceleration,
            gyro_bias=gyro_bias
        )

    def parse_ball_msg(self, msg) -> BallState:
        # TODO: Implement the parsing of the ball state message
        pass
    
    def get_and_publish_command(self):
        if not self.run_policy:
            return
        
        if not self.history_is_initialized:
            self.initialize_history()
            if not self.history_is_initialized:
                rospy.logwarn("Could not initialize history, cannot execute policy.")
                return
            
        observation = ...
        jax_command = ...  # Call the JAX policy with the observation
        
        command = Command()
        command.bodyrates = jax_command.bodyrates.tolist()
        command.collective_thrust = jax_command.thrust.tolist()
        
        self.command_pub.publish(command)

    def stop_policy(self):
        self.run_policy = False
        self.history_is_initialized = False
        rospy.loginfo("Policy execution stopped.")

    def initialize_history(self):
        # Initializes the history by sampling the state buffers at policy sampling rate.
        # If there are not enough samples, copy the last available state.
        # TODO: How can I initialize the action history?
        return False
        
    def validate_pre_start_conditions(self) -> bool:
        # Perform necessary checks before starting the policy:
        # - There are enough samples and duration in the drone state buffer
        # - The drone is stationary (velocity and angular velocity are within a small threshold)
        # - The drone position is within the defined bounds
        
        CHECK_WINDOW_DURATION = 1.0  # seconds
        MIN_SAMPLES = CHECK_WINDOW_DURATION * self.policy_sampling_frequency
        current_time = rospy.get_time()
        
        has_enough_duration = False
        num_samples_within_timeframe = 0
        failed_checks = False
        
        for drone_state in self.drone_state_buffer:
            if drone_state.time < current_time - CHECK_WINDOW_DURATION:
                has_enough_duration = True
                break
            num_samples_within_timeframe += 1
            
            if not self.is_within_bounds(drone_state.position, self.position_bounds):
                rospy.logwarn("Drone position out of bounds: %s", drone_state.position)
                failed_checks = True
                break
            
            if not self.is_within_bounds(drone_state.velocity, (np.array([-0.1, -0.1, -0.1]), np.array([0.1, 0.1, 0.1]))):
                rospy.logwarn(
                    "Drone is not stationary (Linear Velocity = [%f, %f, %f])",
                    drone_state.velocity[0],
                    drone_state.velocity[1],
                    drone_state.velocity[2]
                )
                failed_checks = True
                break
            
            if not self.is_within_bounds(drone_state.angular_velocity, (np.array([-0.1, -0.1, -0.1]), np.array([0.1, 0.1, 0.1]))):
                rospy.logwarn(
                    "Drone is not stationary (Angular Velocity = [%f, %f, %f])",
                    drone_state.angular_velocity[0],
                    drone_state.angular_velocity[1],
                    drone_state.angular_velocity[2]
                )
                failed_checks = True
                break
            
        if not has_enough_duration:
            rospy.logwarn("Not enough duration in drone state buffer to start policy.")
            return False
        
        if num_samples_within_timeframe < MIN_SAMPLES:
            rospy.logwarn("Not enough samples in drone state buffer to start policy.")
            return False
        
        if failed_checks:
            rospy.logwarn("Failed checks for starting policy.")
            return False
        
        return True
            
    def load_jax_policy(self, jax_policy_path: Path):
        # Load the JAX policy from the specified path
        if not jax_policy_path.exists():
            rospy.logerr(f"JAX policy file not found at {jax_policy_path}")
            return None
        
        #TODO: Implement the actual loading of the JAX policy
        policy_fn = ...
        
        rospy.loginfo(f"JAX policy loaded from {jax_policy_path}")
        
        # Run warmup to ensure the policy is JIT-compiled and ready for execution
        for _ in range(10):
            # Generate a warmup observation, adjust the shape as needed
            warmup_observation = np.zeros((1, 10))  # Adjust the shape
            policy_fn(warmup_observation)

        # Test the inference time
        start_time = time.time()
        TEST_ITERATIONS = 100
        for _ in range(TEST_ITERATIONS):
            warmup_observation = np.zeros((1, 10)) # TODO: Adjust the shape
            policy_fn(warmup_observation)
        end_time = time.time()
        inference_time = (end_time - start_time) / TEST_ITERATIONS
        
        
        # Check if the inference time is within acceptable limits
        sampling_period = 1 / self.policy_sampling_frequency
        if inference_time < sampling_period * 0.5:
            rospy.loginfo(f"JAX policy inference time is acceptable: {inference_time * 1000:.6f} ms per iteration")
        elif inference_time < sampling_period * 0.8:
            rospy.logwarn(f"JAX policy inference time is high: {inference_time * 1000:.6f} ms per iteration")
        else:
            rospy.logerr(f"JAX policy inference time is too high: {inference_time * 1000:.6f} ms per iteration")
            raise RuntimeError(
                f"JAX policy inference time exceeds sampling period: {inference_time * 1000:.6f} ms per iteration"
            )

        return policy_fn
    
    def start_inference_server(self, checkpoint_dir: Path, observation_shape: Tuple[int, ...]):
        # Start the inference server with the specified checkpoint directory and observation shape
        if not checkpoint_dir.exists():
            raise FileNotFoundError(f"Checkpoint directory not found at {checkpoint_dir}")

        r_fd, w_fd = os.pipe()
        
        server = subprocess.Popen(
            ["python3.11", "inference_server.py", checkpoint_dir.absolute(), str(observation_shape), str(r_fd)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,  # line buffered
            pass_fds=(r_fd,)  # pass the read end of the pipe to the subprocess
        )
        rospy.loginfo(f"Inference server started with checkpoint directory: {checkpoint_dir}")

        os.close(r_fd)
       
        if not server.stdout or not server.stderr:
            raise RuntimeError("Failed to start inference server: No stdout or stderr pipe")

    def run_inference(self, observation: np.ndarray) -> np.ndarray:
        # Send an inference request to the server and get the response
        if not self.inference_server:
            raise RuntimeError("Inference server is not running")
        inference_msg = InferenceMsg(observation.tolist())
        self.inference_server.stdin.write(inference_msg.to_json() + "\n")
        self.inference_server.stdin.flush()
        response = self.inference_server.stdout.readline().strip()
        response_data = parse_message(response)
        if isinstance(response_data, InferenceReply):
            if response_data.status == "ok":
                return np.array(response_data.result)
            else:
                raise RuntimeError(f"Inference server returned error: {response_data.error}")
        else:
            raise ValueError(f"Unexpected response type: {type(response_data)}")
        
    def check_inference_time(self):
        # Check the inference time of the server
        test_obs = np.zeros((1, 10))  # TODO: Adjust the shape
        expected_inference_time = 1 / self.policy_sampling_frequency
        NUM_ITER = 100

        start_time = time.time()
        for _ in range(NUM_ITER):
            self.run_inference(test_obs)
            
        end_time = time.time()
        inference_time = (end_time - start_time) / NUM_ITER

        if inference_time < expected_inference_time * 0.5:
            rospy.loginfo(f"JAX policy inference time is acceptable: {inference_time * 1000:.6f} ms per iteration")
        elif inference_time < expected_inference_time * 0.8:
            rospy.logwarn(f"JAX policy inference time is high: {inference_time * 1000:.6f} ms per iteration")
        else:
            rospy.logerr(f"JAX policy inference time is too high: {inference_time * 1000:.6f} ms per iteration")
            raise RuntimeError(
                f"JAX policy inference time exceeds sampling period: {inference_time * 1000:.6f} ms per iteration"
            )
        
        
    
def main():
    CONTROL_RATE = 100  # Hz
    
    rospy.init_node("mlp_ctrl_node", anonymous=True)

    pilot = MLPPilot(CONTROL_RATE)
    rospy.loginfo("MLP Control Node initialized and running.")
    
    rate = rospy.Rate(CONTROL_RATE)  # 100 Hz

    while not rospy.is_shutdown():
        pilot.get_and_publish_command()
        rate.sleep()
        rospy.spin_once()