#!/usr/bin/env python3
"""
EKF Covariance Adapter - Interface with robot_localization for dynamic parameter adjustment

This node serves as the bridge between sensor health monitoring and robot_localization
EKF parameter adjustment, providing safe and controlled covariance modifications.

Author: fjgarco
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter

import numpy as np
import yaml
from typing import Dict, List, Any, Optional, Tuple
import threading
import time
from dataclasses import dataclass

# ROS 2 services and messages
from std_msgs.msg import String, Float64MultiArray
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from diagnostic_msgs.msg import DiagnosticArray


@dataclass
class EKFParameterMapping:
    """Mapping between sensors and EKF parameters"""
    sensor_name: str
    ekf_parameter: str
    baseline_value: float
    min_value: float
    max_value: float
    parameter_type: str  # 'process_noise' or 'initial_estimate'


class EKFCovarianceAdapter(Node):
    """
    EKF Covariance Adapter Node
    
    Responsibilities:
    - Interface with robot_localization parameter services
    - Apply safe covariance adjustments based on sensor health
    - Maintain parameter backup and rollback capabilities
    - Provide parameter adjustment logging and monitoring
    """
    
    def __init__(self):
        super().__init__('ekf_covariance_adapter')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self._declare_parameters()
        
        # EKF parameter mappings and state
        self.parameter_mappings: List[EKFParameterMapping] = []
        self.current_parameters: Dict[str, float] = {}
        self.baseline_parameters: Dict[str, float] = {}
        self.parameter_history: List[Dict[str, Any]] = []
        
        # Thread safety
        self.param_lock = threading.RLock()
        
        # Load EKF parameter configuration
        self._load_ekf_configuration()
        
        # Setup service clients for EKF communication
        self._setup_ekf_services()
        
        # Setup subscribers for sensor health updates
        self._setup_subscribers()
        
        # Setup publishers for monitoring
        self._setup_publishers()
        
        # Initialize EKF parameters
        self._initialize_ekf_parameters()
        
        # Start parameter monitoring timer
        self.monitoring_timer = self.create_timer(
            self.monitoring_period,
            self._monitor_ekf_parameters,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("EKF Covariance Adapter initialized successfully")
        self.get_logger().info(f"Managing {len(self.parameter_mappings)} EKF parameters")
    
    def _declare_parameters(self):
        """Declare node parameters"""
        
        # EKF connection parameters
        self.declare_parameter('ekf_namespace', 'robot_localization')
        self.declare_parameter('ekf_node_name', 'ekf_filter_node')
        self.declare_parameter('connection_timeout', 10.0)
        
        # Timing parameters
        self.declare_parameter('monitoring_period', 1.0)  # 1 Hz parameter monitoring
        self.declare_parameter('adjustment_period', 0.5)  # 2 Hz adjustment application
        
        # Safety parameters
        self.declare_parameter('enable_parameter_adjustment', True)
        self.declare_parameter('max_adjustment_factor', 10.0)
        self.declare_parameter('min_adjustment_factor', 0.1)
        self.declare_parameter('parameter_change_rate_limit', 2.0)  # Max 2x change per second
        
        # Configuration file
        self.declare_parameter('ekf_mapping_file', 'config/ekf_parameter_mapping.yaml')
        
        # Get parameter values
        self.ekf_namespace = self.get_parameter('ekf_namespace').value
        self.ekf_node_name = self.get_parameter('ekf_node_name').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        self.monitoring_period = self.get_parameter('monitoring_period').value
        self.adjustment_period = self.get_parameter('adjustment_period').value
        self.enable_parameter_adjustment = self.get_parameter('enable_parameter_adjustment').value
        self.max_adjustment_factor = self.get_parameter('max_adjustment_factor').value
        self.min_adjustment_factor = self.get_parameter('min_adjustment_factor').value
        self.parameter_change_rate_limit = self.get_parameter('parameter_change_rate_limit').value
        self.ekf_mapping_file = self.get_parameter('ekf_mapping_file').value
    
    def _load_ekf_configuration(self):
        """Load EKF parameter mapping configuration"""
        try:
            from ament_index_python.packages import get_package_share_directory
            config_path = get_package_share_directory('adaptive_covariance') + '/' + self.ekf_mapping_file
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Parse parameter mappings
            for mapping_config in config.get('ekf_parameter_mappings', []):
                mapping = EKFParameterMapping(
                    sensor_name=mapping_config['sensor_name'],
                    ekf_parameter=mapping_config['ekf_parameter'],
                    baseline_value=mapping_config['baseline_value'],
                    min_value=mapping_config.get('min_value', mapping_config['baseline_value'] * 0.1),
                    max_value=mapping_config.get('max_value', mapping_config['baseline_value'] * 10.0),
                    parameter_type=mapping_config.get('parameter_type', 'process_noise')
                )
                self.parameter_mappings.append(mapping)
                self.baseline_parameters[mapping.ekf_parameter] = mapping.baseline_value
            
            self.get_logger().info(f"Loaded {len(self.parameter_mappings)} EKF parameter mappings")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load EKF configuration: {str(e)}")
            self._setup_default_ekf_mappings()
    
    def _setup_default_ekf_mappings(self):
        """Setup default EKF parameter mappings if config file is missing"""
        
        # Default mappings for standard robot_localization setup
        default_mappings = [
            # GPS mappings (position)
            {'sensor_name': 'gps', 'ekf_parameter': 'process_noise_covariance.0', 'baseline_value': 0.05},
            {'sensor_name': 'gps', 'ekf_parameter': 'process_noise_covariance.7', 'baseline_value': 0.05},
            {'sensor_name': 'gps', 'ekf_parameter': 'process_noise_covariance.14', 'baseline_value': 0.06},
            
            # IMU mappings (orientation and angular velocity)
            {'sensor_name': 'imu', 'ekf_parameter': 'process_noise_covariance.21', 'baseline_value': 0.03},
            {'sensor_name': 'imu', 'ekf_parameter': 'process_noise_covariance.28', 'baseline_value': 0.03},
            {'sensor_name': 'imu', 'ekf_parameter': 'process_noise_covariance.35', 'baseline_value': 0.06},
            
            # LIDAR mappings (position and orientation)
            {'sensor_name': 'lidar', 'ekf_parameter': 'process_noise_covariance.0', 'baseline_value': 0.1},
            {'sensor_name': 'lidar', 'ekf_parameter': 'process_noise_covariance.7', 'baseline_value': 0.1},
            {'sensor_name': 'lidar', 'ekf_parameter': 'process_noise_covariance.35', 'baseline_value': 0.2},
            
            # Encoder mappings (velocity)
            {'sensor_name': 'encoders', 'ekf_parameter': 'process_noise_covariance.42', 'baseline_value': 0.1},
            {'sensor_name': 'encoders', 'ekf_parameter': 'process_noise_covariance.49', 'baseline_value': 0.1},
            {'sensor_name': 'encoders', 'ekf_parameter': 'process_noise_covariance.77', 'baseline_value': 0.2},
        ]
        
        for mapping_config in default_mappings:
            mapping = EKFParameterMapping(
                sensor_name=mapping_config['sensor_name'],
                ekf_parameter=mapping_config['ekf_parameter'],
                baseline_value=mapping_config['baseline_value'],
                min_value=mapping_config['baseline_value'] * 0.1,
                max_value=mapping_config['baseline_value'] * 10.0,
                parameter_type='process_noise'
            )
            self.parameter_mappings.append(mapping)
            self.baseline_parameters[mapping.ekf_parameter] = mapping.baseline_value
        
        self.get_logger().warn("Using default EKF parameter mappings")
    
    def _setup_ekf_services(self):
        """Setup service clients for EKF parameter communication"""
        
        ekf_service_prefix = f'/{self.ekf_namespace}/{self.ekf_node_name}'
        
        # Service clients
        self.set_params_client = self.create_client(
            SetParameters, f'{ekf_service_prefix}/set_parameters'
        )
        
        self.get_params_client = self.create_client(
            GetParameters, f'{ekf_service_prefix}/get_parameters'
        )
        
        self.list_params_client = self.create_client(
            ListParameters, f'{ekf_service_prefix}/list_parameters'
        )
        
        # Wait for services
        self.get_logger().info("Waiting for EKF services...")
        
        services_ready = True
        if not self.set_params_client.wait_for_service(timeout_sec=self.connection_timeout):
            self.get_logger().error("EKF set_parameters service not available")
            services_ready = False
        
        if not self.get_params_client.wait_for_service(timeout_sec=self.connection_timeout):
            self.get_logger().error("EKF get_parameters service not available")
            services_ready = False
        
        if not self.list_params_client.wait_for_service(timeout_sec=self.connection_timeout):
            self.get_logger().error("EKF list_parameters service not available")
            services_ready = False
        
        if services_ready:
            self.get_logger().info("EKF services connected successfully")
        else:
            self.get_logger().warn("EKF services not available - running in monitoring mode only")
            self.enable_parameter_adjustment = False
    
    def _setup_subscribers(self):
        """Setup subscribers for sensor health updates"""
        
        # Subscribe to sensor health updates from adaptive covariance node
        self.sensor_health_sub = self.create_subscription(
            String, '/adaptive_covariance/sensor_health',
            self._sensor_health_callback, 10,
            callback_group=self.callback_group
        )
        
        # Subscribe to covariance adjustment logs
        self.covariance_log_sub = self.create_subscription(
            String, '/adaptive_covariance/adjustments',
            self._covariance_adjustment_callback, 10,
            callback_group=self.callback_group
        )
    
    def _setup_publishers(self):
        """Setup publishers for monitoring and logging"""
        
        # EKF parameter status publisher
        self.ekf_status_pub = self.create_publisher(
            String, '/ekf_adapter/parameter_status', 10
        )
        
        # Parameter change log publisher
        self.param_log_pub = self.create_publisher(
            String, '/ekf_adapter/parameter_changes', 10
        )
        
        # Current covariance values publisher
        self.current_covariances_pub = self.create_publisher(
            Float64MultiArray, '/ekf_adapter/current_covariances', 10
        )
    
    def _initialize_ekf_parameters(self):
        """Initialize EKF parameters to baseline values"""
        
        if not self.enable_parameter_adjustment:
            return
        
        self.get_logger().info("Initializing EKF parameters to baseline values...")
        
        # Get current parameter values first
        current_params = self._get_current_ekf_parameters()
        
        # Apply baseline values
        param_updates = {}
        for mapping in self.parameter_mappings:
            current_value = current_params.get(mapping.ekf_parameter, mapping.baseline_value)
            self.current_parameters[mapping.ekf_parameter] = current_value
            
            # Only update if significantly different from baseline
            if abs(current_value - mapping.baseline_value) / mapping.baseline_value > 0.1:
                param_updates[mapping.ekf_parameter] = mapping.baseline_value
        
        if param_updates:
            self._apply_parameter_updates(param_updates, "Baseline initialization")
            self.get_logger().info(f"Initialized {len(param_updates)} EKF parameters")
        else:
            self.get_logger().info("EKF parameters already at baseline values")
    
    def _get_current_ekf_parameters(self) -> Dict[str, float]:
        """Get current EKF parameter values"""
        
        if not self.get_params_client.service_is_ready():
            return {}
        
        try:
            # Request current parameter values
            request = GetParameters.Request()
            request.names = [mapping.ekf_parameter for mapping in self.parameter_mappings]
            
            future = self.get_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                current_params = {}
                
                for i, param_name in enumerate(request.names):
                    if i < len(response.values):
                        param_value = response.values[i]
                        if param_value.type == ParameterType.PARAMETER_DOUBLE:
                            current_params[param_name] = param_value.double_value
                        elif param_value.type == ParameterType.PARAMETER_INTEGER:
                            current_params[param_name] = float(param_value.integer_value)
                
                return current_params
            
        except Exception as e:
            self.get_logger().error(f"Failed to get current EKF parameters: {str(e)}")
        
        return {}
    
    def _sensor_health_callback(self, msg: String):
        """Handle sensor health updates"""
        
        try:
            sensor_health = yaml.safe_load(msg.data)
            self._process_sensor_health_update(sensor_health)
        except Exception as e:
            self.get_logger().error(f"Failed to process sensor health update: {str(e)}")
    
    def _covariance_adjustment_callback(self, msg: String):
        """Handle covariance adjustment logs"""
        
        try:
            adjustments = yaml.safe_load(msg.data)
            self._process_covariance_adjustments(adjustments)
        except Exception as e:
            self.get_logger().error(f"Failed to process covariance adjustments: {str(e)}")
    
    def _process_sensor_health_update(self, sensor_health: Dict[str, Any]):
        """Process sensor health update and adjust EKF parameters accordingly"""
        
        if not self.enable_parameter_adjustment:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        param_updates = {}
        
        with self.param_lock:
            for sensor_name, health_data in sensor_health.items():
                covariance_factor = health_data.get('factor', 1.0)
                
                # Apply safety limits
                covariance_factor = max(self.min_adjustment_factor, 
                                      min(self.max_adjustment_factor, covariance_factor))
                
                # Find relevant parameter mappings
                relevant_mappings = [m for m in self.parameter_mappings if m.sensor_name == sensor_name]
                
                for mapping in relevant_mappings:
                    new_value = mapping.baseline_value * covariance_factor
                    
                    # Apply parameter-specific limits
                    new_value = max(mapping.min_value, min(mapping.max_value, new_value))
                    
                    # Check rate limiting
                    current_value = self.current_parameters.get(mapping.ekf_parameter, mapping.baseline_value)
                    if self._is_rate_limited(mapping.ekf_parameter, current_value, new_value, current_time):
                        continue
                    
                    # Only update if change is significant
                    if abs(new_value - current_value) / current_value > 0.05:  # 5% threshold
                        param_updates[mapping.ekf_parameter] = new_value
        
        # Apply parameter updates
        if param_updates:
            reason = f"Sensor health update: {len(param_updates)} parameters"
            self._apply_parameter_updates(param_updates, reason)
    
    def _process_covariance_adjustments(self, adjustments: List[Dict[str, Any]]):
        """Process covariance adjustments from the adaptive covariance node"""
        
        if not self.enable_parameter_adjustment:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        param_updates = {}
        
        with self.param_lock:
            for adjustment in adjustments:
                sensor_name = adjustment.get('sensor', '')
                factor = adjustment.get('factor', 1.0)
                
                # Apply safety limits
                factor = max(self.min_adjustment_factor, min(self.max_adjustment_factor, factor))
                
                # Find relevant parameter mappings
                relevant_mappings = [m for m in self.parameter_mappings if m.sensor_name == sensor_name]
                
                for mapping in relevant_mappings:
                    new_value = mapping.baseline_value * factor
                    
                    # Apply parameter-specific limits
                    new_value = max(mapping.min_value, min(mapping.max_value, new_value))
                    
                    # Check rate limiting
                    current_value = self.current_parameters.get(mapping.ekf_parameter, mapping.baseline_value)
                    if self._is_rate_limited(mapping.ekf_parameter, current_value, new_value, current_time):
                        continue
                    
                    param_updates[mapping.ekf_parameter] = new_value
        
        # Apply parameter updates
        if param_updates:
            reason = f"Covariance adjustment: {len(adjustments)} sensors"
            self._apply_parameter_updates(param_updates, reason)
    
    def _is_rate_limited(self, param_name: str, current_value: float, new_value: float, current_time: float) -> bool:
        """Check if parameter change is rate limited"""
        
        # Find last change time for this parameter
        last_change_time = 0.0
        for record in reversed(self.parameter_history):
            if param_name in record.get('changes', {}):
                last_change_time = record.get('timestamp', 0.0)
                break
        
        time_since_change = current_time - last_change_time
        if time_since_change < 1.0:  # Less than 1 second since last change
            # Check if change rate exceeds limit
            if time_since_change > 0:
                change_rate = abs(new_value - current_value) / current_value / time_since_change
                if change_rate > self.parameter_change_rate_limit:
                    return True  # Rate limited
        
        return False  # Not rate limited
    
    def _apply_parameter_updates(self, param_updates: Dict[str, float], reason: str):
        """Apply parameter updates to EKF"""
        
        if not param_updates or not self.set_params_client.service_is_ready():
            return
        
        try:
            # Prepare parameter update request
            request = SetParameters.Request()
            request.parameters = []
            
            for param_name, value in param_updates.items():
                param = ParameterMsg()
                param.name = param_name
                param.value = ParameterValue()
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
                request.parameters.append(param)
            
            # Send request
            future = self.set_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                successful_updates = {}
                
                for i, result in enumerate(response.results):
                    param_name = request.parameters[i].name
                    if result.successful:
                        successful_updates[param_name] = param_updates[param_name]
                        self.current_parameters[param_name] = param_updates[param_name]
                    else:
                        self.get_logger().warn(f"Failed to update parameter {param_name}: {result.reason}")
                
                if successful_updates:
                    # Log the update
                    update_record = {
                        'timestamp': self.get_clock().now().nanoseconds / 1e9,
                        'reason': reason,
                        'changes': successful_updates,
                        'total_parameters': len(self.parameter_mappings)
                    }
                    self.parameter_history.append(update_record)
                    
                    # Publish update log
                    self._publish_parameter_change_log(update_record)
                    
                    self.get_logger().info(f"Updated {len(successful_updates)} EKF parameters: {reason}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to apply parameter updates: {str(e)}")
    
    def _monitor_ekf_parameters(self):
        """Monitor current EKF parameter values"""
        
        current_params = self._get_current_ekf_parameters()
        
        with self.param_lock:
            # Update internal state
            self.current_parameters.update(current_params)
        
        # Publish current status
        self._publish_ekf_status()
        self._publish_current_covariances()
    
    def _publish_ekf_status(self):
        """Publish EKF parameter status"""
        
        status_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'parameters': self.current_parameters.copy(),
            'baselines': self.baseline_parameters.copy(),
            'adjustment_enabled': self.enable_parameter_adjustment,
            'total_mappings': len(self.parameter_mappings)
        }
        
        # Calculate adjustment factors
        adjustment_factors = {}
        for param_name, current_value in self.current_parameters.items():
            baseline_value = self.baseline_parameters.get(param_name, current_value)
            if baseline_value > 0:
                adjustment_factors[param_name] = current_value / baseline_value
        
        status_data['adjustment_factors'] = adjustment_factors
        
        status_msg = String()
        status_msg.data = yaml.dump(status_data)
        self.ekf_status_pub.publish(status_msg)
    
    def _publish_parameter_change_log(self, update_record: Dict[str, Any]):
        """Publish parameter change log"""
        
        log_msg = String()
        log_msg.data = yaml.dump(update_record)
        self.param_log_pub.publish(log_msg)
    
    def _publish_current_covariances(self):
        """Publish current covariance matrix values"""
        
        # Extract covariance values in order
        covariance_values = []
        for mapping in self.parameter_mappings:
            value = self.current_parameters.get(mapping.ekf_parameter, mapping.baseline_value)
            covariance_values.append(value)
        
        covar_msg = Float64MultiArray()
        covar_msg.data = covariance_values
        self.current_covariances_pub.publish(covar_msg)
    
    def reset_to_baseline(self):
        """Reset all parameters to baseline values"""
        
        self.get_logger().info("Resetting EKF parameters to baseline values")
        
        param_updates = {}
        for mapping in self.parameter_mappings:
            current_value = self.current_parameters.get(mapping.ekf_parameter, mapping.baseline_value)
            if abs(current_value - mapping.baseline_value) > 1e-6:
                param_updates[mapping.ekf_parameter] = mapping.baseline_value
        
        if param_updates:
            self._apply_parameter_updates(param_updates, "Manual reset to baseline")
    
    def emergency_stop_adjustments(self):
        """Emergency stop of parameter adjustments"""
        
        self.get_logger().warn("Emergency stop - disabling parameter adjustments")
        self.enable_parameter_adjustment = False
        
        # Reset to baseline as safety measure
        self.reset_to_baseline()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = EKFCovarianceAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down EKF Covariance Adapter")
        # Reset to baseline on shutdown
        node.reset_to_baseline()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
