#!/usr/bin/env python3
"""
Quadrature Encoder Reader for Jetson GPIO

Implements robust quadrature decoding with interrupt-based edge detection
for accurate tick counting and direction detection.
"""

import time
import threading
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass

try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("WARNING: Jetson.GPIO not available. Using simulation mode.")
    GPIO_AVAILABLE = False


@dataclass
class EncoderState:
    """State information for a single encoder"""
    ticks: int = 0
    last_a: bool = False
    last_b: bool = False
    direction: int = 1  # 1 for forward, -1 for reverse
    last_tick_time: float = 0.0
    rpm: float = 0.0


class QuadratureEncoder:
    """
    Quadrature encoder reader with interrupt-based decoding
    
    Handles A/B channel quadrature signals with debouncing and direction detection.
    Uses GPIO interrupts for real-time response and accurate tick counting.
    """
    
    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        name: str = "encoder",
        invert_direction: bool = False,
        pull_mode: str = "down",
        debounce_time: float = 0.001,
        callback: Optional[Callable] = None
    ):
        """
        Initialize quadrature encoder
        
        Args:
            pin_a (int): GPIO pin for channel A
            pin_b (int): GPIO pin for channel B  
            name (str): Encoder name for identification
            invert_direction (bool): Invert rotation direction
            pull_mode (str): Pull resistor mode ("up", "down", or "none")
            debounce_time (float): Debounce time in seconds
            callback (Callable): Optional callback on tick change
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.invert_direction = invert_direction
        self.debounce_time = debounce_time
        self.callback = callback
        
        # Encoder state
        self.state = EncoderState()
        self.lock = threading.Lock()
        
        # Simulation mode tracking
        self.simulation_mode = not GPIO_AVAILABLE
        self.sim_ticks = 0
        self.sim_direction = 1
        
        # GPIO setup
        if GPIO_AVAILABLE:
            self._setup_gpio(pull_mode)
        else:
            print(f"Encoder {self.name}: Running in simulation mode")
    
    def _setup_gpio(self, pull_mode: str):
        """Setup GPIO pins and interrupts"""
        # Set pin modes
        pull_resistor = GPIO.PUD_OFF
        if pull_mode.lower() == "up":
            pull_resistor = GPIO.PUD_UP
        elif pull_mode.lower() == "down":
            pull_resistor = GPIO.PUD_DOWN
            
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=pull_resistor)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=pull_resistor)
        
        # Read initial states
        self.state.last_a = GPIO.input(self.pin_a)
        self.state.last_b = GPIO.input(self.pin_b)
        
        # Setup interrupts on both edges for both channels
        GPIO.add_event_detect(
            self.pin_a, 
            GPIO.BOTH, 
            callback=self._interrupt_a,
            bouncetime=int(self.debounce_time * 1000)  # Convert to ms
        )
        GPIO.add_event_detect(
            self.pin_b, 
            GPIO.BOTH, 
            callback=self._interrupt_b,
            bouncetime=int(self.debounce_time * 1000)
        )
    
    def _interrupt_a(self, channel):
        """Interrupt handler for channel A"""
        self._process_interrupt('A')
    
    def _interrupt_b(self, channel):
        """Interrupt handler for channel B"""
        self._process_interrupt('B')
    
    def _process_interrupt(self, channel: str):
        """Process encoder interrupt and update tick count"""
        if not GPIO_AVAILABLE:
            return
            
        with self.lock:
            current_time = time.time()
            
            # Read current pin states
            current_a = GPIO.input(self.pin_a)
            current_b = GPIO.input(self.pin_b)
            
            # Determine if this is a valid state change
            if current_a == self.state.last_a and current_b == self.state.last_b:
                return  # No actual change (noise/bounce)
            
            # Quadrature decoding logic
            # Standard quadrature sequence: 00 -> 01 -> 11 -> 10 -> 00 (CW)
            # Reverse sequence: 00 -> 10 -> 11 -> 01 -> 00 (CCW)
            
            direction = 0
            if channel == 'A':
                if current_a != self.state.last_a:  # A changed
                    if current_a == current_b:
                        direction = -1  # CCW
                    else:
                        direction = 1   # CW
            else:  # channel == 'B'
                if current_b != self.state.last_b:  # B changed
                    if current_a == current_b:
                        direction = 1   # CW
                    else:
                        direction = -1  # CCW
            
            # Apply direction inversion if configured
            if self.invert_direction:
                direction = -direction
            
            # Update tick count if direction is valid
            if direction != 0:
                self.state.ticks += direction
                self.state.direction = direction
                
                # Calculate RPM (optional)
                if self.state.last_tick_time > 0:
                    time_diff = current_time - self.state.last_tick_time
                    if time_diff > 0:
                        # RPM calculation (very approximate)
                        self.state.rpm = abs(60.0 / (time_diff * 537.7))  # Assuming PPR
                
                self.state.last_tick_time = current_time
                
                # Call callback if provided
                if self.callback:
                    try:
                        self.callback(self.name, self.state.ticks, direction)
                    except Exception as e:
                        print(f"Encoder callback error: {e}")
            
            # Update last states
            self.state.last_a = current_a
            self.state.last_b = current_b
    
    def get_ticks(self) -> int:
        """Get current tick count"""
        if self.simulation_mode:
            # Simulate encoder movement for testing
            self.sim_ticks += self.sim_direction
            if abs(self.sim_ticks) % 100 == 0:  # Change direction occasionally
                self.sim_direction *= -1
            return self.sim_ticks
            
        with self.lock:
            return self.state.ticks
    
    def reset_ticks(self):
        """Reset tick counter to zero"""
        with self.lock:
            self.state.ticks = 0
            self.state.last_tick_time = 0.0
            if self.simulation_mode:
                self.sim_ticks = 0
    
    def get_direction(self) -> int:
        """Get last movement direction (1=forward, -1=reverse, 0=stopped)"""
        with self.lock:
            return self.state.direction
    
    def get_rpm(self) -> float:
        """Get estimated RPM"""
        with self.lock:
            return self.state.rpm
    
    def get_info(self) -> Dict[str, Any]:
        """Get complete encoder information"""
        with self.lock:
            return {
                'name': self.name,
                'ticks': self.state.ticks,
                'direction': self.state.direction,
                'rpm': self.state.rpm,
                'pin_a': self.pin_a,
                'pin_b': self.pin_b,
                'inverted': self.invert_direction,
                'simulation': self.simulation_mode
            }
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if GPIO_AVAILABLE:
            try:
                GPIO.remove_event_detect(self.pin_a)
                GPIO.remove_event_detect(self.pin_b)
            except Exception as e:
                print(f"Encoder cleanup error: {e}")


class EncoderManager:
    """
    Manager for multiple quadrature encoders
    
    Handles initialization, reading, and cleanup of multiple encoders
    with thread-safe operations and centralized management.
    """
    
    def __init__(self):
        """Initialize encoder manager"""
        self.encoders: Dict[str, QuadratureEncoder] = {}
        self.gpio_initialized = False
        
        # Initialize GPIO if available
        if GPIO_AVAILABLE and not self.gpio_initialized:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.gpio_initialized = True
    
    def add_encoder(
        self,
        name: str,
        pin_a: int,
        pin_b: int,
        invert_direction: bool = False,
        callback: Optional[Callable] = None
    ) -> QuadratureEncoder:
        """
        Add a new encoder to the manager
        
        Args:
            name (str): Unique encoder name
            pin_a (int): GPIO pin for channel A
            pin_b (int): GPIO pin for channel B
            invert_direction (bool): Invert direction
            callback (Callable): Optional callback function
            
        Returns:
            QuadratureEncoder: Created encoder instance
        """
        if name in self.encoders:
            raise ValueError(f"Encoder '{name}' already exists")
        
        encoder = QuadratureEncoder(
            pin_a=pin_a,
            pin_b=pin_b,
            name=name,
            invert_direction=invert_direction,
            callback=callback
        )
        
        self.encoders[name] = encoder
        return encoder
    
    def get_encoder(self, name: str) -> Optional[QuadratureEncoder]:
        """Get encoder by name"""
        return self.encoders.get(name)
    
    def get_all_ticks(self) -> Dict[str, int]:
        """Get tick counts for all encoders"""
        return {name: encoder.get_ticks() for name, encoder in self.encoders.items()}
    
    def reset_all_ticks(self):
        """Reset all encoder tick counters"""
        for encoder in self.encoders.values():
            encoder.reset_ticks()
    
    def get_all_info(self) -> Dict[str, Dict[str, Any]]:
        """Get information for all encoders"""
        return {name: encoder.get_info() for name, encoder in self.encoders.items()}
    
    def cleanup(self):
        """Clean up all encoders and GPIO"""
        for encoder in self.encoders.values():
            encoder.cleanup()
        
        if GPIO_AVAILABLE and self.gpio_initialized:
            try:
                GPIO.cleanup()
            except Exception as e:
                print(f"GPIO cleanup error: {e}")
        
        self.encoders.clear()
        self.gpio_initialized = False
