#!/usr/bin/env python3
"""
Brewery Controller
Main Flask application with PID temperature control
"""

from flask import Flask, render_template, jsonify, request
import json
from datetime import datetime
import threading
import time

# Import hardware modules
try:
    import board
    import busio
    import adafruit_bme280
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("WARNING: Hardware libraries not available. Running in SIMULATION mode.")

# Import PID controller
from simple_pid import PID

app = Flask(__name__)

# Configuration
class Config:
    # BME280 Sensors - Using different I2C addresses
    SENSOR_1_ADDRESS = 0x76  # HLT (Hot Liquor Tank)
    SENSOR_2_ADDRESS = 0x77  # Mash Tun
    
    # GPIO Pin Configuration for 8-Channel Relay Board
    # Relay pins (BCM numbering)
    RELAY_PINS = {
        'heater_1': 17,      # Channel 1 - HLT Heater (will use SSR later)
        'heater_2': 27,      # Channel 2 - Mash Heater (will use SSR later)
        'pump_1': 22,        # Channel 3 - HLT to Mash pump
        'pump_2': 23,        # Channel 4 - Mash to Boil pump
        'spare_1': 24,       # Channel 5 - Spare
        'spare_2': 25,       # Channel 6 - Spare
        'spare_3': 5,        # Channel 7 - Spare
        'spare_4': 6         # Channel 8 - Spare
    }
    
    # Temperature setpoints (Fahrenheit)
    DEFAULT_HLT_TEMP = 170.0
    DEFAULT_MASH_TEMP = 152.0
    
    # PID parameters - Tuned for heating control
    # These work well for most systems, but you may need to adjust
    PID_KP = 8.0      # Proportional gain
    PID_KI = 0.05     # Integral gain
    PID_KD = 2.0      # Derivative gain
    
    # Control parameters
    PID_UPDATE_INTERVAL = 2.0  # Seconds between PID updates
    TEMP_TOLERANCE = 1.0       # Degrees F - acceptable temperature range

config = Config()

# Hardware control class
class BreweryHardware:
    def __init__(self):
        self.simulation_mode = not HARDWARE_AVAILABLE
        self.sensors = {}
        self.relay_states = {name: False for name in config.RELAY_PINS.keys()}
        
        # Simulation data
        self.sim_temps = {
            'hlt': 68.0,
            'mash': 68.0
        }
        
        # Auto mode flags
        self.auto_mode = {
            'hlt': False,
            'mash': False
        }
        
        # Temperature setpoints
        self.setpoints = {
            'hlt': config.DEFAULT_HLT_TEMP,
            'mash': config.DEFAULT_MASH_TEMP
        }
        
        # PID controllers
        self.pid_controllers = {
            'hlt': PID(config.PID_KP, config.PID_KI, config.PID_KD, setpoint=config.DEFAULT_HLT_TEMP),
            'mash': PID(config.PID_KP, config.PID_KI, config.PID_KD, setpoint=config.DEFAULT_MASH_TEMP)
        }
        
        # Configure PID controllers
        for vessel, pid in self.pid_controllers.items():
            pid.output_limits = (0, 100)  # 0-100% output
            pid.sample_time = config.PID_UPDATE_INTERVAL
        
        # Control thread
        self.control_thread = None
        self.control_running = False
        
        if not self.simulation_mode:
            self._init_sensors()
            self._init_relays()
        else:
            print("Running in SIMULATION mode - no hardware control")
        
        # Start control loop
        self.start_control_loop()
    
    def _init_sensors(self):
        """Initialize BME280 sensors"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize sensor 1 (HLT)
            try:
                self.sensors['hlt'] = adafruit_bme280.Adafruit_BME280_I2C(
                    i2c, address=config.SENSOR_1_ADDRESS
                )
                self.sensors['hlt'].sea_level_pressure = 1013.25
                print(f"HLT sensor initialized at 0x{config.SENSOR_1_ADDRESS:02x}")
            except Exception as e:
                print(f"Error initializing HLT sensor: {e}")
            
            # Initialize sensor 2 (Mash)
            try:
                self.sensors['mash'] = adafruit_bme280.Adafruit_BME280_I2C(
                    i2c, address=config.SENSOR_2_ADDRESS
                )
                self.sensors['mash'].sea_level_pressure = 1013.25
                print(f"Mash sensor initialized at 0x{config.SENSOR_2_ADDRESS:02x}")
            except Exception as e:
                print(f"Error initializing Mash sensor: {e}")
                
        except Exception as e:
            print(f"Error initializing I2C: {e}")
    
    def _init_relays(self):
        """Initialize GPIO for relay control"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            for name, pin in config.RELAY_PINS.items():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.HIGH)  # Relays are active LOW
                print(f"Initialized relay '{name}' on GPIO {pin}")
                
        except Exception as e:
            print(f"Error initializing relays: {e}")
    
    def read_sensors(self):
        """Read temperature data from sensors"""
        data = {
            'hlt': {'temp_f': None, 'temp_c': None, 'humidity': None, 'pressure': None},
            'mash': {'temp_f': None, 'temp_c': None, 'humidity': None, 'pressure': None},
            'timestamp': datetime.now().isoformat()
        }
        
        if self.simulation_mode:
            # Simulate temperature changes
            data['hlt']['temp_c'] = self.sim_temps['hlt']
            data['hlt']['temp_f'] = self.sim_temps['hlt'] * 9/5 + 32
            data['hlt']['humidity'] = 45.0
            data['hlt']['pressure'] = 1013.25
            
            data['mash']['temp_c'] = self.sim_temps['mash']
            data['mash']['temp_f'] = self.sim_temps['mash'] * 9/5 + 32
            data['mash']['humidity'] = 45.0
            data['mash']['pressure'] = 1013.25
        else:
            # Read from actual sensors
            for vessel in ['hlt', 'mash']:
                if vessel in self.sensors:
                    try:
                        sensor = self.sensors[vessel]
                        temp_c = sensor.temperature
                        temp_f = temp_c * 9/5 + 32
                        
                        data[vessel]['temp_c'] = round(temp_c, 2)
                        data[vessel]['temp_f'] = round(temp_f, 2)
                        data[vessel]['humidity'] = round(sensor.humidity, 2)
                        data[vessel]['pressure'] = round(sensor.pressure, 2)
                    except Exception as e:
                        print(f"Error reading {vessel} sensor: {e}")
        
        return data
    
    def set_relay(self, relay_name, state):
        """Control relay state (True = ON, False = OFF)"""
        if relay_name not in config.RELAY_PINS:
            return False
        
        self.relay_states[relay_name] = state
        
        if self.simulation_mode:
            print(f"SIM: Relay '{relay_name}' set to {'ON' if state else 'OFF'}")
        else:
            try:
                pin = config.RELAY_PINS[relay_name]
                # Relays are active LOW (LOW = ON, HIGH = OFF)
                GPIO.output(pin, GPIO.LOW if state else GPIO.HIGH)
                print(f"Relay '{relay_name}' (GPIO {pin}) set to {'ON' if state else 'OFF'}")
            except Exception as e:
                print(f"Error setting relay {relay_name}: {e}")
                return False
        
        return True
    
    def get_relay_states(self):
        """Get current state of all relays"""
        return self.relay_states.copy()
    
    def set_auto_mode(self, vessel, enabled):
        """Enable or disable automatic temperature control for a vessel"""
        if vessel not in ['hlt', 'mash']:
            return False
        
        self.auto_mode[vessel] = enabled
        print(f"Auto mode for {vessel.upper()}: {'ENABLED' if enabled else 'DISABLED'}")
        
        # If disabling auto mode, turn off the heater
        if not enabled:
            heater_relay = 'heater_1' if vessel == 'hlt' else 'heater_2'
            self.set_relay(heater_relay, False)
        
        return True
    
    def set_setpoint(self, vessel, temperature):
        """Set target temperature for a vessel"""
        if vessel not in ['hlt', 'mash']:
            return False
        
        self.setpoints[vessel] = float(temperature)
        self.pid_controllers[vessel].setpoint = float(temperature)
        print(f"Setpoint for {vessel.upper()} set to {temperature}°F")
        return True
    
    def control_loop(self):
        """Main control loop for PID temperature control"""
        print("Control loop started")
        
        while self.control_running:
            try:
                # Read current temperatures
                sensor_data = self.read_sensors()
                
                # Control HLT
                if self.auto_mode['hlt'] and sensor_data['hlt']['temp_f'] is not None:
                    current_temp = sensor_data['hlt']['temp_f']
                    pid_output = self.pid_controllers['hlt'](current_temp)
                    
                    # Simple on/off control based on PID output
                    # If PID says we need >50% power, turn heater on
                    heater_state = pid_output > 50
                    self.set_relay('heater_1', heater_state)
                    
                    if self.simulation_mode:
                        # Simulate heating/cooling
                        if heater_state:
                            self.sim_temps['hlt'] = min(self.sim_temps['hlt'] + 0.3, 110)
                        else:
                            self.sim_temps['hlt'] = max(self.sim_temps['hlt'] - 0.1, 20)
                
                # Control Mash
                if self.auto_mode['mash'] and sensor_data['mash']['temp_f'] is not None:
                    current_temp = sensor_data['mash']['temp_f']
                    pid_output = self.pid_controllers['mash'](current_temp)
                    
                    # Simple on/off control based on PID output
                    heater_state = pid_output > 50
                    self.set_relay('heater_2', heater_state)
                    
                    if self.simulation_mode:
                        # Simulate heating/cooling
                        if heater_state:
                            self.sim_temps['mash'] = min(self.sim_temps['mash'] + 0.3, 110)
                        else:
                            self.sim_temps['mash'] = max(self.sim_temps['mash'] - 0.1, 20)
                
            except Exception as e:
                print(f"Error in control loop: {e}")
            
            # Wait before next update
            time.sleep(config.PID_UPDATE_INTERVAL)
    
    def start_control_loop(self):
        """Start the control loop thread"""
        if not self.control_running:
            self.control_running = True
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            print("Control loop thread started")
    
    def stop_control_loop(self):
        """Stop the control loop thread"""
        self.control_running = False
        if self.control_thread:
            self.control_thread.join(timeout=5)
    
    def get_status(self):
        """Get current control status"""
        return {
            'auto_mode': self.auto_mode.copy(),
            'setpoints': self.setpoints.copy()
        }
    
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        self.stop_control_loop()
        
        if not self.simulation_mode:
            # Turn off all relays
            for name in config.RELAY_PINS.keys():
                self.set_relay(name, False)
            GPIO.cleanup()

# Global hardware instance
hardware = BreweryHardware()

# Routes
@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html')

@app.route('/api/sensor-data')
def get_sensor_data():
    """Get current sensor readings"""
    data = hardware.read_sensors()
    return jsonify(data)

@app.route('/api/relay-states')
def get_relay_states():
    """Get current relay states"""
    return jsonify(hardware.get_relay_states())

@app.route('/api/relay/<relay_name>/<action>', methods=['POST'])
def control_relay(relay_name, action):
    """Control a specific relay (manual mode only)"""
    if relay_name not in config.RELAY_PINS:
        return jsonify({'success': False, 'error': 'Invalid relay name'}), 400
    
    if action not in ['on', 'off']:
        return jsonify({'success': False, 'error': 'Invalid action'}), 400
    
    # Check if this is a heater and auto mode is enabled
    if relay_name == 'heater_1' and hardware.auto_mode['hlt']:
        return jsonify({'success': False, 'error': 'HLT is in AUTO mode. Disable auto mode first.'}), 400
    
    if relay_name == 'heater_2' and hardware.auto_mode['mash']:
        return jsonify({'success': False, 'error': 'Mash is in AUTO mode. Disable auto mode first.'}), 400
    
    state = (action == 'on')
    success = hardware.set_relay(relay_name, state)
    
    return jsonify({
        'success': success,
        'relay': relay_name,
        'state': state
    })

@app.route('/api/auto-mode/<vessel>/<action>', methods=['POST'])
def set_auto_mode(vessel, action):
    """Enable or disable automatic temperature control"""
    if vessel not in ['hlt', 'mash']:
        return jsonify({'success': False, 'error': 'Invalid vessel'}), 400
    
    if action not in ['enable', 'disable']:
        return jsonify({'success': False, 'error': 'Invalid action'}), 400
    
    enabled = (action == 'enable')
    success = hardware.set_auto_mode(vessel, enabled)
    
    return jsonify({
        'success': success,
        'vessel': vessel,
        'auto_mode': enabled
    })

@app.route('/api/setpoint/<vessel>', methods=['POST'])
def set_setpoint(vessel):
    """Set target temperature for a vessel"""
    if vessel not in ['hlt', 'mash']:
        return jsonify({'success': False, 'error': 'Invalid vessel'}), 400
    
    data = request.get_json()
    if not data or 'temperature' not in data:
        return jsonify({'success': False, 'error': 'Temperature not provided'}), 400
    
    try:
        temp = float(data['temperature'])
        if temp < 32 or temp > 212:
            return jsonify({'success': False, 'error': 'Temperature out of range (32-212°F)'}), 400
        
        success = hardware.set_setpoint(vessel, temp)
        
        return jsonify({
            'success': success,
            'vessel': vessel,
            'setpoint': temp
        })
    except ValueError:
        return jsonify({'success': False, 'error': 'Invalid temperature value'}), 400

@app.route('/api/status')
def get_status():
    """Get current control status"""
    return jsonify(hardware.get_status())

@app.route('/api/config')
def get_config():
    """Get current configuration"""
    return jsonify({
        'simulation_mode': hardware.simulation_mode,
        'relays': list(config.RELAY_PINS.keys()),
        'default_temps': {
            'hlt': config.DEFAULT_HLT_TEMP,
            'mash': config.DEFAULT_MASH_TEMP
        }
    })

# Cleanup on shutdown
import atexit
atexit.register(hardware.cleanup)

if __name__ == '__main__':
    print("=" * 50)
    print("Brewery Controller Starting")
    print("=" * 50)
    print(f"Mode: {'SIMULATION' if hardware.simulation_mode else 'HARDWARE'}")
    print(f"Web interface: http://localhost:5000")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
