// Brewery Controller JavaScript

class BreweryController {
    constructor() {
        this.updateInterval = null;
        this.config = null;
        this.init();
    }

    async init() {
        console.log('Initializing Brewery Controller...');
        
        // Load configuration
        await this.loadConfig();
        
        // Set up event listeners
        this.setupEventListeners();
        
        // Start data updates
        this.startUpdates();
        
        // Initial data fetch
        this.updateSensorData();
        this.updateRelayStates();
    }

    async loadConfig() {
        try {
            const response = await fetch('/api/config');
            this.config = await response.json();
            
            // Update mode badge
            const modeBadge = document.getElementById('mode-badge');
            if (this.config.simulation_mode) {
                modeBadge.textContent = '⚙️ SIMULATION MODE';
                modeBadge.classList.add('simulation');
            } else {
                modeBadge.textContent = '🔧 HARDWARE MODE';
                modeBadge.classList.add('hardware');
            }
            
            console.log('Configuration loaded:', this.config);
        } catch (error) {
            console.error('Error loading configuration:', error);
        }
    }

    setupEventListeners() {
        // Relay control buttons
        document.querySelectorAll('.relay-btn').forEach(btn => {
            btn.addEventListener('click', (e) => this.toggleRelay(e.target.closest('.relay-btn')));
        });

        // Emergency stop button
        document.getElementById('emergency-stop').addEventListener('click', () => {
            this.emergencyStop();
        });

        // Setpoint buttons
        document.querySelectorAll('.setpoint-btn').forEach(btn => {
            btn.addEventListener('click', (e) => this.setSetpoint(e.target.closest('.setpoint-btn')));
        });

        // Auto mode buttons
        document.querySelectorAll('.auto-btn').forEach(btn => {
            btn.addEventListener('click', (e) => this.toggleAutoMode(e.target.closest('.auto-btn')));
        });
    }

    startUpdates() {
        // Update sensor data every 2 seconds
        this.updateInterval = setInterval(() => {
            this.updateSensorData();
        }, 2000);
    }

    async updateSensorData() {
        try {
            const response = await fetch('/api/sensor-data');
            const data = await response.json();
            
            // Update HLT values
            if (data.hlt.temp_f !== null) {
                document.getElementById('hlt-temp').textContent = data.hlt.temp_f.toFixed(1);
                document.getElementById('hlt-humidity').textContent = 
                    data.hlt.humidity !== null ? `${data.hlt.humidity.toFixed(1)}%` : '--';
                document.getElementById('hlt-pressure').textContent = 
                    data.hlt.pressure !== null ? `${data.hlt.pressure.toFixed(1)} hPa` : '--';
            }
            
            // Update Mash values
            if (data.mash.temp_f !== null) {
                document.getElementById('mash-temp').textContent = data.mash.temp_f.toFixed(1);
                document.getElementById('mash-humidity').textContent = 
                    data.mash.humidity !== null ? `${data.mash.humidity.toFixed(1)}%` : '--';
                document.getElementById('mash-pressure').textContent = 
                    data.mash.pressure !== null ? `${data.mash.pressure.toFixed(1)} hPa` : '--';
            }
            
            // Update last update time
            const now = new Date();
            document.getElementById('last-update').textContent = 
                `Last update: ${now.toLocaleTimeString()}`;
            
            // Update connection status
            document.getElementById('connection-status').className = 'connection-ok';
            document.getElementById('connection-status').textContent = '● Connected';
            
        } catch (error) {
            console.error('Error updating sensor data:', error);
            document.getElementById('connection-status').className = 'connection-error';
            document.getElementById('connection-status').textContent = '● Connection Error';
        }
    }

    async updateRelayStates() {
        try {
            const response = await fetch('/api/relay-states');
            const states = await response.json();
            
            // Update button states
            for (const [relay, state] of Object.entries(states)) {
                const btn = document.querySelector(`[data-relay="${relay}"]`);
                if (btn) {
                    this.updateButtonState(btn, state);
                }
            }
        } catch (error) {
            console.error('Error updating relay states:', error);
        }
    }

    async toggleRelay(btn) {
        const relayName = btn.dataset.relay;
        const currentState = btn.dataset.state;
        const newAction = currentState === 'off' ? 'on' : 'off';
        
        try {
            const response = await fetch(`/api/relay/${relayName}/${newAction}`, {
                method: 'POST'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.updateButtonState(btn, result.state);
                console.log(`Relay ${relayName} turned ${newAction}`);
            } else {
                console.error('Failed to toggle relay:', result.error);
                alert(`Failed to toggle ${relayName}: ${result.error}`);
            }
        } catch (error) {
            console.error('Error toggling relay:', error);
            alert(`Error controlling ${relayName}`);
        }
    }

    updateButtonState(btn, state) {
        btn.dataset.state = state ? 'on' : 'off';
        btn.querySelector('.btn-label').textContent = state ? 'ON' : 'OFF';
    }

    async emergencyStop() {
        if (!confirm('Turn OFF all equipment?')) {
            return;
        }
        
        console.log('EMERGENCY STOP activated');
        
        // Turn off all relays
        const buttons = document.querySelectorAll('.relay-btn');
        const promises = [];
        
        buttons.forEach(btn => {
            const relayName = btn.dataset.relay;
            promises.push(
                fetch(`/api/relay/${relayName}/off`, { method: 'POST' })
                    .then(response => response.json())
                    .then(result => {
                        if (result.success) {
                            this.updateButtonState(btn, false);
                        }
                    })
            );
        });
        
        // Also disable auto modes
        const autoButtons = document.querySelectorAll('.auto-btn');
        autoButtons.forEach(btn => {
            const vessel = btn.dataset.vessel;
            promises.push(
                fetch(`/api/auto-mode/${vessel}/disable`, { method: 'POST' })
                    .then(response => response.json())
                    .then(result => {
                        if (result.success) {
                            this.updateAutoButtonState(btn, false);
                        }
                    })
            );
        });
        
        try {
            await Promise.all(promises);
            alert('All equipment has been turned OFF and auto modes disabled');
        } catch (error) {
            console.error('Error during emergency stop:', error);
            alert('Error during emergency stop - check equipment manually!');
        }
    }

    async setSetpoint(btn) {
        const vessel = btn.dataset.vessel;
        const input = document.getElementById(`${vessel}-setpoint`);
        const temperature = parseFloat(input.value);
        
        if (isNaN(temperature) || temperature < 32 || temperature > 212) {
            alert('Please enter a valid temperature between 32°F and 212°F');
            return;
        }
        
        try {
            const response = await fetch(`/api/setpoint/${vessel}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ temperature: temperature })
            });
            
            const result = await response.json();
            
            if (result.success) {
                console.log(`Setpoint for ${vessel} set to ${temperature}°F`);
                // Visual feedback
                btn.style.backgroundColor = 'rgba(76, 175, 80, 0.4)';
                setTimeout(() => {
                    btn.style.backgroundColor = '';
                }, 500);
            } else {
                alert(`Failed to set setpoint: ${result.error}`);
            }
        } catch (error) {
            console.error('Error setting setpoint:', error);
            alert('Error setting setpoint');
        }
    }

    async toggleAutoMode(btn) {
        const vessel = btn.dataset.vessel;
        const currentMode = btn.dataset.mode;
        const newAction = currentMode === 'off' ? 'enable' : 'disable';
        
        try {
            const response = await fetch(`/api/auto-mode/${vessel}/${newAction}`, {
                method: 'POST'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.updateAutoButtonState(btn, result.auto_mode);
                console.log(`Auto mode for ${vessel}: ${result.auto_mode ? 'ENABLED' : 'DISABLED'}`);
            } else {
                alert(`Failed to toggle auto mode: ${result.error}`);
            }
        } catch (error) {
            console.error('Error toggling auto mode:', error);
            alert('Error toggling auto mode');
        }
    }

    updateAutoButtonState(btn, enabled) {
        btn.dataset.mode = enabled ? 'on' : 'off';
        btn.querySelector('.auto-label').textContent = `AUTO MODE: ${enabled ? 'ON' : 'OFF'}`;
    }

    destroy() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
    }
}

// Initialize when page loads
let controller;
document.addEventListener('DOMContentLoaded', () => {
    controller = new BreweryController();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (controller) {
        controller.destroy();
    }
});
