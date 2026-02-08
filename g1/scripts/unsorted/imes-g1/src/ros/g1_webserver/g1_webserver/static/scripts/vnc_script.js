import RFB from '../scripts/noVNC1.3.0/core/rfb.js';

class VNCClient {
    constructor() {
        this.rfb = null;
        this.reconnectTimeout = null;
        this.maxReconnectAttempts = 0;
        this.currentReconnectAttempt = 0;

        this.initializeElements();
        this.bindEvents();
        this.updateUI('disconnected');

        this.handleConnected = this.handleConnected.bind(this);
        this.handleDisconnected = this.handleDisconnected.bind(this);
        this.handleCredentialsRequired = this.handleCredentialsRequired.bind(this);
        this.handleSecurityFailure = this.handleSecurityFailure.bind(this);
        this.handleClipboard = this.handleClipboard.bind(this);
    }

    initializeElements() {
        this.elements = {
            screen: document.getElementById('vnc-screen'),
            connectBtn: document.getElementById('vnc-connect-btn'),
            disconnectBtn: document.getElementById('vnc-disconnect-btn'),
            statusIndicator: document.getElementById('vnc-status'),
            passwordInput: document.getElementById('vnc-password'),
            hostInput: document.getElementById('vnc-host'),
            portInput: document.getElementById('vnc-port')
        };

        // Validate all required elements exist
        for (const [key, element] of Object.entries(this.elements)) {
            if (!element) {
                throw new Error(`Required element not found: ${key}`);
            }
        }
    }

    bindEvents() {
        this.elements.connectBtn.addEventListener('click', () => this.connect());
        this.elements.disconnectBtn.addEventListener('click', () => this.disconnect());

        // Restart VNC button
        const restartBtn = document.getElementById('vnc-restart-btn');
        if (restartBtn) {
            restartBtn.addEventListener('click', () => this.restartVNCServer());
        }

        // Allow Enter key to trigger connection
        this.elements.passwordInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !this.elements.connectBtn.disabled) {
                this.connect();
            }
        });
    }

    async restartVNCServer() {
        try {
            this.updateUI('connecting', 'Restarting VNC server...');

            const response = await fetch('/restart_vnc', {
                method: 'POST'
            });

            const result = await response.json();

            if (response.ok && result.status === 'success') {
                this.updateUI('disconnected', 'VNC server restarted');
                console.log('VNC server restarted successfully');
            } else {
                throw new Error(result.message || 'Unknown error');
            }
        } catch (error) {
            this.handleError('Restart failed', error.message);
        }
    }

    validateInputs() {
        const host = this.elements.hostInput.value.trim();
        const port = this.elements.portInput.value.trim();

        if (!host) {
            throw new Error('Host is required');
        }

        if (!port || isNaN(port) || port < 1 || port > 65535) {
            throw new Error('Valid port number (1-65535) is required');
        }

        return { host, port: parseInt(port) };
    }

    async connect() {
        try {
            // Clear any existing reconnect timeout
            this.clearReconnectTimeout();

            // Disconnect existing connection if any
            if (this.rfb) {
                await this.disconnect();
            }

            const { host, port } = this.validateInputs();
            const password = this.elements.passwordInput.value;
            const websocketUrl = `ws://${host}:${port}`;

            this.updateUI('connecting');

            this.rfb = new RFB(this.elements.screen, websocketUrl, {
                credentials: { password: password }
            });

            this.setupRFBEventListeners();
            this.configureRFBSettings();

        } catch (error) {
            this.handleError('Connection failed', error.message);
        }
    }

    setupRFBEventListeners() {
        this.rfb.addEventListener('connect', () => this.handleConnected());
        this.rfb.addEventListener('disconnect', (e) => this.handleDisconnected(e));
        this.rfb.addEventListener('credentialsrequired', () => this.handleCredentialsRequired());
        this.rfb.addEventListener('securityfailure', (e) => this.handleSecurityFailure(e));
        this.rfb.addEventListener('clipboard', (e) => this.handleClipboard(e));
    }

    removeRFBEventListeners() {
        if (!this.rfb) return;
        this.rfb.removeEventListener('connect', this.handleConnected);
        this.rfb.removeEventListener('disconnect', this.handleDisconnected);
        this.rfb.removeEventListener('credentialsrequired', this.handleCredentialsRequired);
        this.rfb.removeEventListener('securityfailure', this.handleSecurityFailure);
        this.rfb.removeEventListener('clipboard', this.handleClipboard);
    }

    configureRFBSettings() {
        this.rfb.scaleViewport = true;
        this.rfb.resizeSession = true;
    }

    async disconnect() {
        return new Promise((resolve) => {
            if (this.rfb) {
                this.clearReconnectTimeout();
                this.removeRFBEventListeners();

                // Remove event listeners to prevent unwanted callbacks
                const tempRfb = this.rfb;
                this.rfb = null;

                tempRfb.disconnect();
                resolve();
            } else {
                resolve();
            }
        });
    }

    handleConnected() {
        this.currentReconnectAttempt = 0;
        this.updateUI('connected');
        console.log('VNC connection established successfully');
    }

    handleDisconnected(event) {
        const isCleanDisconnect = event.detail.clean;

        if (isCleanDisconnect) {
            this.updateUI('disconnected', 'Disconnected');
        } else {
            this.updateUI('disconnected', 'Connection lost');
            this.attemptReconnect();
        }

        this.rfb = null;
    }

    handleCredentialsRequired() {
        this.updateUI('error', 'Authentication required - please check password');
    }

    handleSecurityFailure(event) {
        const errorMsg = `Security failure: ${event.detail.status}`;
        this.updateUI('error', errorMsg);
    }

    handleClipboard(event) {
        console.log('Clipboard data received:', event.detail.text);

        // Optional: Copy to system clipboard if supported
        if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(event.detail.text).catch(err => {
                console.warn('Failed to copy to system clipboard:', err);
            });
        }
    }

    attemptReconnect() {
        if (this.currentReconnectAttempt >= this.maxReconnectAttempts) {
            this.updateUI('error', `Failed to reconnect after ${this.maxReconnectAttempts} attempts`);
            return;
        }

        this.currentReconnectAttempt++;
        const delay = Math.min(5000 * this.currentReconnectAttempt, 30000); // Exponential backoff, max 30s

        this.updateUI('connecting', `Reconnecting in ${delay / 1000}s... (attempt ${this.currentReconnectAttempt}/${this.maxReconnectAttempts})`);

        this.reconnectTimeout = setTimeout(() => {
            this.connect();
        }, delay);
    }

    clearReconnectTimeout() {
        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }
    }

    updateUI(state, message = null) {
        const states = {
            disconnected: {
                status: message || 'Disconnected',
                color: '#cf2323',
                connectDisabled: false,
                disconnectDisabled: true
            },
            connecting: {
                status: message || 'Connecting...',
                color: 'orange',
                connectDisabled: true,
                disconnectDisabled: false
            },
            connected: {
                status: message || 'Connected',
                color: '#3d8a3d',
                connectDisabled: true,
                disconnectDisabled: false
            },
            error: {
                status: message || 'Error',
                color: 'red',
                connectDisabled: false,
                disconnectDisabled: true
            }
        };

        const config = states[state];
        if (!config) return;

        this.elements.statusIndicator.textContent = config.status;
        this.elements.statusIndicator.style.color = config.color;
        this.elements.connectBtn.disabled = config.connectDisabled;
        this.elements.disconnectBtn.disabled = config.disconnectDisabled;
    }

    handleError(title, message) {
        console.error(`${title}: ${message}`);
        this.updateUI('error', `${title}: ${message}`);
    }

    destroy() {
        this.clearReconnectTimeout();
        if (this.rfb) {
            this.rfb.disconnect();
            this.rfb = null;
        }
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', function () {
    try {
        window.vncClient = new VNCClient();

        // Cleanup on page unload
        window.addEventListener('beforeunload', () => {
            if (window.vncClient) {
                window.vncClient.destroy();
            }
        });

    } catch (error) {
        console.error('Failed to initialize VNC client:', error);
        alert('Failed to initialize VNC client. Please check the console for details.');
    }
});