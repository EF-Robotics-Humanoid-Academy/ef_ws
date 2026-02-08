/*
Software License Agreement (BSD)

@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
*/
document.addEventListener('DOMContentLoaded', () => {
    // ------------------------------------------------------------------------------ Joystick Setup for two joysticks

    function setupJoystick(joystickId, zoneId) {
        let joystickInterval = null;
        let lastAngle = 0;
        let lastDistance = 0;

        // Calculate responsive size based on container
        const container = document.getElementById(zoneId);
        const containerWidth = container.offsetWidth;
        const joystickSize = Math.min(containerWidth, 200);

        const joystick = nipplejs.create({
            zone: container,
            color: 'var(--project-color)',
            mode: 'dynamic',
            size: joystickSize,
        });

        joystick.on('move', (event, data) => {
            lastAngle = data.angle.degree;
            lastDistance = data.distance;

            if (!joystickInterval) {
                joystickInterval = setInterval(() => {
                    sendJoystickDataToBackend(joystickId, lastAngle, lastDistance);
                }, 100);
            }
        });

        joystick.on('end', () => {
            clearInterval(joystickInterval);
            joystickInterval = null;
            sendJoystickDataToBackend(joystickId, 0, 0); // stop movement for this joystick
        });
    }

    function sendJoystickDataToBackend(joystickId, angle, distance) {
        fetch('/web_joystick_control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ joystickId, angle, distance }),
        })
        .then(response => response.json())
        .then(data => {
            // Optional: show status per joystick if you want
            // console.log(`Joystick ${joystickId} data sent`);
        })
        .catch(error => {
            console.error(`Error sending joystick ${joystickId} data:`, error);
        });
    }

    // Initialize both joysticks with their respective zone IDs and IDs
    setupJoystick('joystick1', 'joystick1');
    setupJoystick('joystick2', 'joystick2');
    setupJoystick('joystick1', 'joystick1b');
    setupJoystick('joystick2', 'joystick2b');
});
