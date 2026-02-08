/*
Software License Agreement (BSD)

@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
*/
document.addEventListener('DOMContentLoaded', () => {
    const terminalInput = document.getElementById('terminal_input');

    // Function for terminal submission
    window.sendTerminalData = function () {
        const audio = document.getElementById('buttonClickSound');
        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }

        const data = {
            terminal_input: terminalInput.value
        };

        fetch('/terminal_submission', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data),
        })
            .then(response => response.json())
            .then(responseData => {
                const responseMessage = document.getElementById('responseMessage');
                responseMessage.innerText = `${responseData.message}`;
                responseMessage.style.color = 'var(--project-color)';
            })
            .catch(() => {
                const responseMessage = document.getElementById('responseMessage');
                responseMessage.innerText = 'Error sending data';
                responseMessage.style.color = 'var(--project-error-color)';
            });
    };

    window.sendClearData = function () {
        const responseMessage = document.getElementById('responseMessage');
        responseMessage.innerText = 'Logs Cleared';
        responseMessage.style.color = '';
    };

    // Function for WebButton
    window.webButton = function (action) {

        const customAudio = document.getElementById(action + 'Sound');
        const defaultAudio = document.getElementById('buttonClickSound');

        const audio = customAudio || defaultAudio; // Prefer custom, fallback to default

        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }

        fetch('/web_button', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                action: action,
            }),
        })
            .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                return response.json();
            })
            .then(data => {
                console.log('Server Response:', data);

                // Display success message dynamically
                const responseMessage = document.getElementById('responseMessage');
                responseMessage.innerText = data.message || `Action "${action}" performed successfully.`;
                responseMessage.style.color = 'var(--project-color)';
            })
            .catch(error => {
                console.error('Error:', error);

                // Display error message dynamically
                const responseMessage = document.getElementById('responseMessage');
                responseMessage.innerText = `Failed to perform action "${action}".`;
                responseMessage.style.color = 'var(--project-error-color)';
            });
    };
});
