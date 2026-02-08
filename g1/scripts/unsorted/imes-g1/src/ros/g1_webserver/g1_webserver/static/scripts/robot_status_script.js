/*
Software License Agreement (BSD)

@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
*/
document.addEventListener('DOMContentLoaded', () => {
    const statusInfo = document.getElementById('statusInfo');
    const responseMessage = document.getElementById('responseMessage');

    // Function to fetch robot statuses from the server
    function fetchStatus() {
        fetch('/status')  // Ensure this matches your server endpoint
            .then(response => response.json())
            .then(data => {
                statusInfo.innerHTML = '';  // Clear previous status
                if (data.statuses && Array.isArray(data.statuses)) {
                    // Iterate over the statuses and display them with colors
                    data.statuses.forEach(status => {
                        const statusItem = document.createElement('div');
                        statusItem.className = 'status-item';

                        // Split status text by the first colon
                        const [beforeColon, afterColon] = status.split(/:(.*)/); // Split only on the first colon

                        // Create reset and deactivate buttons
                        const miniresetButton = document.createElement('button');
                        miniresetButton.className = 'mini-button';
                        miniresetButton.onclick = () => resetComponent(beforeColon);
                        miniresetButton.style.marginRight = '15px';
                        miniresetButton.style.marginLeft = '20px';


                        const resetIcon = document.createElement('i');
                        resetIcon.className = 'fas fa-redo';  // "redo" icon for restart/reset
                        miniresetButton.appendChild(resetIcon);

                        const minideactivateButton = document.createElement('button');
                        minideactivateButton.className = 'mini-button-r';
                        minideactivateButton.onclick = () => deactivateComponent(beforeColon);
                        minideactivateButton.style.marginRight = '30px';

                        const deactivateIcon = document.createElement('i');
                        deactivateIcon.className = 'fas fa-power-off';  // power-off icon
                        minideactivateButton.appendChild(deactivateIcon);

                        // Create span for the default colored part
                        const defaultSpan = document.createElement('span');
                        defaultSpan.innerText = beforeColon + ': ';
                        defaultSpan.style.color = 'inherit'; // Default color

                        // Create span for the colored part
                        const coloredSpan = document.createElement('span');
                        coloredSpan.innerText = afterColon || ''; // Handle missing after-colon case
                        coloredSpan.style.color = getStatusColor(status); // Apply status color

                        // Append buttons first
                        statusItem.appendChild(miniresetButton);
                        statusItem.appendChild(minideactivateButton);

                        // Then append text
                        statusItem.appendChild(defaultSpan);
                        statusItem.appendChild(coloredSpan);
                        statusItem.style.fontSize = 'var(--project-font-size-normal)';

                        // Append the status item to the container
                        statusInfo.appendChild(statusItem);
                    });
                } else {
                    responseMessage.innerText = 'No status data available.';
                }
            })
            .catch(error => {
                responseMessage.innerText = 'Error fetching status.';
                console.error(error);
            });
    }

    // Function to get color based on the status/mission
    function getStatusColor(text) {
        if (text.includes('Active')) {
            return 'var(--project-color)';  // Active mission or status
        } else {
            return 'var(--project-error-color)';  // Default color for other statuses
        }
    }

    // Function to reset a specific component
    function resetComponent(componentName) {
        const audio = document.getElementById('buttonClickSound');
        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }
        
        fetch('/reset_component', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ component: componentName }),
        })
            .then(response => response.json())
            .then(data => {
                fetchStatus(); // Refresh status
                responseMessage.innerText = data.message;
            })
            .catch(error => {
                console.error(error);
            });
    }

    // Function to deactivate a specific component
    function deactivateComponent(componentName) {
        const audio = document.getElementById('buttonClickSound');
        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }

        fetch('/deactivate_component', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ component: componentName }),
        })
            .then(response => response.json())
            .then(data => {
                fetchStatus(); // Refresh status
                responseMessage.innerText = data.message;
            })
            .catch(error => {
                console.error(error);
            });
    }

    // Function to re-activate all the services
    window.reactivateServices = function () {
        const audio = document.getElementById('buttonReactivateAllServicesSound');
        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }

        fetch('/reactivate_services', {
            method: 'POST',
        })
            .then(response => response.json())
            .then(data => {
                responseMessage.innerText = 'All services reactivated';
            })
            .catch(error => {
                alert('Error in re-actiavation of services');
                console.error(error);
            });
    };

    // Function to de-activate all the services
    window.deactivateServices = function () {
        const audio = document.getElementById('buttonDeactivateAllServicesSound');
        if (audio) {
            audio.currentTime = 0;  // Rewind to start
            audio.play();
        }

        fetch('/deactivate_services', {
            method: 'POST',
        })
            .then(response => response.json())
            .then(data => {
                responseMessage.innerText = 'All services deactivated';
            })
            .catch(error => {
                alert('Error in de-actiavation of services');
                console.error(error);
            });
    };

    fetchStatus();
    setInterval(fetchStatus, 2000);
});