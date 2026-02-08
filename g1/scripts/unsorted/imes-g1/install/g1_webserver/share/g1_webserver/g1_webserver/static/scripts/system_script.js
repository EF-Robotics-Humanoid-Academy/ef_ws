// System monitoring data storage
const systemData = {
    cpu: { data: [], labels: [] },
    memory: { data: [], labels: [] },
    network: { data: [], labels: [] },  // Changed from 'disk' to 'network'
    temperature: { data: [], labels: [] }
};

const maxDataPoints = 20;
let charts = {};

// Chart configuration
const chartConfig = {
    type: 'line',
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            x: {
                display: false,
                grid: {
                    color: 'rgba(52, 167, 52, 0.1)'
                }
            },
            y: {
                beginAtZero: true,
                max: 100,
                grid: {
                    color: 'rgba(52, 167, 52, 0.1)'
                },
                ticks: {
                    color: '#6edf6e',
                    font: { size: 10 }
                }
            }
        },
        plugins: {
            legend: { display: false }
        },
        elements: {
            line: {
                tension: 0.4,
                borderWidth: 2
            },
            point: {
                radius: 0,
                hoverRadius: 4
            }
        },
        animation: {
            duration: 200
        }
    }
};

// Initialize charts
function initializeCharts() {
    const cpuCtx = document.getElementById('cpuChart').getContext('2d');
    const memoryCtx = document.getElementById('memoryChart').getContext('2d');
    const networkCtx = document.getElementById('networkChart').getContext('2d');  // Changed from 'diskChart' to 'networkChart'
    const temperatureCtx = document.getElementById('temperatureChart').getContext('2d');

    charts.cpu = new Chart(cpuCtx, {
        ...chartConfig,
        data: {
            labels: systemData.cpu.labels,
            datasets: [{
                data: systemData.cpu.data,
                borderColor: '#ff6b6b',
                backgroundColor: 'rgba(255, 107, 107, 0.1)',
                fill: true
            }]
        }
    });

    charts.memory = new Chart(memoryCtx, {
        ...chartConfig,
        data: {
            labels: systemData.memory.labels,
            datasets: [{
                data: systemData.memory.data,
                borderColor: '#4ecdc4',
                backgroundColor: 'rgba(78, 205, 196, 0.1)',
                fill: true
            }]
        }
    });

    // Network chart with custom scaling for MB/s
    charts.network = new Chart(networkCtx, {
        ...chartConfig,
        data: {
            labels: systemData.network.labels,
            datasets: [{
                data: systemData.network.data,
                borderColor: '#45b7d1',
                backgroundColor: 'rgba(69, 183, 209, 0.1)',
                fill: true
            }]
        },
        options: {
            ...chartConfig.options,
            scales: {
                ...chartConfig.options.scales,
                y: {
                    ...chartConfig.options.scales.y,
                    max: null,  // Let it auto-scale for network data
                    // suggestedMax: 10,
                    ticks: {
                        ...chartConfig.options.scales.y.ticks,
                        callback: function (value) {
                            return value.toFixed(1) + ' MB/s';
                        }
                    }
                }
            }
        }
    });

    charts.temperature = new Chart(temperatureCtx, {
        ...chartConfig,
        data: {
            labels: systemData.temperature.labels,
            datasets: [{
                data: systemData.temperature.data,
                borderColor: '#f39c12',
                backgroundColor: 'rgba(243, 156, 18, 0.1)',
                fill: true
            }]
        },
        options: {
            ...chartConfig.options,
            scales: {
                ...chartConfig.options.scales,
                y: {
                    ...chartConfig.options.scales.y,
                    max: 100,
                    ticks: {
                        ...chartConfig.options.scales.y.ticks,
                        callback: function (value) {
                            return value + '°C';
                        }
                    }
                }
            }
        }
    });
}

// Update chart data
function updateChartData(chartKey, value) {
    const now = new Date().toLocaleTimeString();

    systemData[chartKey].data.push(value);
    systemData[chartKey].labels.push(now);

    if (systemData[chartKey].data.length > maxDataPoints) {
        systemData[chartKey].data.shift();
        systemData[chartKey].labels.shift();
    }

    charts[chartKey].update('none');
}

// Original system monitoring functions
document.addEventListener('DOMContentLoaded', () => {
    const systemInfo = document.getElementById('systemInfo');
    const healthAlerts = document.getElementById('healthAlerts');

    // Initialize charts after DOM is loaded
    setTimeout(initializeCharts, 100);

    function fetchSystemInfo() {
        fetch('/system_info')
            .then(response => response.json())
            .then(data => {
                systemInfo.innerHTML = '';
                if (data.system_info && Array.isArray(data.system_info)) {
                    data.system_info.forEach(info => {
                        const infoItem = document.createElement('p');
                        const parts = info.split(':');
                        if (parts.length > 1) {
                            const beforeColon = document.createElement('span');
                            beforeColon.classList.add('js-text-color');

                            // Detect which metric it is and prepend icon
                            let iconHtml = '';
                            if (parts[0].includes('CPU')) iconHtml = '<i class="fas fa-microchip"></i> ';
                            if (parts[0].includes('Memory')) iconHtml = '<i class="fas fa-memory"></i> ';
                            if (parts[0].includes('Disk')) iconHtml = '<i class="fas fa-hdd"></i> ';
                            if (parts[0].includes('Network')) iconHtml = '<i class="fas fa-network-wired"></i> ';
                            if (parts[0].includes('Temperature')) iconHtml = '<i class="fas fa-thermometer-half"></i> ';
                            if (parts[0].includes('Processor')) iconHtml = '<i class="fas fa-microchip"></i> ';
                            if (parts[0].includes('Architecture')) iconHtml = '<i class="fas fa-cubes"></i> ';
                            if (parts[0].includes('Operating System')) iconHtml = '<i class="fas fa-desktop"></i> ';
                            if (parts[0].includes('System Uptime')) iconHtml = '<i class="fas fa-clock"></i> ';

                            beforeColon.innerHTML = iconHtml + parts[0].trim() + ': ';

                            const afterColon = document.createElement('span');
                            afterColon.innerText = parts.slice(1).join(':').trim();
                            afterColon.classList.add('js-text');
                            infoItem.appendChild(beforeColon);
                            infoItem.appendChild(afterColon);
                        } else {
                            infoItem.innerText = info;
                        }
                        systemInfo.appendChild(infoItem);
                    });
                } else {
                    systemInfo.innerText = 'No system info available.';
                }
            })
            .catch(error => {
                systemInfo.innerText = 'Error fetching system info.';
                console.error(error);
            });
    }

    function fetchHealthMetrics() {
        fetch('/system_info')
            .then(response => response.json())
            .then(data => {
                healthAlerts.innerHTML = '';

                let cpuUsage = 0;
                let memoryUsage = 0;
                let freeMemory = 0;
                let diskUsage = 0;
                let networkActivity = 0;  // Changed from diskUsage to networkActivity
                let systemTemperature = 0;

                data.system_info.forEach(info => {
                    if (info.includes('CPU Usage')) {
                        cpuUsage = parseFloat(info.split(':')[1].trim().replace('%', ''));
                    }
                    if (info.includes('Used Memory')) {
                        const usedMemory = parseFloat(info.split(':')[1].trim().replace(' GB', ''));
                        const totalMemoryInfo = data.system_info.find(info => info.includes('Total Memory'));
                        if (totalMemoryInfo) {
                            const totalMemory = parseFloat(totalMemoryInfo.split(':')[1].trim().replace(' GB', ''));
                            memoryUsage = (usedMemory / totalMemory) * 100;
                        }
                    }
                    if (info.includes('Free Memory')) {
                        freeMemory = parseFloat(info.split(':')[1].trim().replace(' GB', ''));
                    }
                    if (info.includes('Used Disk')) {
                        const usedDisk = parseFloat(info.split(':')[1].trim().replace(' GB', ''));
                        const totalDiskInfo = data.system_info.find(info => info.includes('Total Disk'));
                        if (totalDiskInfo) {
                            const totalDisk = parseFloat(totalDiskInfo.split(':')[1].trim().replace(' GB', ''));
                            diskUsage = (usedDisk / totalDisk) * 100;
                        }
                    }
                    // Parse network activity rate
                    if (info.includes('Network Rate')) {
                        networkActivity = parseFloat(info.split(':')[1].trim().replace(' MB/s', ''));
                    }
                    if (info.includes('Temperature')) {
                        systemTemperature = parseFloat(info.split(':')[1].trim().replace('°C', ''));
                    }
                });

                // Update charts with new data
                if (charts.cpu) updateChartData('cpu', cpuUsage);
                if (charts.memory) updateChartData('memory', memoryUsage);
                if (charts.disk) updateChartData('disk', diskUsage);
                if (charts.network) updateChartData('network', networkActivity);  // Changed from disk to network
                if (charts.temperature) updateChartData('temperature', systemTemperature);

                let hasAlerts = false;

                if (cpuUsage > 65) {
                    const cpuAlert = document.createElement('p');
                    cpuAlert.innerHTML = `<span style="color: pink;"><i class="fas fa-exclamation-triangle"></i> High CPU Usage:</span> ${cpuUsage}%`;
                    healthAlerts.appendChild(cpuAlert);
                    hasAlerts = true;
                }

                if (memoryUsage > 80) {
                    const memoryAlert = document.createElement('p');
                    memoryAlert.innerHTML = `<span style="color: pink;"><i class="fas fa-exclamation-triangle"></i> High Memory Usage:</span> ${memoryUsage.toFixed(2)}%`;
                    healthAlerts.appendChild(memoryAlert);
                    hasAlerts = true;
                }

                if (freeMemory < 3) {
                    const freeMemoryAlert = document.createElement('p');
                    freeMemoryAlert.innerHTML = `<span style="color: pink;"><i class="fas fa-exclamation-triangle"></i> Low Free Memory:</span> ${freeMemory.toFixed(2)} GB`;
                    healthAlerts.appendChild(freeMemoryAlert);
                    hasAlerts = true;
                }

                if (diskUsage > 80) {
                    const diskAlert = document.createElement('p');
                    diskAlert.innerHTML = `<span style="color: pink;"><i class="fas fa-exclamation-triangle"></i> High Disk Usage:</span> ${diskUsage.toFixed(2)}%`;
                    healthAlerts.appendChild(diskAlert);
                    hasAlerts = true;
                }

                // Network activity alert (high network usage)
                if (networkActivity > 10) {  // Alert if network activity > 10 MB/s
                    const networkAlert = document.createElement('p');
                    networkAlert.innerHTML = `<span style="color: pink;"><i class="fas fa-exclamation-triangle"></i> High Network Activity:</span> ${networkActivity.toFixed(2)} MB/s`;
                    healthAlerts.appendChild(networkAlert);
                    hasAlerts = true;
                }

                if (!hasAlerts) {
                    const noAlerts = document.createElement('p');
                    noAlerts.innerHTML = `<span><i class="fas fa-check-circle"></i> System is healthy</span>`;
                    healthAlerts.appendChild(noAlerts);
                }
            })
            .catch(error => {
                healthAlerts.innerText = 'Error fetching health metrics.';
                console.error(error);
            });
    }

    // Initial fetch
    fetchSystemInfo();
    fetchHealthMetrics();

    // Update every second
    setInterval(() => {
        fetchSystemInfo();
        fetchHealthMetrics();
    }, 1000);
});