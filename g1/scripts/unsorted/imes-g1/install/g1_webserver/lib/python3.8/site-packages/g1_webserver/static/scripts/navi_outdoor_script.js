/*
Software License Agreement (BSD)

@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
*/

// Default coordinates (NRW, Germany)
var defaultLat = 50.95358;
var defaultLon = 6.60168;
var defaultZoom = 16;
var coordinatesList = [];
var markers = [];
var lines = [];
var robotMarker;
var map;
var robotIcon;
var locationIcon;
var polylineDecoratorReady = false;
var arrowAnimationInterval;
var arrowPattern;

// Waypoint tool state
var waypointToolActive = false;

// No-go zones
var noGoZones = [];
var noGoPolygons = [];
var currentNoGoPoints = [];
var isCreatingNoGoZone = false;
var noGoTempMarkers = [];
var noGoTempLines = [];

let tileLayers;
let currentTileLayer;

// Ensure map container has valid size
var mapContainer = document.getElementById("map");
if (!mapContainer) {
    console.error("Map container is missing");
}

function switchMapType(mapType) {
    if (currentTileLayer) {
        map.removeLayer(currentTileLayer);
    }

    if (tileLayers[mapType]) {
        currentTileLayer = tileLayers[mapType];
        currentTileLayer.addTo(map);
    }
}

function toggleMapSidebar() {
    const sidebar = document.getElementById('mapSidebar');
    sidebar.classList.toggle('collapsed');
}

// Toggle waypoint tool activation
function toggleWaypointTool() {
    waypointToolActive = !waypointToolActive;
    const button = document.getElementById('waypointToolToggle');
    if (waypointToolActive) {
        button.textContent = 'Deactivate Tool';
        button.classList.add('deactivate-btn');
        updateResponseMessage('Waypoint tool activated - Click on map to add waypoints');
    } else {
        button.textContent = 'Activate Tool';
        button.classList.remove('deactivate-btn');
        updateResponseMessage('Waypoint tool deactivated');
    }
}

// Toggle no-go zone creation
function toggleNoGoZoneCreation() {
    isCreatingNoGoZone = !isCreatingNoGoZone;
    const button = document.getElementById('noGoZoneToggle');

    if (isCreatingNoGoZone) {
        button.textContent = 'Finish No-Go Zone';
        button.classList.add('deactivate-btn');
        currentNoGoPoints = [];
        updateResponseMessage('No-go zone creation active - Click on map to add polygon points');
    } else {
        button.textContent = 'Create No Go Zone';
        button.classList.remove('deactivate-btn');

        // Finish creating the current no-go zone
        if (currentNoGoPoints.length >= 3) {
            createNoGoZone();
        }
        updateResponseMessage('No-go zone creation finished');
    }
}

// Create no-go zone from current points
function createNoGoZone() {
    if (currentNoGoPoints.length < 3) {
        alert('A no-go zone needs at least 3 points to form a polygon');
        return;
    }

    // Create polygon
    const polygon = L.polygon(currentNoGoPoints, {
        color: 'red',
        fillColor: 'red',
        fillOpacity: 0.3,
        weight: 2
    }).addTo(map);

    // Add to no-go zones array
    noGoZones.push({
        id: Date.now(),
        coordinates: currentNoGoPoints.map(point => ({ lat: point.lat, lng: point.lng }))
    });

    noGoPolygons.push(polygon);

    // Clear current points
    currentNoGoPoints = [];

    updateResponseMessage(`No-go zone created with ${currentNoGoPoints.length} points`);
}

// Save no-go zones to backend
function saveNoGoZones() {
    if (noGoZones.length === 0) {
        alert('No no-go zones to save');
        return;
    }

    fetch('/save_no_go_zones', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(noGoZones)
    })
        .then(response => response.text())
        .then(msg => {
            alert("No-go zones saved successfully!");
            updateResponseMessage('No-go zones saved successfully');
        })
        .catch(err => {
            console.error("Failed to save no-go zones", err);
            updateResponseMessage('Failed to save no-go zones');
        });
}

// Load no-go zones from backend
function loadNoGoZones() {
    fetch('/load_no_go_zones')
        .then(response => response.json())
        .then(zones => {
            clearNoGoZones();
            zones.forEach(zone => {
                const coords = zone.coordinates.map(coord => L.latLng(coord.lat, coord.lng));
                const polygon = L.polygon(coords, {
                    color: 'red',
                    fillColor: 'red',
                    fillOpacity: 0.3,
                    weight: 2
                }).addTo(map);

                noGoZones.push(zone);
                noGoPolygons.push(polygon);
            });
            updateResponseMessage(`Loaded ${zones.length} no-go zones`);
        })
        .catch(err => {
            console.error("Failed to load no-go zones", err);
            updateResponseMessage('Failed to load no-go zones');
        });
}

// Clear all no-go zones
function clearNoGoZones() {
    noGoPolygons.forEach(polygon => map.removeLayer(polygon));
    noGoTempMarkers.forEach(marker => map.removeLayer(marker));
    noGoTempLines.forEach(line => map.removeLayer(line));

    noGoZones = [];
    noGoPolygons = [];
    currentNoGoPoints = [];
    noGoTempMarkers = [];
    noGoTempLines = [];

    updateResponseMessage('No-go zones cleared');
}

// Update response message
function updateResponseMessage(message) {
    const responseElement = document.getElementById('responseMessage');
    if (responseElement) {
        responseElement.textContent = message;
    }
}

// Add point to current no-go zone
function addNoGoZonePoint(latlng) {
    currentNoGoPoints.push(latlng);

    // Create a temporary marker for the point
    const marker = L.marker(latlng, {
        icon: L.icon({
            iconUrl: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjAiIGhlaWdodD0iMjAiIHZpZXdCb3g9IjAgMCAyMCAyMCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGNpcmNsZSBjeD0iMTAiIGN5PSIxMCIgcj0iOCIgZmlsbD0icmVkIiBzdHJva2U9IndoaXRlIiBzdHJva2Utd2lkdGg9IjIiLz4KPC9zdmc+',
            iconSize: [12, 12],
            iconAnchor: [6, 6]
        })
    }).addTo(map);

    noGoTempMarkers.push(marker);

    // Draw temporary lines showing the polygon being created
    if (currentNoGoPoints.length > 1) {
        const tempLine = L.polyline(currentNoGoPoints, {
            color: 'red',
            weight: 2,
            dashArray: '5, 5'
        }).addTo(map);
        noGoTempLines.push(tempLine);
    }

    updateResponseMessage(`Added point ${currentNoGoPoints.length} to no-go zone`);
}

function updateRobotPosition(lat, lon) {
    if (!map) {
        console.error("Map is not initialized yet.");
        return;
    }

    if (!isNaN(lat) && !isNaN(lon)) {
        if (robotMarker) {
            robotMarker.setLatLng([lat, lon]);
        } else {
            robotMarker = L.marker([lat, lon], { icon: robotIcon }).addTo(map);
        }
        // Set map view to robot position
        // map.setView([lat, lon]);
    } else {
        console.error("Invalid position data received", lat, lon);
    }
}

function fetchRobotPosition() {
    if (!map) {
        console.error("Map is not initialized yet.");
        return;
    }

    fetch('/update_gps_position', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ lat: null, lon: null }) // Request for latest position
    })
        .then(response => response.json())
        .then(data => {
            const { lat, lon } = data;
            console.log(`Fetched Position: lat: ${lat}, lon: ${lon}`);
            if (lat && lon) {
                updateRobotPosition(lat, lon);
                setTimeout(() => {
                    map.invalidateSize();
                }, 100);
            } else {
                console.error('Invalid data: Missing lat or lon');
            }
        })
        .catch(error => console.error("Error fetching robot position:", error));
}

// Add a waypoint marker (only when tool is active)
function addWaypoint(latlng) {
    if (!waypointToolActive) return;

    var marker = L.marker(latlng, {
        icon: locationIcon,
        draggable: false
    }).addTo(map);

    markers.push(marker);
    updateCoordinatesList();
    updateResponseMessage(`Added waypoint ${markers.length}`);
}

// Save waypoints (send to backend)
function saveWaypoints() {
    if (coordinatesList.length === 0) {
        alert('No waypoints to save');
        return;
    }

    const coords = coordinatesList.map(p => ({ lat: p.lat, lon: p.lng }));
    fetch('/save_waypoints', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(coords)
    })
        .then(response => response.text())
        .then(msg => {
            alert("Waypoints saved successfully!");
            updateResponseMessage('Waypoints saved successfully');
        })
        .catch(err => {
            console.error("Failed to save waypoints", err);
            updateResponseMessage('Failed to save waypoints');
        });
}

// Load waypoints (request from backend)
function loadWaypoints() {
    fetch('/load_waypoints')
        .then(response => response.json())
        .then(waypoints => {
            clearWaypoints();
            waypoints.forEach(coord => {
                const marker = L.marker([coord.lat, coord.lon], {
                    icon: locationIcon,
                    draggable: false
                }).addTo(map);
                markers.push(marker);
            });
            updateCoordinatesList();
            updateResponseMessage(`Loaded ${waypoints.length} waypoints`);
        })
        .catch(err => {
            console.error("Failed to load waypoints", err);
            updateResponseMessage('Failed to load waypoints');
        });
}

// Maintain coordinates list and draw lines
function updateCoordinatesList() {
    coordinatesList = markers.map(marker => marker.getLatLng());
    drawLine();
}

// Draw lines between waypoints
function drawLine() {
    if (!map) {
        console.error("Map is not initialized yet.");
        return;
    }

    // Clear previous lines and animation
    lines.forEach(item => map.removeLayer(item));
    lines = [];

    if (arrowAnimationInterval) {
        clearInterval(arrowAnimationInterval);
        arrowAnimationInterval = null;
    }

    if (coordinatesList.length > 1) {
        const polyline = L.polyline(coordinatesList, {
            color: 'var(--project-color)',
            weight: 5,
            opacity: 0.3
        }).addTo(map);
        lines.push(polyline);

        if (polylineDecoratorReady && typeof L.Symbol !== 'undefined' && typeof L.Symbol.arrowHead === 'function') {
            // Fixed-pixel animation
            let offsetPx = 0;
            const repeatPx = 30; // Spacing between arrows in pixels

            const decorator = L.polylineDecorator(polyline, {
                patterns: [
                    {
                        offset: offsetPx + 'px',
                        repeat: repeatPx + 'px',
                        symbol: L.Symbol.arrowHead({
                            pixelSize: 10,
                            polygon: false,
                            pathOptions: { stroke: true, color: 'red', weight: 2 }
                        })
                    }
                ]
            }).addTo(map);

            lines.push(decorator);

            // Animate by shifting the offset in pixels
            arrowAnimationInterval = setInterval(() => {
                offsetPx = (offsetPx + 1) % repeatPx;
                decorator.setPatterns([
                    {
                        offset: offsetPx + 'px',
                        repeat: repeatPx + 'px',
                        symbol: L.Symbol.arrowHead({
                            pixelSize: 10,
                            polygon: false,
                            pathOptions: { stroke: true, color: 'var(--project-color)', weight: 2 }
                        })
                    }
                ]);
            }, 60); // Controls animation speed
        } else {
            console.warn("Arrow heads skipped: plugin not ready.");
        }
    }
}

// Clear waypoints
function clearWaypoints() {
    markers.forEach(marker => map.removeLayer(marker));
    lines.forEach(line => map.removeLayer(line));
    if (arrowAnimationInterval) {
        clearInterval(arrowAnimationInterval);
        arrowAnimationInterval = null;
    }
    markers = [];
    lines = [];
    coordinatesList = [];
    updateResponseMessage('Waypoints cleared');
}

// Center map to robot position
function centerToRobot() {
    if (!map) {
        console.error("Map is not initialized yet.");
        updateResponseMessage('Map not initialized');
        return;
    }

    if (robotMarker) {
        const robotPos = robotMarker.getLatLng();
        map.setView(robotPos, map.getZoom());
        updateResponseMessage('Centered to robot position');
    } else {
        updateResponseMessage('Robot position not available');
    }
}

document.addEventListener("DOMContentLoaded", function () {

    function loadOpenStreetMap() {
        console.log("Loading OpenStreetMap...");

        // Load Leaflet CSS
        var leafletCSS = document.createElement("link");
        leafletCSS.rel = "stylesheet";
        leafletCSS.href = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.css";
        document.head.appendChild(leafletCSS);

        // Load Leaflet JS
        var leafletJS = document.createElement("script");
        leafletJS.src = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.js";

        leafletJS.onload = function () {
            console.log("Leaflet loaded");

            // Load PolylineDecorator
            var decoratorScript = document.createElement("script");
            decoratorScript.src = "https://cdnjs.cloudflare.com/ajax/libs/leaflet-polylinedecorator/1.1.0/leaflet.polylineDecorator.min.js";

            decoratorScript.onload = function () {
                console.log("PolylineDecorator loaded");
                polylineDecoratorReady = true;
                initMap();
            };

            decoratorScript.onerror = function () {
                console.error("Failed to load PolylineDecorator");
                initMap();  // fallback: initialize map without arrows
            };

            document.body.appendChild(decoratorScript);
        };

        leafletJS.onerror = function () {
            console.error("Failed to load Leaflet");
            loadFallbackMap();
        };

        document.body.appendChild(leafletJS);
    }

    function loadFallbackMap() {
        console.warn("No internet! Loading fallback.");
        mapContainer.innerHTML = "<p>Offline mode: No map available.</p>";
    }

    function initMap() {
        console.log("OpenStreetMap loaded successfully.");
        map = L.map('map').setView([defaultLat, defaultLon], defaultZoom);

        // Tile layers
        tileLayers = {
            'satellite': L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
                attribution: 'Tiles &copy; Esri',
                maxZoom: 22
            }),
            'street': L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
                maxZoom: 22
            }),
            'dark': L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
                attribution: '&copy; <a href="https://carto.com/">CARTO</a>',
                subdomains: 'abcd',
                maxZoom: 22
            })
        };

        // Set default tile layer
        currentTileLayer = tileLayers['street'];
        currentTileLayer.addTo(map);

        // Show GPS coordinates on hover
        const coordsDisplay = L.control({ position: 'bottomleft' });

        coordsDisplay.onAdd = function (map) {
            this._div = L.DomUtil.create('div', 'coords-display');
            this.update();
            return this._div;
        };

        coordsDisplay.update = function (latlng) {
            this._div.innerHTML = latlng
                ? `Lat: ${latlng.lat.toFixed(5)}<br>Lon: ${latlng.lng.toFixed(5)}`
                : 'Hover over map';
        };

        coordsDisplay.addTo(map);

        const centerControl = L.control({ position: 'topleft' });

        centerControl.onAdd = function (map) {
            const div = L.DomUtil.create('div', 'leaflet-control-center-robot');
            div.innerHTML = '<button onclick="centerToRobot()" title="Center to Robot"><i class="fas fa-crosshairs"></i></button>';

            // Prevent map clicks when clicking the button
            L.DomEvent.disableClickPropagation(div);

            return div;
        };

        centerControl.addTo(map);

        map.on('mousemove', function (e) {
            coordsDisplay.update(e.latlng);
        });

        // Define icons after Leaflet is loaded
        robotIcon = L.icon({
            iconUrl: '/static/media/gps/robot.webp',
            iconSize: [42, 42],
            iconAnchor: [16, 32],
            popupAnchor: [0, -32]
        });

        locationIcon = L.icon({
            iconUrl: '/static/media/gps/location.png',
            iconSize: [20, 20],
            iconAnchor: [10, 20],
            popupAnchor: [0, -20]
        });

        // Handle map clicks for waypoints and no-go zones
        map.on('click', function (e) {
            if (waypointToolActive) {
                addWaypoint(e.latlng);
            } else if (isCreatingNoGoZone) {
                addNoGoZonePoint(e.latlng);
            }
        });

        // Prevent clicks on UI from adding waypoints
        L.DomEvent.disableClickPropagation(document.querySelector('.map-sidebar'));

        // Start fetching robot position
        setInterval(fetchRobotPosition, 1000);
    }

    if (navigator.onLine) {
        loadOpenStreetMap();
    } else {
        loadFallbackMap();
    }

    window.addEventListener("online", () => location.reload());
    window.addEventListener("offline", loadFallbackMap);
});