let isFirstLoad = true;

async function updateMap() {
    const mapImage = document.getElementById('mapImage');
    const mapLoader = document.getElementById('mapLoader');

    // Show loader only if it's the first load or the map failed previously
    if (isFirstLoad || mapImage.src === "") {
        mapLoader.style.display = "block";
        mapImage.style.display = "none";
    }

    try {
        const response = await fetch('/get_map', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });

        if (response.ok) {
            const blob = await response.blob();
            const url = URL.createObjectURL(blob);
            
            if (mapImage.src !== url) {
                mapImage.src = url; // Update only if changed
            }

            mapImage.style.display = "block"; // Show map
            mapLoader.style.display = "none"; // Hide loader
            isFirstLoad = false; // Mark first load as completed
        } else {
            mapImage.alt = "Map Unavailable. Please launch SLAM or Map Navigation!";
            mapImage.style.display = "block"; // Show error message
            mapLoader.style.display = "block"; // Keep loader visible
        }
    } catch (error) {
        console.error("Error fetching map:", error);
        mapImage.alt = "Error loading map.";
        mapImage.style.display = "block"; // Show error message
        mapLoader.style.display = "block"; // Keep loader visible
    }
}

// Refresh the map every second
setInterval(updateMap, 1000);
updateMap();