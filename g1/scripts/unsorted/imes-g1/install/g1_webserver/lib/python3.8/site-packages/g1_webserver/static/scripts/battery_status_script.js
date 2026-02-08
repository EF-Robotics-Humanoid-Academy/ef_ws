// battery_status_script.js
// Idempotent, safe to call multiple times via window.initBatteryStatus()

(function () {
  let pollingTimer = null;
  let inFlight = false;
  let initialized = false;
  let warnedMissingMainEls = false;

  function fetchBatteryStatus() {
    if (inFlight) return;
    inFlight = true;

    fetch('/battery_status', { cache: 'no-store' })
      .then(response => {
        if (!response.ok) throw new Error('Network response was not ok');
        return response.json();
      })
      .then(data => {
        updateBatteryDisplay(data);
        updateMiniBattery(data.percentage, data.status);
      })
      .catch(error => {
        console.error('Error fetching battery status:', error);
        const batteryStatusContainer = document.querySelector('.battery-status-container');
        if (batteryStatusContainer) {
          batteryStatusContainer.innerHTML = '<p>Error loading battery status</p>';
        }
      })
      .finally(() => { inFlight = false; });
  }

  function updateBatteryDisplay(data) {
    const batteryLevel = document.getElementById('batteryLevel');
    const batteryPercentage = document.getElementById('batteryPercentage');
    const batteryVoltage = document.getElementById('batteryVoltage');
    const batteryStatus = document.getElementById('batteryStatus');

    // Main panel may not exist on every page; fail softly
    if (!batteryLevel || !batteryPercentage) {
      if (!warnedMissingMainEls) {
        console.warn('[battery] Main battery elements not found (ok on pages without main panel).');
        warnedMissingMainEls = true;
      }
      return;
    }

    const displayPercentage = clampPercent(data.percentage);

    // Vertical fill (main widget uses height)
    batteryLevel.style.height = `${displayPercentage}%`;
    batteryPercentage.textContent = `${displayPercentage}%`;

    // Color thresholds
    if (displayPercentage > 60) {
      batteryLevel.style.backgroundColor = '#34a734';
    } else if (displayPercentage > 20) {
      batteryLevel.style.backgroundColor = '#ffcc00';
    } else {
      batteryLevel.style.backgroundColor = '#ff3333';
    }

    // Charging waves
    if (data.status === 'Charging') {
      batteryLevel.classList.add('charging');
      if (batteryLevel.querySelectorAll('.battery-wave').length === 0) {
        for (let i = 0; i < 3; i++) {
          const wave = document.createElement('div');
          wave.className = 'battery-wave';
          batteryLevel.appendChild(wave);
        }
      }
    } else {
      batteryLevel.classList.remove('charging');
    }

    if (batteryVoltage) batteryVoltage.textContent = `${toFixedOrNA(data.voltage, 2)}V`;
    if (batteryStatus)  batteryStatus.textContent  = data.status || 'N/A';
  }

  function updateMiniBattery(percent, status) {
    const miniLevel = document.getElementById('batteryMiniLevel');
    const miniText  = document.getElementById('batteryMiniPercentage');
    const miniWrap  = document.querySelector('.battery-status-mini');

    // Navbar might be injected after we start polling; no-op until present
    if (!miniLevel || !miniText) return;

    const p = clampPercent(percent);

    // Mini widget uses vertical fill as well
    miniLevel.style.height = p + '%';

    if (p > 60) {
      miniLevel.style.backgroundColor = '#34a734';
    } else if (p > 20) {
      miniLevel.style.backgroundColor = '#ffcc00';
    } else {
      miniLevel.style.backgroundColor = '#ff3333';
    }

    miniText.textContent = `${p}%`;

    if (miniWrap) {
      miniWrap.title = `${p}%${status ? ' • ' + status : ''}`;
    }
  }

  function clampPercent(v) {
    const n = Math.round(Number(v) || 0);
    return Math.max(0, Math.min(100, n));
  }

  function toFixedOrNA(v, digits) {
    const n = Number(v);
    return Number.isFinite(n) ? n.toFixed(digits) : 'N/A';
  }

  function startPolling() {
    // Clear any previous interval (idempotent)
    if (pollingTimer) clearInterval(pollingTimer);

    // Initial fetch immediately
    fetchBatteryStatus();

    // Poll every 1s
    pollingTimer = setInterval(fetchBatteryStatus, 1000);
  }

  function initBatteryStatus() {
    if (initialized) {
      // Already running; nothing to do
      return;
    }
    initialized = true;
    console.info('[battery] Initialized');

    // Start polling regardless of current DOM; UI updates will apply when elements appear
    startPolling();
  }

  // Expose initializer for the loader
  window.initBatteryStatus = initBatteryStatus;

  // Optional auto-init if script is included statically on pages without dynamic navbar
  if (document.readyState !== 'loading') {
    // Safe: if your Option B loader also calls init, the guard prevents double init
    try { initBatteryStatus(); } catch (e) { console.error(e); }
  } else {
    // Fallback: if this file is ever loaded before DOM ready (static include case)
    document.addEventListener('DOMContentLoaded', () => {
      try { initBatteryStatus(); } catch (e) { console.error(e); }
    }, { once: true });
  }
})();
