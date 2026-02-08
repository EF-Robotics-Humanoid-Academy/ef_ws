// connection_status_script.js
// Idempotent polling of /webserver_connection_status with graceful UI fallbacks.

(function () {
  const REQUEST_TIMEOUT_MS = 3000;
  const POLL_INTERVAL_MS   = 5000; // Check cadence
  const BASE_URL = ''; // e.g., 'http://192.168.123.164:9000' (optional)

  let initialized = false;
  let pollTimer = null;
  let inflight = false;

  // ---- UI helpers ----
  function setStatus(state) {
    if (typeof window.updateConnectionStatus === 'function') {
      // If your app provides a central updater, use it
      try { window.updateConnectionStatus(state); return; } catch {}
    }
    const statusEl = document.getElementById('connectionStatus');
    const textEl   = document.getElementById('connectionText');
    if (!statusEl || !textEl) return;

    statusEl.classList.remove('connected', 'disconnected', 'connecting');

    if (state === 'connected') {
      statusEl.classList.add('connected');
      textEl.textContent = 'Connected';
    } else if (state === 'disconnected') {
      statusEl.classList.add('disconnected');
      textEl.textContent = 'Offline';
    } else {
      statusEl.classList.add('connecting');
      textEl.textContent = 'Connecting...';
    }
  }

  function setConnectionErrorUI(msg = 'Error loading connection status') {
    const container = document.querySelector('.connection-status-container');
    if (container) container.innerHTML = `<p>${msg}</p>`;
    setStatus('disconnected');
  }

  // ---- Networking helpers ----
  async function fetchWithTimeout(path, opts = {}) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);
    try {
      const base = BASE_URL || window.location.origin;
      const u = new URL(path, base);
      u.searchParams.set('_', String(Date.now())); // cache-bust
      const resp = await fetch(u.toString(), {
        ...opts,
        cache: 'no-store',
        headers: {
          'Accept': 'application/json',
          'Pragma': 'no-cache',
          'Cache-Control': 'no-cache',
          ...(opts.headers || {}),
        },
        signal: controller.signal,
        // If API is cross-origin and needs cookies: credentials: 'include',
      });
      return resp;
    } finally {
      clearTimeout(timer);
    }
  }

  async function fetchConnectionStatus() {
    if (inflight) return;
    inflight = true;

    try {
      setStatus('connecting');

      const resp = await fetchWithTimeout('/webserver_connection_status', { method: 'GET' });
      if (!resp || !resp.ok) {
        setConnectionErrorUI(`Server responded ${resp ? resp.status : 'no response'}`);
        return;
      }

      const ct = (resp.headers.get('content-type') || '').toLowerCase();
      if (!ct.includes('application/json')) {
        setConnectionErrorUI('Unexpected response (not JSON)');
        return;
      }

      const data = await resp.json().catch(() => null);
      if (data && data.status === 'ok') {
        const container = document.querySelector('.connection-status-container');
        if (container) container.innerHTML = '';
        setStatus('connected');
      } else {
        setConnectionErrorUI('Invalid payload from server');
      }
    } catch (err) {
      console.error('Error fetching connection status:', err);
      setConnectionErrorUI('Error loading connection status');
    } finally {
      inflight = false;
    }
  }

  function startPolling() {
    if (pollTimer) clearInterval(pollTimer);
    fetchConnectionStatus(); // immediate
    pollTimer = setInterval(fetchConnectionStatus, POLL_INTERVAL_MS);
  }

  function initConnectionStatus() {
    if (initialized) return;
    initialized = true;
    console.info('[conn] Initialized');
    startPolling();
  }

  // Expose explicit initializer for loader
  window.initConnectionStatus = initConnectionStatus;

  // Optional auto-init for static-inclusion pages
  if (document.readyState !== 'loading') {
    try { initConnectionStatus(); } catch (e) { console.error(e); }
  } else {
    document.addEventListener('DOMContentLoaded', () => {
      try { initConnectionStatus(); } catch (e) { console.error(e); }
    }, { once: true });
  }
})();
