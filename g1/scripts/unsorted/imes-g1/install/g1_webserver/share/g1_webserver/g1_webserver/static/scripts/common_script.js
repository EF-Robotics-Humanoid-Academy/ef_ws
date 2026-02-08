/*
Software License Agreement (BSD)
@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH
*/

// --- 0) Apply saved theme ASAP (avoid FOUC)
const savedTheme = localStorage.getItem('theme') || 'light-theme';
document.documentElement.classList.add(savedTheme);

// --- 1) Utilities
function waitFor(predicateFn, { timeoutMs = 5000, intervalMs = 50 } = {}) {
    return new Promise((resolve, reject) => {
        const start = performance.now();
        const t = setInterval(() => {
            try {
                if (predicateFn()) {
                    clearInterval(t);
                    resolve(true);
                } else if (performance.now() - start > timeoutMs) {
                    clearInterval(t);
                    reject(new Error('waitFor timeout'));
                }
            } catch (e) {
                clearInterval(t);
                reject(e);
            }
        }, intervalMs);
    });
}

function loadClassicScript(src) {
    return new Promise((resolve, reject) => {
        // Avoid duplicates (loose match so cache-busted URLs still match)
        if ([...document.scripts].some(s => s.src && s.src.includes(src))) {
            resolve('already-present');
            return;
        }
        const s = document.createElement('script');
        s.src = src + (src.includes('?') ? '&' : '?') + 'v=' + Date.now(); // dev cache-bust
        s.onload = () => resolve('loaded');
        s.onerror = () => reject(new Error(`Failed to load ${src}`));
        document.head.appendChild(s);
    });
}

const loadBatteryScript = () => loadClassicScript('../static/scripts/battery_status_script.js');
const loadConnectionScript = () => loadClassicScript('../static/scripts/connection_status_script.js');

// --- 2) Main boot: navbar → scripts → init → footer/popup
(async () => {
    // 2.1 Navbar
    const nav = document.querySelector('.navbar');
    const navBarUrl = '../static/common/navbar.html';

    if (nav) {
        try {
            // GET the navbar and read Last-Modified from the same response
            const res = await fetch(navBarUrl, { cache: 'no-cache' });
            const html = await res.text();
            nav.innerHTML = html;

            // derive the date from the navbar file (updates only when file changes)
            let lm = res.headers.get('Last-Modified');

            // robust fallback: try a HEAD if header was stripped by server on GET
            if (!lm) {
                try {
                    const headRes = await fetch(navBarUrl, { method: 'HEAD', cache: 'no-cache' });
                    lm = headRes.headers.get('Last-Modified');
                } catch { /* ignore */ }
            }

            if (lm) {
                setFirmwareDateFrom(new Date(lm));        // uses navbar's timestamp
            } // else: leave whatever is in #firmwareDate unchanged

            // Theme toggle
            const root = document.documentElement;
            const toggleBtn = document.getElementById('themeToggleBtn');
            if (toggleBtn) {
                toggleBtn.addEventListener('click', (e) => {
                    e.preventDefault();
                    if (root.classList.contains('light-theme')) {
                        root.classList.replace('light-theme', 'dark-theme');
                        localStorage.setItem('theme', 'dark-theme');
                    } else {
                        root.classList.replace('dark-theme', 'light-theme');
                        localStorage.setItem('theme', 'light-theme');
                    }
                });
            }

            // 2.2 Load scripts that depend on navbar elements
            await Promise.all([
                loadBatteryScript(),
                loadConnectionScript(),
            ]);

            // 2.3 Wait for elements provided by navbar.html
            await waitFor(() =>
                document.getElementById('batteryMiniLevel') &&
                document.getElementById('batteryMiniPercentage') &&
                document.getElementById('connectionStatus') &&
                document.getElementById('connectionText')
            );

            // 2.4 Initialize idempotent modules
            if (typeof window.initBatteryStatus === 'function') {
                window.initBatteryStatus();
            } else {
                console.warn('initBatteryStatus() not found. Check battery_status_script.js path/definition.');
            }

            if (typeof window.initConnectionStatus === 'function') {
                window.initConnectionStatus();
            } else {
                console.warn('initConnectionStatus() not found. Check connection_status_script.js path/definition.');
            }

        } catch (e) {
            console.error('Navbar/Scripts init failed:', e);
        }
    } else {
        console.warn('“.navbar” container not found — navbar-dependent modules not initialized.');
    }

    // 2.5 Footer
    const footer_ = document.querySelector('.footer_');
    const footerBarUrl = '../static/common/footer.html';
    if (footer_) {
        try {
            footer_.innerHTML = await fetch(footerBarUrl).then(r => r.text());
        } catch (e) {
            console.error('Footer load failed:', e);
        }
    }

    // 2.6 Popup
    const popup_ = document.querySelector('.popup_');
    const popUpUrl = '../static/common/popup.html';
    if (popup_) {
        try {
            popup_.innerHTML = await fetch(popUpUrl).then(r => r.text());
            const popup = document.getElementById('startupPopup');
            if (popup) {
                popup.classList.remove('fade-out');
                popup.style.display = 'block';
                popup.classList.add('show');
                setTimeout(() => { closePopup(); }, 30000);
            }
        } catch (e) {
            console.error('Popup load failed:', e);
        }
    }
})();

// --- 3) Popup close helper
function closePopup() {
    const popup = document.getElementById('startupPopup');
    if (!popup) return;
    popup.classList.remove('show');
    popup.classList.add('fade-out');
    popup.addEventListener('animationend', () => {
        popup.style.display = 'none';
        popup.classList.remove('fade-out');
    }, { once: true });
}

// --- 4) Firmware date
function setFirmwareDateFrom(dateObj) {
    const el = document.getElementById('firmwareDate');
    if (!el || !(dateObj instanceof Date) || isNaN(dateObj)) return;

    const day = String(dateObj.getDate()).padStart(2, '0');
    const month = dateObj.toLocaleString('default', { month: 'short' });
    const year = String(dateObj.getFullYear());
    const hours = String(dateObj.getHours()).padStart(2, '0');
    const mins = String(dateObj.getMinutes()).padStart(2, '0');
    const seconds = String(dateObj.getSeconds()).padStart(2, '0');

    el.textContent = `FW: ${day} ${month} ${year}`;
}
