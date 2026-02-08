// video.js
// Receive-only ROS 2 image stream over rosbridge WebSocket (no roslib dependency).
// Multi-robot & save-locations configurable at the top.
//
// ───── CONFIGURE HERE ──────────────────────────────────────────────────────────
const CONFIG = {
    // Define all robots here. Keys are your friendly IDs.
    robots: {
        g1_5049: {
            wsUrl: 'ws://192.168.123.164:9090',
            topic: '/g1_unit_5849/d435/image_raw/compressed'
        },
        // Add more robots as needed:
        // turtlebot: { wsUrl: 'ws://10.0.0.42:9090', topic: '/camera/image/compressed' },
        // g2_lab:    { wsUrl: 'ws://172.16.0.15:9090', topic: '/stereo/left/image_raw/compressed' },
    },

    // Which robot to load by default:
    defaultRobot: 'g1_5049',

    // Saving options. If File System Access API is supported and enabled,
    // the app will prompt for folders (once per session) and save directly there.
    // Otherwise it falls back to a standard in-browser download.
    save: {
        useFileSystemAccess: true,   // set to false to always use download fallback
        namePatterns: {
            snapshot: 'snapshot_{robot}_{ts}.png',
            recording: 'recording_{robot}_{ts}.webm'
        }
        // Note: Directory handles are requested lazily on first save.
    }
};
// ───────────────────────────────────────────────────────────────────────────────

(() => {
    // ------- Elements -------
    const $ = (id) => document.getElementById(id);
    const wsUrlInput = $('wsUrl');
    const topicInput = $('topicName');
    const canvas = $('videoCanvas');
    const ctx = canvas.getContext('2d', { desynchronized: true });
    const statusDot = $('statusDot');
    const statusText = $('statusText');
    const btnConnect = $('btnConnect');
    const btnDisconnect = $('btnDisconnect');
    const btnSnapshot = $('btnSnapshot');
    const btnRecord = $('btnRecord');
    const btnDebug = $('btnDebug');
    const debugBox = $('debugBox');
    const header = $('videoHeader');
    const container = document.getElementById('video-web-panel');

    // ------- Derived config / state -------
    const supportsFSA = !!(window.showDirectoryPicker) && CONFIG.save.useFileSystemAccess;

    // Current robot selection (from CONFIG)
    let robotKey = CONFIG.defaultRobot;
    let robot = CONFIG.robots[robotKey];
    if (!robot) {
        const firstKey = Object.keys(CONFIG.robots)[0];
        robotKey = firstKey || 'default';
        robot = CONFIG.robots[robotKey] || { wsUrl: '', topic: '' };
    }

    // Pre-fill inputs from CONFIG (user can still override in UI)
    wsUrlInput.value = wsUrlInput.value || robot.wsUrl || '';
    topicInput.value = topicInput.value || robot.topic || '';

    // ------- ROS / drawing state -------
    let ws = null;
    let subscribedTopic = null;
    let lastFrameTime = 0;
    let frameCount = 0;
    let fps = 0;
    let rafScheduled = false;
    let pendingDataUrl = null;
    const img = new Image();

    // Recording state
    let mediaRecorder = null;
    let recordChunks = [];
    let recording = false;

    // File System Access API state (requested lazily)
    let snapshotDirHandle = null;
    let recordingDirHandle = null;

    // --------- UI helpers ----------
    function setStatus(mode, text) {
        statusDot.classList.remove('connected', 'connecting');
        if (mode === 'connected') statusDot.classList.add('connected');
        else if (mode === 'connecting') statusDot.classList.add('connecting');
        statusText.textContent = text;
    }
    function setButtons(connected) {
        btnConnect.disabled = connected;
        btnDisconnect.disabled = !connected;
        btnSnapshot.disabled = !connected;
        btnRecord.disabled = !connected;
    }
    function toggleDebug() {
        debugBox.classList.toggle('show');
    }
    function updateDebug(extra = '') {
        const now = performance.now();
        if (now - lastFrameTime > 1000) {
            fps = frameCount;
            frameCount = 0;
            lastFrameTime = now;
        }
        const lines = [
            `Connected: ${!!ws && ws.readyState === 1}`,
            `Robot: ${robotKey}`,
            `Topic: ${subscribedTopic || '-'}`,
            `FPS (ui): ${fps}`,
            extra
        ].filter(Boolean);
        debugBox.textContent = lines.join('\n');
    }

    // --------- Collapse toggle (default collapsed) ----------
    header.addEventListener('click', () => {
        container.classList.toggle('collapsed');
    });

    // --------- Filename helpers ----------
    function ts() {
        const d = new Date();
        const pad = (n) => String(n).padStart(2, '0');
        return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
    }
    function formatName(pattern) {
        return pattern
            .replace('{robot}', robotKey)
            .replace('{ts}', ts());
    }

    // --------- Save helpers ----------
    async function ensureSnapshotDir() {
        if (!supportsFSA) return null;
        if (!snapshotDirHandle) {
            snapshotDirHandle = await window.showDirectoryPicker({ id: 'snapshots' });
        }
        return snapshotDirHandle;
    }
    async function ensureRecordingDir() {
        if (!supportsFSA) return null;
        if (!recordingDirHandle) {
            recordingDirHandle = await window.showDirectoryPicker({ id: 'recordings' });
        }
        return recordingDirHandle;
    }

    async function saveBlobToDir(dirHandle, fileName, blob) {
        if (!dirHandle) throw new Error('No directory handle');
        const fileHandle = await dirHandle.getFileHandle(fileName, { create: true });
        const writable = await fileHandle.createWritable();
        await writable.write(blob);
        await writable.close();
    }

    function downloadFallback(blob, fileName) {
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = fileName;
        document.body.appendChild(a);
        a.click();
        a.remove();
        URL.revokeObjectURL(url);
    }

    // --------- Snapshot ----------
    btnSnapshot.addEventListener('click', async () => {
        try {
            // toBlob is safer for large frames than toDataURL
            canvas.toBlob(async (blob) => {
                if (!blob) {
                    setStatus('connected', 'Snapshot failed: empty frame.');
                    return;
                }
                const fileName = formatName(CONFIG.save.namePatterns.snapshot);
                if (supportsFSA) {
                    try {
                        const dir = await ensureSnapshotDir();
                        await saveBlobToDir(dir, fileName, blob);
                        setStatus('connected', `Snapshot saved: ${fileName}`);
                    } catch (e) {
                        console.warn('FSA snapshot save failed, falling back:', e);
                        downloadFallback(blob, fileName);
                        setStatus('connected', `Snapshot downloaded: ${fileName}`);
                    }
                } else {
                    downloadFallback(blob, fileName);
                    setStatus('connected', `Snapshot downloaded: ${fileName}`);
                }
            }, 'image/png');
        } catch (e) {
            console.warn(e);
            setStatus('connected', 'Snapshot failed (cross-origin or canvas tainted).');
        }
    });

    // --------- Recording (WebM) ----------
    btnRecord.addEventListener('click', async () => {
        if (!recording) {
            const stream = canvas.captureStream(30);
            const mime = MediaRecorder.isTypeSupported('video/webm;codecs=vp9')
                ? 'video/webm;codecs=vp9'
                : 'video/webm';
            mediaRecorder = new MediaRecorder(stream, { mimeType: mime, videoBitsPerSecond: 4_000_000 });
            recordChunks = [];
            mediaRecorder.ondataavailable = (e) => { if (e.data && e.data.size) recordChunks.push(e.data); };
            mediaRecorder.onstop = async () => {
                try {
                    const blob = new Blob(recordChunks, { type: mediaRecorder.mimeType });
                    const fileName = formatName(CONFIG.save.namePatterns.recording);
                    if (supportsFSA) {
                        try {
                            const dir = await ensureRecordingDir();
                            await saveBlobToDir(dir, fileName, blob);
                            setStatus('connected', `Recording saved: ${fileName}`);
                        } catch (e) {
                            console.warn('FSA recording save failed, falling back:', e);
                            downloadFallback(blob, fileName);
                            setStatus('connected', `Recording downloaded: ${fileName}`);
                        }
                    } else {
                        downloadFallback(blob, fileName);
                        setStatus('connected', `Recording downloaded: ${fileName}`);
                    }
                } catch (e) {
                    console.warn(e);
                    setStatus('connected', 'Recording save failed.');
                }
            };
            mediaRecorder.start();
            recording = true;
            btnRecord.classList.add('active');
            btnRecord.textContent = 'Stop Record';
        } else {
            try {
                mediaRecorder.stop();
            } catch { }
            recording = false;
            btnRecord.classList.remove('active');
            btnRecord.textContent = 'Start Record';
        }
    });

    // --------- Drawing logic (drop-old-frames) ----------
    img.onload = () => {
        // Resize canvas to source frame on first draw or when dimensions change
        if (canvas.width !== img.naturalWidth || canvas.height !== img.naturalHeight) {
            canvas.width = img.naturalWidth;
            canvas.height = img.naturalHeight;
        }
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        frameCount++;
        updateDebug();
    };
    function drawLatest() {
        rafScheduled = false;
        if (pendingDataUrl) img.src = pendingDataUrl;
    }

    // --------- rosbridge protocol (no roslib) ----------
    function subscribe(topic) {
        if (!ws || ws.readyState !== 1) return;
        // Unsubscribe previous
        if (subscribedTopic) {
            ws.send(JSON.stringify({ op: 'unsubscribe', topic: subscribedTopic }));
            subscribedTopic = null;
        }
        // Subscribe compressed image
        const msg = {
            op: 'subscribe',
            topic,
            type: 'sensor_msgs/msg/CompressedImage',
            throttle_rate: 0,
            queue_length: 1,
            compression: 'none'
        };
        ws.send(JSON.stringify(msg));
        subscribedTopic = topic;
        updateDebug('Subscribed');
    }

    function connect() {
        // If the operator typed a different IP/topic in the UI, honor it.
        const url = wsUrlInput.value.trim() || robot.wsUrl || '';
        const topic = topicInput.value.trim() || robot.topic || '';

        if (ws) disconnect(); // clean slate
        if (!url) {
            setStatus('idle', 'No WebSocket URL');
            return;
        }
        if (!topic) {
            setStatus('idle', 'No topic');
            return;
        }

        setStatus('connecting', 'Connecting…');
        setButtons(false);

        ws = new WebSocket(url);
        ws.binaryType = 'arraybuffer'; // we still expect JSON, but set safe default

        ws.onopen = () => {
            setStatus('connected', 'Connected');
            setButtons(true);
            subscribe(topic);
        };

        ws.onmessage = (event) => {
            // rosbridge messages are JSON
            let data;
            try {
                if (typeof event.data === 'string' && event.data[0] !== '{') return;
                data = JSON.parse(event.data);
            } catch {
                return;
            }
            if (data.op === 'publish' && data.topic === subscribedTopic && data.msg) {
                // sensor_msgs/msg/CompressedImage: data.msg.data is base64
                const b64 = data.msg.data;
                if (!b64) return;
                // Assume JPEG (most common); browser handles PNG if provided
                pendingDataUrl = 'data:image/jpeg;base64,' + b64;
                if (!rafScheduled) {
                    rafScheduled = true;
                    requestAnimationFrame(drawLatest);
                }
            } else if (data.op === 'status') {
                updateDebug(`Bridge status: ${data.level} ${data.msg || ''}`);
            }
        };

        ws.onerror = (e) => {
            console.warn('WS error', e);
            setStatus('connecting', 'Error—retry or check URL');
        };

        ws.onclose = () => {
            setStatus('idle', 'Disconnected');
            setButtons(false);
            subscribedTopic = null;
        };
    }

    function disconnect() {
        if (ws) {
            try {
                if (subscribedTopic) ws.send(JSON.stringify({ op: 'unsubscribe', topic: subscribedTopic }));
            } catch { }
            try { ws.close(); } catch { }
            ws = null;
        }
        setStatus('idle', 'Disconnected');
        setButtons(false);
    }

    // --------- Wire up buttons ----------
    btnConnect.addEventListener('click', connect);
    btnDisconnect.addEventListener('click', disconnect);
    btnDebug.addEventListener('click', toggleDebug);

    // --------- Convenience: Enter key focuses next / connects ----------
    [wsUrlInput, topicInput].forEach((el, idx, arr) => {
        el.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                if (idx < arr.length - 1) arr[idx + 1].focus();
                else connect();
            }
        });
    });

    // --------- Initial status ----------
    setStatus('idle', 'Collapsed • Ready');
    setButtons(false);
})();
