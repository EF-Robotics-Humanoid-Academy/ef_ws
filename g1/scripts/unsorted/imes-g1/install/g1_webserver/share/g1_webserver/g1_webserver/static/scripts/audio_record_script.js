let mediaRecorder;
let audioChunks = [];
let isRecording = false;
let audioContext, analyser, dataArray, animationId;
let startTime, timerInterval;

const micButton = document.getElementById('micButton');
const recordingStatus = document.getElementById('recordingStatus');
const visualizer = document.getElementById('soundVisualizer');

// Set initial state
recordingStatus.textContent = 'IDLE';
recordingStatus.classList.remove('recording');
recordingStatus.classList.add('idle');


function getTimestamp() {
    const now = new Date();
    const pad = (n) => n.toString().padStart(2, '0');
    const date = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}`;
    const time = `${pad(now.getHours())}-${pad(now.getMinutes())}-${pad(now.getSeconds())}`;
    return `${date}_${time}`;
}

function updateVisualizer() {
    analyser.getByteFrequencyData(dataArray);
    const bars = visualizer.querySelectorAll('.sound-bar');
    bars.forEach((bar, i) => {
        const value = dataArray[i] || 0;
        const height = (value / 255) * 30 + 8;
        bar.style.height = `${height}px`;
    });
    animationId = requestAnimationFrame(updateVisualizer);
}

function startTimer() {
    startTime = Date.now();
    timerInterval = setInterval(() => {
        const elapsed = Math.floor((Date.now() - startTime) / 1000);
        const minutes = Math.floor(elapsed / 60).toString().padStart(2, '0');
        const seconds = (elapsed % 60).toString().padStart(2, '0');
        recordingStatus.textContent = `RECORDING ${minutes}:${seconds}`;
        recordingStatus.classList.remove('idle');
        recordingStatus.classList.add('recording');

    }, 1000);
}

function stopTimer() {
    clearInterval(timerInterval);
}

micButton.onclick = async () => {

    try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });

        const audio = document.getElementById('buttonClickSound');
        if (audio) {
            audio.currentTime = 0;
            audio.play();
        }

        if (!isRecording) {

            const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            mediaRecorder = new MediaRecorder(stream);
            audioChunks = [];

            const source = new (window.AudioContext || window.webkitAudioContext)();
            audioContext = source;
            const sourceNode = audioContext.createMediaStreamSource(stream);
            analyser = audioContext.createAnalyser();
            analyser.fftSize = 32;
            dataArray = new Uint8Array(analyser.frequencyBinCount);
            sourceNode.connect(analyser);

            updateVisualizer();
            visualizer.classList.add('active');
            micButton.classList.add('recording');
            startTimer();

            mediaRecorder.ondataavailable = e => audioChunks.push(e.data);
            mediaRecorder.onstop = () => {
                cancelAnimationFrame(animationId);
                visualizer.classList.remove('active');
                micButton.classList.remove('recording');
                stopTimer();

                const audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
                const formData = new FormData();
                const filename = `recording_${getTimestamp()}.webm`;
                formData.append('audio', audioBlob, filename);

                fetch('/save_audio', {
                    method: 'POST',
                    body: formData
                })
                    .then(res => res.json())
                    .then(data => {
                        recordingStatus.textContent = data.message || 'SAVED';
                        console.log('Saved audio:', data);
                    })
                    .catch(err => {
                        console.error('Upload error:', err);
                        recordingStatus.textContent = 'Upload failed';
                    });
            };

            mediaRecorder.start();
            isRecording = true;
        } else {
            mediaRecorder.stop();
            audioContext.close();
            isRecording = false;
            recordingStatus.textContent = 'PROCESSING';
            recordingStatus.classList.remove('idle');
            recordingStatus.classList.add('recording');
        }
    } catch (err) {
        alert("Microphone access denied or not supported on this device.");
        console.error("Microphone error:", err);
    }
};
