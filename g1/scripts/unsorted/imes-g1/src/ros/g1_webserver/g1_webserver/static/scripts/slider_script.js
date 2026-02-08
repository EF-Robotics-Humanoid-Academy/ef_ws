// Function to update slider values and send data to the backend
function sendSliderData() {
    const linearVelocity = document.getElementById('slider1').value;
    const angularVelocity = document.getElementById('slider2').value;

    // Update the displayed values with units
    document.getElementById('slider1-value').textContent = `${linearVelocity / 100} m/s`;
    document.getElementById('slider2-value').textContent = `${angularVelocity / 100} rad/s`;

    // Send the data to the Python backend
    fetch('/slider_control', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            linear_velocity: linearVelocity,
            angular_velocity: angularVelocity,
        }),
    })
        .then(response => response.json())
        .then(data => {
            console.log('Slider Data Sent Successfully:', data);
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

function sendCustomSliderData() {
    const first = document.getElementById('VSlider1').value;
    const second = document.getElementById('VSlider2').value;
    const third = document.getElementById('VSlider3').value;

    document.getElementById('VSlider1-value').textContent = `${first}mm`;
    document.getElementById('VSlider2-value').textContent = `${second}mm`;
    document.getElementById('VSlider3-value').textContent = `${third}mm`;

    fetch('/rig_control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            rig_base: first,
            rig_left: second,
            rig_right: third,
        }),
    })
        .then(response => response.json())
        .then(data => console.log('Rig Slider Data Sent:', data))
        .catch(error => console.error('Error:', error));
}

// Add event listeners
document.getElementById('VSlider1').addEventListener('input', sendCustomSliderData);
document.getElementById('VSlider2').addEventListener('input', sendCustomSliderData);
document.getElementById('VSlider3').addEventListener('input', sendCustomSliderData);

// Twistmux sliders
document.getElementById('slider1').addEventListener('input', sendSliderData);
document.getElementById('slider2').addEventListener('input', sendSliderData);