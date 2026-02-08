#!/usr/bin/env bash

# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

# Variables
SERVER_IP="192.168.123.164"
SSL_DIR="/etc/nginx/ssl"
NGINX_CONF="/etc/nginx/sites-available/g1_webserver"
PROXY_PASS_URL="http://127.0.0.1:9000"

# Define color codes
ORANGE='\033[38;5;208m'
YELLOW='\033[93m'
WHITE='\033[0m'
NC='\033[0m' 

log() { echo -e "${1}${2}${NC}"; }

# Update package lists and install necessary packages
log "${ORANGE}" "Installing VNC server and XFCE desktop environment..."
sudo apt update -y && sudo apt install -y tigervnc-standalone-server xfce4 xfce4-goodies dbus-x11

# Create the ~/.vnc directory if it doesn't exist
mkdir -p ~/.vnc
echo "mybotshop" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Setup the VNC server configuration
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XKL_XMODMAP_DISABLE=1

# Start basic X services
xsetroot -solid grey
vncconfig -iconic &

# Start XFCE (keep running in background)
startxfce4 &

# Requires Superuser privileges to run
# sudo chmod +x .vnc/xstartup
# Wait a bit for XFCE to load before setting the wallpaper
source /opt/mybotshop/install/setup.bash
(sleep 5 && \
 WALLPAPER_PATH="$(ros2 pkg prefix g1_webserver)/share/g1_webserver/g1_webserver/static/media/wallpaper/vnc_wallpaper.jpg" && \
 for i in {0..3}; do
   xfconf-query -c xfce4-desktop \
     -p /backdrop/screen0/monitorVNC-0/workspace$i/last-image \
     -n -t string -s "$WALLPAPER_PATH"
   xfconf-query -c xfce4-desktop \
     -p /backdrop/screen0/monitorVNC-0/workspace$i/image-style \
     -n -t int -s 3
 done && xfdesktop --reload) &

# Keep the session alive
while true; do
    sleep 3600
done
EOF

# Make the xstartup script executable
sudo chmod +x ~/.vnc/xstartup

log "${ORANGE}" "Installing Websocket..."
sudo apt install -y websockify

log "${ORANGE}" "Installing Webserver dependencies..."
python3 -m pip install playsound Flask waitress 

log "${ORANGE}" "Installing LLM dependencies..."
sudo apt install ffmpeg
sudo apt-get install python3-pyaudio 
python3 -m pip install edge-tts pydub SpeechRecognition

log "${ORANGE}" "Updating ufw policies..."
sudo ufw allow 5901/tcp && sudo ufw allow 6080/tcp && sudo ufw allow 9000

# Final instructions
log "${ORANGE}" "VNC server setup complete."
log "${YELLOW}" "To start the VNC server:"
log "${WHITE}" "vncserver :1 -geometry 1920x1080 -depth 24 -localhost no"

log "${YELLOW}" "To check VNC server status:"
log "${WHITE}" "vncserver -list"

log "${YELLOW}" "To stop the VNC server:"
log "${WHITE}" "vncserver -kill :1 && rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1"

log "${ORANGE}" "VNC Client Setup"
log "${YELLOW}" "For first-time setup, run:"
log "${WHITE}" "sudo apt install tigervnc-viewer && sudo ufw allow 5901/tcp"

log "${YELLOW}" "To connect to the VNC server:"
log "${WHITE}" "vncviewer $SERVER_IP:1"

log "${ORANGE}" "ROS2 Bridge installation"
sudo apt install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-image-transport*

# Security Error in Unitree Robots/ Nvidia Platforms

# Check without password: 
#     vncserver :1 -geometry 1280x800 -localhost no -SecurityTypes None --I-KNOW-THIS-IS-INSECURE

# Reinsert password: 
#     vncpasswd ~/.vnc/passwd
#     Ensure view only is enabled with same password


# Add Reverse Proxy

# log "${ORANGE}" "Installing Nginx server..."
# sudo apt install nginx

# log "${YELLOW}" "Creating SSL directory and self-signed certificate..."
# sudo mkdir -p "$SSL_DIR"

# sudo openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
#   -keyout "$SSL_DIR/key.pem" \
#   -out "$SSL_DIR/cert.pem" \
#   -subj "/C=DE/ST=North Rhein Westphalia/L=Bergheim/O=MyOrganization/CN=$SERVER_IP"

# log "${YELLOW}" "Writing nginx configuration..."

# log "${YELLOW}" "Writing nginx configuration..."

# sudo tee "$NGINX_CONF" > /dev/null <<EOF
# server {
#     listen 80;
#     server_name $SERVER_IP;

#     return 301 https://\$host\$request_uri;
# }

# server {
#     listen 443 ssl;
#     server_name $SERVER_IP;

#     ssl_certificate $SSL_DIR/cert.pem;
#     ssl_certificate_key $SSL_DIR/key.pem;

#     ssl_protocols TLSv1.2 TLSv1.3;
#     ssl_ciphers HIGH:!aNULL:!MD5;

#     location / {
#         proxy_pass $PROXY_PASS_URL;
#         proxy_http_version 1.1;
#         proxy_set_header Host \$host;
#         proxy_set_header X-Real-IP \$remote_addr;
#         proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
#         proxy_set_header X-Forwarded-Proto \$scheme;
#         proxy_set_header Upgrade \$http_upgrade;
#         proxy_set_header Connection "upgrade";
#     }
# }
# EOF

# log "${YELLOW}" "Enabling site, Testing, and Reloading..."
# if [ ! -f "/etc/nginx/sites-enabled/g1_webserver" ]; then
#     sudo ln -s "$NGINX_CONF" /etc/nginx/sites-enabled/
# else
#     log "${ORANGE}" "Symbolic link already exists. Skipping."
# fi
# sudo nginx -t
# sudo systemctl reload nginx

log "${YELLOW}" "Setup complete. Access your site at https://$SERVER_IP"

