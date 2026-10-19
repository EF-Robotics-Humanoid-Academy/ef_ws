# USB controller input stream

This service reads a generic USB game controller and streams its axes, buttons,
and hat/D-pad values over HTTP using Server-Sent Events.

## Run on Windows

```powershell
py -m pip install -r requirements.txt
py controller_stream.py
```

The service listens on `0.0.0.0:8765` by default. A client on the same network
can connect to:

- `http://LAPTOP_IP:8765/events` for live events
- `http://LAPTOP_IP:8765/snapshot` for the current state

Each event is JSON in an SSE `data:` line, for example:

```json
{"type":"state","data":{"timestamp":1710000000.0,"controllers":[{"id":0,"name":"USB Gamepad","axes":[0.0,-1.0],"buttons":[false,true],"hats":[[0,0]]}]}}
```

If Windows Firewall prompts when starting the server, allow inbound TCP on port
8765 for the network profile you intend to use. To use another port:

```powershell
py controller_stream.py --port 9000
```

The controller is enumerated at startup. If it is connected after startup,
restart the service.
