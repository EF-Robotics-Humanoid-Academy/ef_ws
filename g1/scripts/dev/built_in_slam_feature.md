# Eingebaute SLAM-Funktion (G1) - Kurzuebersicht

Diese Notiz fasst den eingebauten SLAM-Stack im Repo zusammen (`slam_operate` + DDS-Map/Odom-Themen), das aktuelle Verhalten in `ef_client` sowie den Vergleich zu externem SLAM.

---

## 1) Kern-Themen (kurz)

RPC-Service:
- `/slam_operate` (Service-Name: `slam_operate`, Version `1.0.0.1`)

Wichtige DDS-Themen fuer eingebautes SLAM und Map/Navi:
- `rt/utlidar/map_state` (HeightMap_)
- `rt/utlidar/switch` (`"ON"` / `"OFF"`)
- `rt/unitree/slam_mapping/odom`
- `rt/unitree/slam_mapping/points`
- `rt/slam_info` (JSON-Status)
- `rt/slam_key_info` (JSON-Key-Events / Task-Ergebnisse)

Verwandte Punktwolken-Themen:
- `rt/utlidar/cloud_deskewed`
- `rt/utlidar/cloud_livox_mid360`

---

## 2) `slam_operate` API (im Repo genutzt)

Aus `navigation/obstacle_avoidance/slam_service.py`:
- `1801` `start_mapping(slam_type="indoor")`
- `1802` `end_mapping(address)`
- `1804` `init_pose(...)`
- `1102` `pose_nav(targetPose, mode=1)`
- `1201` `pause_nav()`
- `1202` `resume_nav()`
- `1901` `close_slam()`

---

## 3) Abhaengigkeiten (aus `stack.txt` / `steps.txt`)

Der lokale/eingebaute SLAM-Pfad in diesem Workspace nutzt:
- Livox SDK2 Shared Library (ueber `livox2_python.py`)
- `kiss-icp` (in den Notizen kompatibel mit API von `v1.2.3`)
- Open3D (`Visualizer`) bzw. headless Varianten
- PyQt/PySide in GUI-Wrappern (`run_geoff_gui*`, `ef_client.slam_service`)
- Unitree SDK2 Python (`unitree_sdk2py`)

Laufzeit-Hinweise aus den Logdateien:
- Open3D-Fenster kann von Wayland/X11-Konfiguration abhaengen.
- OpenBLAS-Warnungen wurden in den Notizen per Umgebungsvariablen abgefangen.

---

## 4) Eingebauter SLAM-Flow mit `ef_client.Robot`

Wichtige Einstiege in `ef_client.py`:
- `start_slam(viz=False)` / `stop_slam()`
- `slam_service(...)` (PyQt-Bedienfenster)
- `set_path_point(x, y, yaw)` + `navigate_path(...)`
- `nav_pose(...)` (erst `pose_nav`, dann lokaler Fallback)
- `debug_api(...)` fuer rohe `slam_operate` Request/Response-Pruefung

`slam_service()` bietet aktuell:
- SLAM starten/stoppen
- Teleop starten (`keyboard_controller.py --input curses`)
- aktuelle Pose zur Pfadliste hinzufuegen (bevorzugt SLAM-Pose, wenn verfuegbar)
- Pfad folgen
- Umschalten obstacle-avoid Verhalten
- Lidar-Nahbereichsfilter konfigurieren

Zusatz:
- `enable_usb_controller(...)` ist im `Robot` verfuegbar und kann parallel genutzt werden.

---

## 5) Kartenerzeugung + Punktaufnahme

Zwei praktische Kartenpfade im Repo:
1. Lokales Live-Speichern (`live_slam_save.py` / headless Runner) -> `.pcd` (+ Pose-JSON)
2. DDS-basierte Konvertierung (`slam_map.py`, `create_map.py`) aus:
   - `HeightMap_` (`rt/utlidar/map_state`)
   - oder PointCloud2-Themen aus SLAM/Lidar

Pfadpunkte fuer Ausfuehrung kommen aus:
- `set_path_point(...)`
- oder GUI-Knopf "Add Current Pose" in `slam_service()`

---

## 6) Genauigkeitsgrenzen auf diesem G1 (wichtig)

Beobachtet und im Code bereits beruecksichtigt:
- Eingebautes `pose_nav` kann bei Frame-/Relokalisierungsproblemen ablehnen oder abbrechen.
- Lokale obstacle-avoid Navigation (`obs_avoid=True`) ist oft robuster als reines `pose_nav`, aber nicht perfekt exakt.
- Die aktuelle Implementierung nutzt bereits Fallback:
  - zuerst `pose_nav`
  - bei Ablehnung/Inkonsistenz Wechsel auf lokalen dynamischen Planer

Praktisch fuer reproduzierbares Verhalten:
- Waehren der Pfadausfuehrung SLAM weiterlaufen lassen.
- Eher dichtere und kuerzere Wegsegmente setzen.
- Lokalen Planer-Fallback aktiviert lassen, wenn `pose_nav` driftet oder ablehnt.

---

## 7) Warum externes SLAM oft nuetzlicher ist

Wenn hoehere Konsistenz bei Karte/Pose benoetigt wird:
- externen SLAM/State-Estimator als primaere Lokalisierung verwenden
- eingebauten Stack vor allem fuer schnelle Demo, Topic-Inspektion und Kompatibilitaet behalten

---

## 8) Externes-SLAM-Rezept (nach deiner Notiz)

High-Level-Pipeline:
1. Zugriff auf rohe 3D-Lidar-Punkte (Distanz/Winkel oder XYZ-Frames).
2. Bewegungsinformation aus Scan-zu-Scan-Geometrie (Ableitungen) gewinnen.
3. ICP (oder GICP/NDT) fuer Scan-Ausrichtung.
4. IMU fuer kurzfristig stabile Orientierung/Geschwindigkeit fusionieren.
5. Particle Filter (oder EKF/UKF) fuer robusten globalen Zustand.
6. Globale Karte + lokale Submaps pflegen.
7. Stabile Pose an die Navigationsschicht ausgeben.

Minimale Datenschnittstellen:
- Lidar-Stream: Punktwolkenframes mit Zeitstempeln
- IMU-Stream: Winkelgeschwindigkeit + Beschleunigung (+ ggf. Orientierung)
- Ausgangspose: `(x, y, yaw)` mit fixer Rate fuer Controller/Navi

---

## 9) Praktische Empfehlung

Eingebautes SLAM nutzen, wenn:
- schnelle Inbetriebnahme mit vorhandenen Repo-Skripten/GUIs im Vordergrund steht.

Externes SLAM nutzen, wenn:
- bessere Lokalisierungsstabilitaet und weniger Navigationsabweichung benoetigt werden.

Beides kombinieren:
- eingebauter Stack fuer Service-/Topic-Kompatibilitaet (`slam_operate`, DDS-Werkzeuge)
- externer Stack als primaere Lokalisierung fuer Navigation
