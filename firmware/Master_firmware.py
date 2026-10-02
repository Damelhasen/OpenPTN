#!/usr/bin/env python3

import argparse
import csv
import http.server
import ipaddress
import json
import math
import socket
import time
import threading
from pymavlink import mavutil
from datetime import datetime, timezone
from pathlib import Path
import webbrowser

try:
    from rtlsdr import RtlSdr
except ImportError:
    RtlSdr = None
import numpy as np

CSV_FIELDS = (
    "timestamp_utc",
    "frequency_hz",
    "gps_time_usec",
    "fix_type",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "relative_altitude_m",
    "position_time_boot_ms",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "attitude_time_boot_ms",
    "attitude_timestamp_utc",
    "eph_m",
    "epv_m",
    "ground_speed_m_s",
    "course_deg",
    "satellites_visible",
    "signal_power_dbfs",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scan radio signal strength along a GPS track."
    )
    parser.add_argument("--frequency-mhz", type=float)
    parser.add_argument("--output")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sample-count", type=int, default=256 * 1024)
    parser.add_argument("--gps-rate-hz", type=float, default=5.0)
    parser.add_argument("--dashboard-port", type=int, default=8765)
    args = parser.parse_args()
    if args.frequency_mhz is not None and args.frequency_mhz <= 0:
        parser.error("--frequency-mhz must be greater than zero")
    if args.sample_count <= 0 or args.gps_rate_hz <= 0 or args.baud <= 0:
        parser.error("sample count, GPS rate, and baud must be greater than zero")
    if not 1 <= args.dashboard_port <= 65535:
        parser.error("dashboard port must be between 1 and 65535")
    return args


def ask_positive_number(prompt: str) -> float:
    while True:
        try:
            value = float(input(prompt))
            if value > 0:
                return value
        except ValueError:
            pass
        print("Enter a number greater than zero.")


def collect_scan_settings(args):
    frequency_mhz = args.frequency_mhz
    if frequency_mhz is None:
        frequency_mhz = ask_positive_number("Target frequency (MHz): ")

    now = datetime.now().astimezone()
    output_path = Path(
        args.output or "dronescan_{}.csv".format(now.strftime("%Y%m%d_%H%M%S"))
    ).expanduser()
    return frequency_mhz, output_path


def listen_freq(sdr_device, frequency_hz: float, sample_count: int) -> float:
    sdr_device.center_freq = frequency_hz
    samples = sdr_device.read_samples(sample_count)
    mean_power = float(np.mean(np.abs(samples) ** 2))
    return 10 * math.log10(max(mean_power, 1e-12))


def gps_row(
    message,
    frequency_hz: float,
    signal_power_dbfs: float,
    attitude,
    global_position,
) -> dict:
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "frequency_hz": frequency_hz,
        "gps_time_usec": message.time_usec,
        "fix_type": message.fix_type,
        "latitude_deg": message.lat / 1e7 if abs(message.lat) <= 900000000 else "",
        "longitude_deg": message.lon / 1e7 if abs(message.lon) <= 1800000000 else "",
        "altitude_m": message.alt / 1000,
        "relative_altitude_m": (
            global_position["relative_altitude_m"] if global_position else ""
        ),
        "position_time_boot_ms": global_position["time_boot_ms"] if global_position else "",
        "roll_deg": attitude["roll_deg"] if attitude else "",
        "pitch_deg": attitude["pitch_deg"] if attitude else "",
        "yaw_deg": attitude["yaw_deg"] if attitude else "",
        "attitude_time_boot_ms": attitude["time_boot_ms"] if attitude else "",
        "attitude_timestamp_utc": attitude["timestamp_utc"] if attitude else "",
        "eph_m": message.eph / 100,
        "epv_m": message.epv / 100,
        "ground_speed_m_s": message.vel / 100,
        "course_deg": message.cog / 100,
        "satellites_visible": message.satellites_visible,
        "signal_power_dbfs": signal_power_dbfs,
    }


def create_heat_map(csv_path: Path):
    import folium
    from folium.plugins import HeatMap

    if not csv_path.exists():
        return None

    points = []
    with csv_path.open("r", newline="", encoding="utf-8") as log_file:
        for row in csv.DictReader(log_file):
            try:
                latitude = float(row["latitude_deg"])
                longitude = float(row["longitude_deg"])
                fix_type = int(row["fix_type"])
                power = float(row["signal_power_dbfs"])
            except (KeyError, TypeError, ValueError):
                continue
            if fix_type >= 2 and -90 <= latitude <= 90 and -180 <= longitude <= 180:
                points.append((latitude, longitude, power))

    if not points:
        return None

    powers = [point[2] for point in points]
    minimum_power = min(powers)
    maximum_power = max(powers)
    power_range = maximum_power - minimum_power
    heat_points = [
        [latitude, longitude, (power - minimum_power) / power_range if power_range else 1]
        for latitude, longitude, power in points
    ]
    center = [
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    ]

    signal_map = folium.Map(location=center, zoom_start=17, control_scale=True)
    HeatMap(
        heat_points,
        radius=22,
        blur=18,
        min_opacity=0.2,
        max_zoom=19,
    ).add_to(signal_map)
    legend = """
    <div style="position:fixed;bottom:28px;left:28px;z-index:9999;background:white;padding:10px 12px;border:1px solid #777;font:13px sans-serif">
      <b>Signal power (dBFS)</b><br>
      Weaker: {minimum:.1f} &nbsp; | &nbsp; Stronger: {maximum:.1f}<br>
      Heat intensity is relative within this scan.
    </div>
    """.format(minimum=minimum_power, maximum=maximum_power)
    signal_map.get_root().html.add_child(folium.Element(legend))

    map_path = csv_path.with_suffix(".html")
    signal_map.save(str(map_path))
    return map_path


def dashboard_snapshot(state, state_lock):
    with state_lock:
        snapshot = {
            key: value for key, value in state.items() if key != "raw_points"
        }
        points = list(state["raw_points"])

    if points:
        powers = [point[2] for point in points]
        minimum_power = min(powers)
        maximum_power = max(powers)
        power_range = maximum_power - minimum_power
        snapshot["points"] = [
            [latitude, longitude, (power - minimum_power) / power_range if power_range else 1]
            for latitude, longitude, power, _ in points
        ]
        snapshot["track_points"] = [
            [latitude, longitude, power, timestamp]
            for latitude, longitude, power, timestamp in points
        ]
        snapshot["minimum_power_dbfs"] = minimum_power
        snapshot["maximum_power_dbfs"] = maximum_power
    else:
        snapshot["points"] = []
        snapshot["track_points"] = []
        snapshot["minimum_power_dbfs"] = None
        snapshot["maximum_power_dbfs"] = None

    snapshot["map_url"] = "/final-map" if snapshot.get("map_path") else None
    return snapshot


def local_network_ip() -> str:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
                probe.connect(("192.0.2.1", 80))
                address = probe.getsockname()[0]
                if not ipaddress.ip_address(address).is_loopback:
                        return address
        except OSError:
                pass
        finally:
                probe.close()

        for result in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                address = result[4][0]
                if not ipaddress.ip_address(address).is_loopback:
                        return address
        return "127.0.0.1"


def start_dashboard(state, state_lock, port: int):
        page = """<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>DroneScan Live</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
    <style>
        :root { color-scheme: light; font: 15px/1.45 system-ui, sans-serif; }
        * { box-sizing: border-box; }
        body { margin: 0; color: #14201f; background: #edf1ee; }
        header { display: flex; flex-wrap: wrap; align-items: center; gap: 16px 28px;
            padding: 14px 20px; background: #173b35; color: #fff; }
        h1 { margin: 0; font-size: 19px; font-weight: 650; }
        #status { padding: 4px 9px; border: 1px solid #83c8a9; border-radius: 3px; color: #c8f2d9; }
        #stats { display: flex; flex-wrap: wrap; gap: 8px 22px; margin-left: auto; }
        #stats span { white-space: nowrap; }
        main { padding: 14px; }
        #map { width: 100%; height: calc(100vh - 115px); min-height: 360px; border: 1px solid #bdc9c4; }
        #legend { position: absolute; right: 28px; bottom: 28px; z-index: 1000; padding: 9px 12px;
            background: #fff; border: 1px solid #89958f; font-size: 13px; }
        #final-link { display: none; color: #145e48; }
        @media (max-width: 600px) { header { gap: 8px 14px; } #stats { margin-left: 0; } main { padding: 8px; } #map { height: calc(100vh - 155px); } }
    </style>
</head>
<body>
    <header>
        <h1>DroneScan</h1><span id="status">Connecting</span>
        <div id="stats">
            <span>Frequency <b id="frequency">--</b></span>
            <span>Local time <b id="clock">--</b></span>
            <span>Samples <b id="samples">0</b></span>
            <span>Current <b id="current">--</b></span>
            <span>GPS <b id="gps">--</b></span>
            <span>Altitude <b id="altitude">--</b></span>
            <span>Attitude <b id="attitude">--</b></span>
            <span>Last fix <b id="last-fix">--</b></span>
            <a id="final-link" href="/final-map" target="_blank" rel="noopener">Open saved map</a>
        </div>
    </header>
    <main><div id="map"></div></main>
    <div id="legend">Signal power (dBFS)<br><span id="range">Waiting for samples</span><br>Heat is relative within this scan.</div>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script src="https://unpkg.com/leaflet.heat/dist/leaflet-heat.js"></script>
    <script>
        const map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19, attribution: '&copy; OpenStreetMap contributors'
        }).addTo(map);
        const heat = L.heatLayer([], {radius: 24, blur: 18, minOpacity: 0.2,
            gradient: {0.2: '#3288bd', 0.45: '#abdda4', 0.7: '#fdae61', 1: '#d53e4f'}}).addTo(map);
        const markers = L.layerGroup().addTo(map);
        let centered = false;
        let markerCount = 0;
        function updateClock() {
            document.getElementById('clock').textContent = new Date().toLocaleTimeString();
        }
        async function refresh() {
            try {
                const response = await fetch('/api/status', {cache: 'no-store'});
                const data = await response.json();
                document.getElementById('status').textContent = data.status;
                document.getElementById('frequency').textContent = `${data.frequency_mhz} MHz`;
                document.getElementById('samples').textContent = data.rows;
                document.getElementById('current').textContent = data.current_power_dbfs === null
                    ? '--' : Number(data.current_power_dbfs).toFixed(1) + ' dBFS';
                document.getElementById('gps').textContent = data.current_gps || '--';
                document.getElementById('altitude').textContent = data.current_altitude_m === null
                    ? '--' : `${Number(data.current_altitude_m).toFixed(1)} m MSL / ${data.current_relative_altitude_m === null ? '--' : Number(data.current_relative_altitude_m).toFixed(1) + ' m rel'}`;
                document.getElementById('attitude').textContent = data.current_attitude || '--';
                document.getElementById('last-fix').textContent = data.last_sample_time_utc
                    ? new Date(data.last_sample_time_utc).toLocaleTimeString() + ' local'
                    : '--';
                document.getElementById('range').textContent = data.minimum_power_dbfs === null
                    ? 'Waiting for samples' : `${Number(data.minimum_power_dbfs).toFixed(1)} to ${Number(data.maximum_power_dbfs).toFixed(1)} dBFS`;
                heat.setLatLngs(data.points);
                for (const point of data.track_points.slice(markerCount)) {
                    const marker = L.circleMarker([point[0], point[1]], {
                        radius: 4, color: '#173b35', weight: 1, fillOpacity: 0.85
                    });
                    const popup = document.createElement('div');
                    popup.textContent = `${new Date(point[3]).toLocaleString()} · ${Number(point[2]).toFixed(1)} dBFS`;
                    marker.bindPopup(popup).addTo(markers);
                }
                markerCount = data.track_points.length;
                if (!centered && data.points.length) {
                    map.setView([data.points[0][0], data.points[0][1]], 18);
                    centered = true;
                }
                if (data.map_url) document.getElementById('final-link').style.display = 'inline';
            } catch (error) {
                document.getElementById('status').textContent = 'Reconnecting';
            }
        }
        updateClock();
        setInterval(updateClock, 1000);
        refresh();
        setInterval(refresh, 1000);
    </script>
</body>
</html>"""

        class DashboardHandler(http.server.BaseHTTPRequestHandler):
                def log_message(self, format_string, *args):
                        pass

                def send_body(self, body: bytes, content_type: str):
                        self.send_response(200)
                        self.send_header("Content-Type", content_type)
                        self.send_header("Content-Length", str(len(body)))
                        self.send_header("Cache-Control", "no-store")
                        self.end_headers()
                        self.wfile.write(body)

                def do_GET(self):
                        if self.path == "/":
                                self.send_body(page.encode("utf-8"), "text/html; charset=utf-8")
                        elif self.path == "/api/status":
                                payload = json.dumps(dashboard_snapshot(state, state_lock)).encode("utf-8")
                                self.send_body(payload, "application/json; charset=utf-8")
                        elif self.path == "/final-map":
                                snapshot = dashboard_snapshot(state, state_lock)
                                map_path = snapshot.get("map_path")
                                if map_path and Path(map_path).is_file():
                                        self.send_body(Path(map_path).read_bytes(), "text/html; charset=utf-8")
                                else:
                                        self.send_error(404, "The scan heat map is not ready yet")
                        else:
                                self.send_error(404)

        server = http.server.ThreadingHTTPServer(("0.0.0.0", port), DashboardHandler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        dashboard_url = "http://{}:{}/".format(local_network_ip(), port)
        return server, dashboard_url


def main():
    args = parse_args()
    if RtlSdr is None:
        raise RuntimeError("pyrtlsdr is required; install firmware/requirements.txt")

    try:
        frequency_mhz, output_path = collect_scan_settings(args)
    except ValueError as error:
        raise SystemExit(str(error))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    frequency_hz = frequency_mhz * 1e6
    if output_path.exists() and output_path.stat().st_size > 0:
        print("Overwriting previous session file {}".format(output_path))
    state = {
        "status": "connecting",
        "frequency_mhz": frequency_mhz,
        "rows": 0,
        "current_power_dbfs": None,
        "current_gps": None,
        "current_altitude_m": None,
        "current_relative_altitude_m": None,
        "current_attitude": None,
        "last_sample_time_utc": None,
        "map_path": None,
        "raw_points": [],
    }
    state_lock = threading.Lock()
    dashboard_server, dashboard_url = start_dashboard(
        state, state_lock, args.dashboard_port
    )
    print("Live dashboard: {}".format(dashboard_url))
    print("Open this address on another device on the same network.")

    sdr_device = None
    master = None
    attitude = None
    global_position = None
    try:
        sdr_device = RtlSdr()
        master = mavutil.mavlink_connection(args.port, baud=args.baud)
        sdr_device.sample_rate = 2.4e6
        sdr_device.gain = "auto"

        print("Waiting for flight-controller heartbeat...")
        master.wait_heartbeat()
        with state_lock:
            state["status"] = "scanning"
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        ):
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1e6 / args.gps_rate_hz,
                0,
                0,
                0,
                0,
                0,
            )

        print(
            "Scanning {:.6g} MHz until Ctrl+C; logging to {}".format(
                frequency_mhz, output_path
            )
        )
        with output_path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            while True:
                message = master.recv_match(
                    type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "ATTITUDE"],
                    blocking=True,
                    timeout=1,
                )
                if message is None:
                    continue
                message_type = message.get_type()
                if message_type == "ATTITUDE":
                    attitude = {
                        "time_boot_ms": message.time_boot_ms,
                        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                        "roll_deg": math.degrees(message.roll),
                        "pitch_deg": math.degrees(message.pitch),
                        "yaw_deg": math.degrees(message.yaw),
                    }
                    with state_lock:
                        state["current_attitude"] = "R {:.1f}° / P {:.1f}° / Y {:.1f}°".format(
                            attitude["roll_deg"],
                            attitude["pitch_deg"],
                            attitude["yaw_deg"],
                        )
                    continue
                if message_type == "GLOBAL_POSITION_INT":
                    global_position = {
                        "time_boot_ms": message.time_boot_ms,
                        "relative_altitude_m": message.relative_alt / 1000,
                    }
                    with state_lock:
                        state["current_relative_altitude_m"] = global_position[
                            "relative_altitude_m"
                        ]
                    continue

                signal_power_dbfs = listen_freq(
                    sdr_device, frequency_hz, args.sample_count
                )
                row = gps_row(
                    message,
                    frequency_hz,
                    signal_power_dbfs,
                    attitude,
                    global_position,
                )
                writer.writerow(row)
                log_file.flush()
                with state_lock:
                    state["rows"] += 1
                    state["current_power_dbfs"] = signal_power_dbfs
                    state["last_sample_time_utc"] = row["timestamp_utc"]
                    state["current_altitude_m"] = row["altitude_m"]
                    state["current_relative_altitude_m"] = (
                        row["relative_altitude_m"]
                        if row["relative_altitude_m"] != ""
                        else state["current_relative_altitude_m"]
                    )
                    if row["fix_type"] >= 2 and row["latitude_deg"] != "":
                        state["current_gps"] = "{:.6f}, {:.6f}".format(
                            row["latitude_deg"], row["longitude_deg"]
                        )
                        state["raw_points"].append(
                            (
                                row["latitude_deg"],
                                row["longitude_deg"],
                                signal_power_dbfs,
                                row["timestamp_utc"],
                            )
                        )
                    else:
                        state["current_gps"] = "No GPS fix"
                print(
                    f"GPS {message.lat / 1e7:.7f}, {message.lon / 1e7:.7f} "
                    f"fix={message.fix_type} signal={signal_power_dbfs:.1f} dBFS"
                )
    except KeyboardInterrupt:
        print("Stopping scan and saving the session map.")
        with state_lock:
            state["status"] = "stopped"
    finally:
        if master is not None:
            master.close()
        if sdr_device is not None:
            sdr_device.close()

    map_path = create_heat_map(output_path)
    if map_path is None:
        print("No valid GPS fixes were logged; no heat map was created.")
    else:
        print("Heat map saved to {}".format(map_path.resolve()))
        with state_lock:
            state["map_path"] = str(map_path.resolve())
        webbrowser.open(dashboard_url)

    print("Dashboard remains available at {}".format(dashboard_url))
    print("Press Ctrl+C again to stop the dashboard server.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping dashboard server.")
    finally:
        dashboard_server.shutdown()
        dashboard_server.server_close()


if __name__ == "__main__":
    main()