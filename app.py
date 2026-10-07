from flask import Flask, render_template, request, jsonify, make_response
import time, json, os, threading, socket

app = Flask(__name__)

AUTH_KEY        = "iiitdm-authority-2026"   # must match ADMIN_KEY in index.html
STATE_FILE      = "state_backup.json"
SAVE_DEBOUNCE_S = 5.0
STALE_BREACH_S  = 15
ARM_REQ_TTL_S   = 35   # ESP gives up after 30s
_last_save      = 0.0

lock            = threading.Lock()
fleet_data      = {}
control_state   = {}
flight_requests = {}
command_queue   = {}
arm_requests    = {}
# { "Drone-1": { "status": "none|pending|approved|denied" } }

GEOFENCE_ZONES = []   # draw zones from the dashboard; none preset for Yashobhoomi
geo_lock      = threading.Lock()
_zone_counter = 4

# â”€â”€ State persistence â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def save_state(force=False):
    global _last_save
    now = time.time()
    if not force and now - _last_save < SAVE_DEBOUNCE_S:
        return
    _last_save = now
    try:
        with lock:
            fleet_snap    = dict(fleet_data)
            control_snap  = {k: dict(v) for k, v in control_state.items()}
            req_snap      = {k: dict(v) for k, v in flight_requests.items()}
            arm_req_snap  = {k: dict(v) for k, v in arm_requests.items()}
        with geo_lock:
            geo_snap  = list(GEOFENCE_ZONES)
            zone_snap = _zone_counter
        with open(STATE_FILE, "w") as f:
            json.dump({"fleet":fleet_snap,"control":control_snap,
                       "requests":req_snap,"arm_requests":arm_req_snap,
                       "geofence":geo_snap,
                       "zone_counter":zone_snap}, f, indent=2)
    except Exception as e:
        print(f"[WARN] save_state: {e}")

def load_state():
    global fleet_data, control_state, flight_requests, arm_requests, GEOFENCE_ZONES, _zone_counter
    if not os.path.exists(STATE_FILE):
        return
    try:
        with open(STATE_FILE) as f:
            d = json.load(f)
        fleet_data        = d.get("fleet",    {})
        control_state     = d.get("control",  {})
        flight_requests   = d.get("requests", {})
        arm_requests      = d.get("arm_requests", {})
        old_defaults = {("Z1","Admin Block"),("Z2","Lab Complex"),("Z3","Power Station")}  # legacy Kurnool zones
        GEOFENCE_ZONES[:] = [z for z in d.get("geofence", GEOFENCE_ZONES)
                             if (z.get("id"), z.get("label")) not in old_defaults]
        _zone_counter     = d.get("zone_counter", 4)
        print(f"[INFO] State restored: {len(fleet_data)} drone(s), {len(GEOFENCE_ZONES)} zone(s)")
    except Exception as e:
        print(f"[WARN] load_state: {e}")

load_state()

# â”€â”€ Helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def check_auth():
    return request.headers.get("X-Authority-Key") == AUTH_KEY

def auth_required():
    return jsonify({"status":"forbidden","reason":"Invalid or missing X-Authority-Key"}), 403

def default_control():
    return {"flight_approved":False,"override":False,"breach":False,
            "breach_zone":None,"override_by":None,"locked_at":None}

def check_geofence(lat, lon):
    with geo_lock:
        zones = list(GEOFENCE_ZONES)
    for z in zones:
        if z["lat_min"] <= lat <= z["lat_max"] and z["lon_min"] <= lon <= z["lon_max"]:
            return z
    return None

def enqueue_cmd(drone_id, cmd_type, payload=None):
    with lock:
        d_data = fleet_data.get(drone_id)
        if not d_data:
            return jsonify({"status":"not_found"}), 404
        if time.time() - d_data.get("last_seen", 0) > 10:
            return jsonify({"status":"offline","reason":"Drone not seen in >10s"}), 409
        if drone_id not in command_queue:
            command_queue[drone_id] = []
        entry = {"cmd": cmd_type}
        if payload:
            entry.update(payload)
        command_queue[drone_id].append(entry)
    print(f"[QUEUE] {drone_id} â† {cmd_type}")
    return jsonify({"status":"queued","cmd":cmd_type})

# â”€â”€ CORS â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.before_request
def handle_options():
    if request.method == "OPTIONS":
        return make_response()

@app.after_request
def after_request(response):
    response.headers.add("Access-Control-Allow-Origin",  "*")
    response.headers.add("Access-Control-Allow-Headers", "*")
    response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    return response

# â”€â”€ Pages â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/pilot")
def pilot_page():
    return render_template("pilot.html")

# â”€â”€ Drone telemetry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/update", methods=["POST","OPTIONS"])
def update():
    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify({"status":"bad_request"}), 400
    drone_id = data.get("id")
    if not drone_id:
        return jsonify({"status":"no_id"}), 400

    with lock:
        data["last_seen"] = time.time()
        fleet_data[drone_id] = data
        if drone_id not in control_state:
            control_state[drone_id] = default_control()
        cs = control_state[drone_id]

        try:
            lat = float(data.get("lat", 0))
            lon = float(data.get("lon", 0))
        except:
            lat, lon = 0.0, 0.0

        # Auto-reset approval when drone disarms after flight
        is_armed   = data.get("armed", False)
        was_armed  = cs.get("_was_armed", False)
        was_approved = cs.get("flight_approved", False)
        if was_armed and not is_armed and was_approved:
            cs["flight_approved"] = False
            if drone_id in flight_requests:
                flight_requests[drone_id]["status"] = "none"
            print(f"[RESET] {drone_id} disarmed â€” authorization reset")
        if (was_armed and not is_armed and drone_id in arm_requests
                and arm_requests[drone_id].get("status") in ("approved", "denied")):
            arm_requests[drone_id]["status"] = "none"
        cs["_was_armed"] = is_armed

        # Geofence check (skip if GPS not fixed)
        if lat != 0.0 and lon != 0.0:
            zone = check_geofence(lat, lon)
            if zone and not cs["breach"]:
                cs.update({"override":True,"breach":True,
                           "breach_zone":f"{zone['id']}: {zone['label']}",
                           "override_by":"geofence","locked_at":time.time()})
                print(f"[BREACH] {drone_id} entered {zone['id']}")
            elif not zone and cs["breach"] and cs["override_by"] == "geofence":
                cs.update({"breach":False,"breach_zone":None,
                           "override":False,"override_by":None,"locked_at":None})
                print(f"[CLEAR] {drone_id} exited geofence")

    save_state()
    return jsonify({"status":"success"})

@app.route("/get_fleet")
def get_fleet():
    now = time.time()
    with lock:
        active = {id: d for id, d in fleet_data.items()
                  if now - d.get("last_seen", 0) < 10}
    return jsonify(active)

@app.route("/control_state")
def get_control_state():
    with lock:
        snapshot = dict(control_state)
    return jsonify(snapshot)

# â”€â”€ Pilot requests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/pilot/request", methods=["POST","OPTIONS"])
def pilot_request():
    data     = request.get_json(force=True, silent=True) or {}
    drone_id = data.get("drone_id", "").strip()
    if not drone_id:
        return jsonify({"status":"error","reason":"no drone_id"}), 400
    with lock:
        existing = flight_requests.get(drone_id, {})
        if existing.get("status") == "pending" or (existing.get("status") == "approved" and control_state.get(drone_id, {}).get("flight_approved", False)):
            return jsonify({"status":existing["status"]})
        flight_requests[drone_id] = {"drone_id":drone_id,"status":"pending",
                                     "requested_at":time.time()}
    save_state(force=True)
    print(f"[REQUEST] {drone_id} â€” pilot requesting to fly")
    return jsonify({"status":"requested"})

@app.route("/pilot/status/<drone_id>")
def pilot_status(drone_id):
    with lock:
        req = flight_requests.get(drone_id, {})
        cs  = control_state.get(drone_id, {})
    req_status = req.get("status", "none")
    flight_ok  = cs.get("flight_approved", False)
    if flight_ok or req_status == "approved": status = "approved"
    elif req_status == "denied":              status = "denied"
    elif req_status == "pending":             status = "waiting"
    else:                                     status = "none"
    return jsonify({"approved":(status=="approved"),"status":status})

@app.route("/pilot/requests")
def get_pilot_requests():
    with lock:
        snapshot = dict(flight_requests)
    return jsonify(snapshot)

# â”€â”€ Arm requests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/drone/arm_request", methods=["POST","OPTIONS"])
def drone_arm_request():
    data     = request.get_json(force=True, silent=True) or {}
    drone_id = data.get("drone_id", "").strip()
    if not drone_id:
        return jsonify({"status":"error","reason":"no drone_id"}), 400
    with lock:
        existing = arm_requests.get(drone_id, {})
        if existing.get("status") != "pending":
            arm_requests[drone_id] = {"status":"pending","requested_at":time.time()}
    save_state(force=True)
    print(f"[ARM REQUEST] {drone_id} â€” FC requesting arm authorization")
    return jsonify({"status":"pending"})

def _expire_arm(drone_id):
    """Call with `lock` held. Pending requests older than the TTL become 'none'."""
    req = arm_requests.get(drone_id, {"status":"none"})
    if req.get("status") == "pending" and time.time() - req.get("requested_at", 0) > ARM_REQ_TTL_S:
        req["status"] = "none"
        arm_requests[drone_id] = req
    return dict(req)

@app.route("/drone/poll_arm_permission/<drone_id>")
def poll_arm_permission(drone_id):
    with lock:
        req = _expire_arm(drone_id)
    return jsonify(req)

@app.route("/authority/approve_arm/<drone_id>", methods=["POST"])
def authority_approve_arm(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        arm_requests[drone_id] = {"status":"approved"}
    save_state(force=True)
    print(f"[ARM APPROVED] {drone_id}")
    return jsonify({"status":"approved"})

@app.route("/authority/deny_arm/<drone_id>", methods=["POST"])
def authority_deny_arm(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        arm_requests[drone_id] = {"status":"denied"}
    save_state(force=True)
    print(f"[ARM DENIED] {drone_id}")
    return jsonify({"status":"denied"})

@app.route("/authority/arm_requests")
def get_arm_requests():
    with lock:
        for did in list(arm_requests):
            _expire_arm(did)
        snapshot = {k: dict(v) for k, v in arm_requests.items()}
    return jsonify(snapshot)

# â”€â”€ Authority commands â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/authority/approve/<drone_id>", methods=["POST"])
def authority_approve(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        if drone_id not in fleet_data or time.time() - fleet_data[drone_id].get("last_seen",0) > 10:
            return jsonify({"status":"not_found","reason":"Drone not online"}), 404
        if drone_id not in control_state:
            control_state[drone_id] = default_control()
        control_state[drone_id]["flight_approved"] = True
        if drone_id in flight_requests:
            flight_requests[drone_id]["status"] = "approved"
        else:
            flight_requests[drone_id] = {"drone_id":drone_id,"status":"approved",
                                         "requested_at":time.time()}
    save_state(force=True)
    print(f"[APPROVED] {drone_id}")
    return jsonify({"status":"approved"})

@app.route("/authority/deny/<drone_id>", methods=["POST"])
def authority_deny(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        if drone_id in flight_requests:
            flight_requests[drone_id]["status"] = "denied"
        if drone_id in control_state:
            control_state[drone_id]["flight_approved"] = False
    save_state(force=True)
    print(f"[DENIED] {drone_id}")
    return jsonify({"status":"denied"})

@app.route("/authority/override/<drone_id>", methods=["POST"])
def authority_override(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        if drone_id not in control_state:
            control_state[drone_id] = default_control()
        control_state[drone_id].update({"override":True,"override_by":"manual",
                                        "locked_at":time.time()})
    save_state(force=True)
    print(f"[OVERRIDE] {drone_id}")
    return jsonify({"status":"override_active"})

@app.route("/authority/release/<drone_id>", methods=["POST"])
def authority_release(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        if drone_id not in control_state:
            return jsonify({"status":"not_found"}), 404
        control_state[drone_id].update({"override":False,"breach":False,
                                        "breach_zone":None,"override_by":None,"locked_at":None})
        if drone_id in flight_requests:
            flight_requests[drone_id]["status"] = "none"
    save_state(force=True)
    print(f"[RELEASE] {drone_id}")
    return jsonify({"status":"released"})

@app.route("/authority/rtl/<drone_id>")
def authority_rtl(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        cs = control_state.get(drone_id, {})
    if not (cs.get("override") or cs.get("breach")):
        return jsonify({"status":"blocked","reason":"Take override first"}), 403
    return enqueue_cmd(drone_id, "rtl")

@app.route("/authority/kill/<drone_id>")
def authority_kill(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        cs = control_state.get(drone_id, {})
    if not (cs.get("override") or cs.get("breach")):
        return jsonify({"status":"blocked","reason":"Take override first"}), 403
    return enqueue_cmd(drone_id, "kill")

@app.route("/authority/hover/<drone_id>")
def authority_hover(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        cs = control_state.get(drone_id, {})
    if not (cs.get("override") or cs.get("breach")):
        return jsonify({"status":"blocked","reason":"Take override first"}), 403
    return enqueue_cmd(drone_id, "hover")

@app.route("/authority/land/<drone_id>")
def authority_land(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        cs = control_state.get(drone_id, {})
    if not (cs.get("override") or cs.get("breach")):
        return jsonify({"status":"blocked","reason":"Take override first"}), 403
    return enqueue_cmd(drone_id, "land")

@app.route("/authority/move/<drone_id>", methods=["POST"])
def authority_move(drone_id):
    if not check_auth(): return auth_required()
    with lock:
        if drone_id not in control_state:
            control_state[drone_id] = default_control()
        cs_snap = dict(control_state[drone_id])
    if not (cs_snap.get("override") or cs_snap.get("breach")):
        return jsonify({"status":"blocked","reason":"Take override first"}), 403
    data = request.get_json(force=True, silent=True) or {}
    try:
        vx = float(data.get("vx", 0.0))
        vy = float(data.get("vy", 0.0))
        vz = float(data.get("vz", 0.0))
    except:
        return jsonify({"status":"bad_request","reason":"vx/vy/vz must be numbers"}), 400
    return enqueue_cmd(drone_id, "move", {"vx":vx,"vy":vy,"vz":vz})

# â”€â”€ ESP command poll â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/drone/poll_commands/<drone_id>")
def poll_commands(drone_id):
    with lock:
        cmds = command_queue.pop(drone_id, [])
    if cmds:
        print(f"[POLL] {drone_id} fetched {len(cmds)} cmd(s): {[c['cmd'] for c in cmds]}")
    return jsonify(cmds)

# â”€â”€ Geofence â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
@app.route("/geofence/zones")
def get_zones():
    with geo_lock:
        snapshot = list(GEOFENCE_ZONES)
    return jsonify(snapshot)

@app.route("/geofence/add", methods=["POST"])
def add_zone():
    if not check_auth(): return auth_required()
    global _zone_counter
    data = request.get_json(force=True, silent=True) or {}
    try:
        label   = str(data["label"]).strip()
        lat_min = float(data["lat_min"])
        lat_max = float(data["lat_max"])
        lon_min = float(data["lon_min"])
        lon_max = float(data["lon_max"])
    except (KeyError, TypeError, ValueError) as e:
        return jsonify({"status":"bad_request","reason":str(e)}), 400
    if not label:
        return jsonify({"status":"bad_request","reason":"label required"}), 400
    if lat_min >= lat_max or lon_min >= lon_max:
        return jsonify({"status":"bad_request","reason":"min must be < max"}), 400
    with geo_lock:
        zone_id = f"Z{_zone_counter}"
        _zone_counter += 1
        zone = {"id":zone_id,"label":label,"lat_min":lat_min,
                "lat_max":lat_max,"lon_min":lon_min,"lon_max":lon_max}
        GEOFENCE_ZONES.append(zone)
    save_state(force=True)
    print(f"[GEOFENCE] Added {zone_id}: {label}")
    return jsonify({"status":"added","zone":zone})

@app.route("/geofence/delete/<zone_id>", methods=["POST"])
def delete_zone(zone_id):
    if not check_auth(): return auth_required()
    with geo_lock:
        before = len(GEOFENCE_ZONES)
        GEOFENCE_ZONES[:] = [z for z in GEOFENCE_ZONES if z["id"] != zone_id]
        removed = before - len(GEOFENCE_ZONES)
    if removed == 0:
        return jsonify({"status":"not_found"}), 404
    save_state(force=True)
    print(f"[GEOFENCE] Deleted {zone_id}")
    return jsonify({"status":"deleted"})

# â”€â”€ Background threads â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def _watchdog():
    """Auto-clear stale breach state for offline drones."""
    while True:
        time.sleep(10)
        now = time.time()
        with lock:
            active = {id for id, d in fleet_data.items()
                      if now - d.get("last_seen", 0) < 10}
            changed = False
            for did, cs in list(control_state.items()):
                if (cs.get("breach") and cs.get("override_by") == "geofence"
                        and did not in active and cs.get("locked_at")
                        and now - cs["locked_at"] > STALE_BREACH_S):
                    cs.update({"breach":False,"breach_zone":None,
                               "override":False,"override_by":None,"locked_at":None})
                    print(f"[WATCHDOG] {did} stale breach auto-cleared")
                    changed = True
        if changed:
            save_state(force=True)

def _udp_broadcast():
    """Broadcast server IP so ESP8266 can auto-discover without hardcoding."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    msg = b"DRONE-ATC:5000"
    print("[UDP] Broadcasting server presence on port 2390...")
    while True:
        try:
            sock.sendto(msg, ("255.255.255.255", 2390))
        except Exception as e:
            print(f"[UDP] Broadcast error: {e}")
        time.sleep(3)

threading.Thread(target=_watchdog,      daemon=True).start()
threading.Thread(target=_udp_broadcast, daemon=True).start()

# â”€â”€ Start â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
if __name__ == "__main__":
    print("=" * 52)
    print("  IIITDM Kurnool â€” Drone ATC Server")
    print(f"  Auth Key  : {AUTH_KEY}")
    print(f"  Dashboard : http://localhost:5000")
    print(f"  Pilot     : http://localhost:5000/pilot")
    print(f"  UDP Bcast : port 2390 (ESP auto-discovers)")
    print("=" * 52)
    app.run(host="0.0.0.0", port=5000, debug=False)
