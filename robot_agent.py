import cv2
import json
import time
import numpy as np
import threading
import paho.mqtt.client as mqtt
from ultralytics import YOLO
from groq import Groq

# =====================================================================
# SYSTEM KEYROUTING & INSTANTIATIONS
# =====================================================================
GROQ_KEY_NAVIGATOR = "YOUR_GROQ_KEY_1"
GROQ_KEY_STRATEGIST = "YOUR_GROQ_KEY_2"

agent_navigator = Groq(api_key=GROQ_KEY_NAVIGATOR)
agent_strategist = Groq(api_key=GROQ_KEY_STRATEGIST)

PUBLIC_BROKER = "broker.hivemq.com"
PORT = 1883
PREFIX = "efe_robot_2026"

TOPIC_CONFIG = f"{PREFIX}/dashboard/config"
TOPIC_MOVE = f"{PREFIX}/pi/cmd/move"
TOPIC_ARM = f"{PREFIX}/pi/cmd/arm"

# =====================================================================
# RUNTIME CONFIGURATION & DYNAMIC PARAMETERS
# =====================================================================
model = YOLO('yolov8n.tflite') # Ensure model is exported to INT8 TFLite
FRAME_WIDTH = 640
FRAME_CENTER = FRAME_WIDTH // 2
BASE_SPEED = 35
KP = 0.45

# Dynamic targets synchronized from your upgraded website dashboard
TARGET_CLASS = "cup"
TARGET_COLOR = "red"
CONFIDENCE_THRESHOLD = 0.75  # Set dynamically by dashboard slider (0.10 - 0.99)
MISSION_MODE = "track"       # track, grab, sort, inspect

agent_active = False
system_lock = False

# OpenCV HSV Color Space Map Boundaries
COLOR_RANGES = {
    "red": {"low": np.array([0, 120, 70]), "high": np.array([10, 255, 255])},
    "blue": {"low": np.array([94, 80, 2]), "high": np.array([126, 255, 255])},
    "green": {"low": np.array([25, 52, 72]), "high": np.array([102, 255, 255])},
    "yellow": {"low": np.array([20, 100, 100]), "high": np.array([30, 255, 255])}
}

# =====================================================================
# NETWORKING HANDLERS WITH UPGRADED PARSING
# =====================================================================
def on_connect(client, userdata, flags, rc):
    print(f"[SYSTEM INITIALIZED]: MQTT Online. Subscribed to telemetry namespace: {PREFIX}/#")
    client.subscribe(TOPIC_CONFIG)

def on_message(client, userdata, msg):
    global TARGET_CLASS, TARGET_COLOR, CONFIDENCE_THRESHOLD, MISSION_MODE
    try:
        data = json.loads(msg.payload.decode())
        
        # Sync all parameters incoming from the web client
        TARGET_CLASS = data.get("target_object", TARGET_CLASS)
        TARGET_COLOR = data.get("target_color", TARGET_COLOR)
        CONFIDENCE_THRESHOLD = data.get("confidence_threshold", CONFIDENCE_THRESHOLD)
        MISSION_MODE = data.get("mission_mode", MISSION_MODE)
        
        print(f"\n[MISSION CONFIG SYNCHRONIZED] ID: {data.get('mission_id', 'N/A')}")
        print(f" -> Mode: {MISSION_MODE.upper()}")
        print(f" -> Class: {TARGET_CLASS} | Color: {TARGET_COLOR}")
        print(f" -> YOLO Confidence Barrier: {CONFIDENCE_THRESHOLD * 100:.0f}%")
    except Exception as e:
        print(f"Upgraded payload parse failure: {e}")

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(PUBLIC_BROKER, PORT, 60)
mqtt_client.loop_start()

# =====================================================================
# BACKGROUND ASYNCHRONOUS AGENT PROCESSOR
# =====================================================================
def run_agentic_pipeline(label):
    global agent_active, system_lock
    try:
        print(f"[AGENTS ACTIVATED]: Processing cloud intelligence for target: {label}")
        
        # Call Agent Alpha (Navigator Validation)
        nav_prompt = f"Verify if a '{label}' is typically safe to handle with a small gripper claw. Output JSON: {{\"valid\": true/false}}"
        nav_res = agent_navigator.chat.completions.create(
            messages=[{"role": "user", "content": nav_prompt}],
            model="llama3-8b-8192", response_format={"type": "json_object"}
        )
        if not json.loads(nav_res.choices[0].message.content).get("valid", True):
            print("[AGENT COORD]: Target marked unsafe. Aborting hardware interaction sequence.")
            system_lock = False; agent_active = False; return

        # Call Agent Beta (Torque / Grip Pressure Assessment)
        strat_prompt = f"Assess structural stiffness of a '{label}'. Output JSON: {{\"force_threshold\": \"low\"|\"medium\"|\"high\"}}"
        strat_res = agent_strategist.chat.completions.create(
            messages=[{"role": "user", "content": strat_prompt}],
            model="llama3-8b-8192", response_format={"type": "json_object"}
        )
        force_tier = json.loads(strat_res.choices[0].message.content).get("force_threshold", "medium")
        
        force_map = {"low": 1200, "medium": 1950, "high": 2750}
        target_force = force_map.get(force_tier, 1950)

        # Broadcast hardware interaction configuration with chosen mission action context
        payload = {
            "action": MISSION_MODE, 
            "force_limit": target_force
        }
        mqtt_client.publish(TOPIC_ARM, json.dumps(payload))
        print(f"[COMMAND DISPATCHED TO CHASSIS]: Action: {MISSION_MODE.upper()} | Force safe constraint: {target_force}")
        
        time.sleep(12) # Block tracking for duration of structural movement sequence
        
    except Exception as e:
        print(f"[CLOUD TRANSACTION ERROR]: {e}")
    finally:
        system_lock = False
        agent_active = False

# =====================================================================
# MAIN TRACKING CONTINUUM WITH DYNAMIC FILTERING
# =====================================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret or system_lock:
        cv2.waitKey(30); continue
        
    results = model(frame, stream=True)
    object_found = False
    
    for r in results:
        for box in r.boxes:
            label = model.names[int(box.cls[0])]
            confidence = float(box.conf[0]) # Get actual detection confidence
            
            # Filter 1: Match target type AND current dashboard confidence threshold
            if label == TARGET_CLASS and confidence >= CONFIDENCE_THRESHOLD:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0: continue
                
                # Filter 2: Verify specific color criteria matches inside bounding box
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                bounds = COLOR_RANGES.get(TARGET_COLOR, COLOR_RANGES["red"])
                mask = cv2.inRange(hsv_roi, bounds["low"], bounds["high"])
                
                if cv2.countNonZero(mask) > (roi.size * 0.05): # Minimum 5% pixel match density
                    object_found = True
                    x_center = int((x1 + x2) / 2)
                    
                    # Proportional turn output tracking error calculations
                    steer_val = int((x_center - FRAME_CENTER) * KP)
                    steer_val = max(-100, min(100, steer_val))
                    
                    # Proximity lock checking via bounding box width
                    if (x2 - x1) > 260:
                        # Bring chassis wheels to an immediate stop
                        mqtt_client.publish(TOPIC_MOVE, json.dumps({"speed": 0, "steer": 0, "hold": True}))
                        
                        # Handle decision matrix based on selected dashboard mode
                        if MISSION_MODE == "track":
                            print("[PROXIMITY LOCK]: Object reached. Holding tracking target position.")
                        elif MISSION_MODE in ["grab", "sort"] and not agent_active:
                            system_lock = True; agent_active = True
                            threading.Thread(target=run_agentic_pipeline, args=(label,)).start()
                        elif MISSION_MODE == "inspect":
                            print(f"[INSPECTION LOCK]: Target object '{label}' analyzed with confidence {confidence*100:.1f}%.")
                            time.sleep(2) # Hold frame position temporarily
                    else:
                        # Drive towards target with proportional steering correction adjustments
                        mqtt_client.publish(TOPIC_MOVE, json.dumps({"speed": BASE_SPEED, "steer": steer_val, "hold": False}))
                    break
        if object_found: break

    if not object_found:
        # Revert to a safe auto-sweep scanning rotation mode if no tracking matches are found
        mqtt_client.publish(TOPIC_MOVE, json.dumps({"speed": 0, "steer": 35, "hold": False}))
        time.sleep(0.02)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
