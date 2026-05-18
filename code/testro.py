import cv2
import time
from inference_sdk import InferenceHTTPClient

# -----------------------------
# Roboflow client
# -----------------------------
print("[DEBUG] Initializing Roboflow client...")

client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="kGDr4Cupwm8dn1KnZrx3"
)

WORKSPACE_NAME = "label-y3rcq"
WORKFLOW_ID = "find-soda-can"

# -----------------------------
# MJPEG stream URL
# -----------------------------
STREAM_URL = "http://10.155.65.112:81/stream"

print(f"[DEBUG] Connecting to stream: {STREAM_URL}")

cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print("[ERROR] Failed to open MJPEG stream.")
    exit()

print("[DEBUG] Stream opened successfully.")

# -----------------------------
# Grab ONE frame
# -----------------------------
print("[DEBUG] Reading frame from stream...")

ret, frame = cap.read()

if not ret:
    print("[ERROR] Failed to read frame from stream.")
    cap.release()
    exit()

print("[DEBUG] Frame received.")

# Image dimensions
h, w = frame.shape[:2]

print(f"[DEBUG] Frame size: {w}x{h}")

cx_img = w / 2
cy_img = h / 2

print(f"[DEBUG] Image center: ({cx_img}, {cy_img})")

# -----------------------------
# Save frame
# -----------------------------
tmp_path = f"debug_{int(time.time())}.jpg"

print(f"[DEBUG] Saving frame to {tmp_path}")

success = cv2.imwrite(tmp_path, frame)

if not success:
    print("[ERROR] Failed to save frame.")
    cap.release()
    exit()

print("[DEBUG] Frame saved successfully.")

# -----------------------------
# Run Roboflow workflow
# -----------------------------
print("[DEBUG] Sending image to Roboflow...")
start_time = time.time()

result = client.run_workflow(
    workspace_name=WORKSPACE_NAME,
    workflow_id=WORKFLOW_ID,
    images={
        "image": tmp_path
    },
    use_cache=True
)

elapsed = time.time() - start_time

print(f"[DEBUG] Roboflow inference completed in {elapsed:.2f} seconds")

# -----------------------------
# Print raw result
# -----------------------------
print("\n========== RAW RESULT ==========")
print(result)
print("================================\n")

# -----------------------------
# Extract detections
# -----------------------------
try:
    preds = result[0]["predictions"]["predictions"]
except Exception as e:
    print("[ERROR] Failed to parse predictions:")
    print(e)
    preds = []

print(f"[DEBUG] Number of predictions: {len(preds)}")

if preds:
    # Print all detections
    for i, p in enumerate(preds):
        print(f"\n[DEBUG] Prediction {i}:")
        print(p)

    # pick highest confidence detection
    best = max(preds, key=lambda p: p.get("confidence", 0))

    print("\n[DEBUG] Highest confidence detection:")
    print(best)

    ox = best["x"]
    oy = best["y"]
    conf = best.get("confidence", 0)

    dx = ox - cx_img
    dy = oy - cy_img

    ndx = dx / cx_img
    ndy = dy / cy_img

    print("\n========== DETECTION ==========")
    print(f"Class:         {best.get('class', 'unknown')}")
    print(f"Object center: ({ox:.1f}, {oy:.1f})")
    print(f"Confidence:    {conf:.3f}")
    print(f"Offset pixels: dx={dx:.1f}, dy={dy:.1f}")
    print(f"Normalized:    ndx={ndx:.3f}, ndy={ndy:.3f}")
    print("================================")

else:
    print("[DEBUG] No detections found.")

# -----------------------------
# Cleanup
# -----------------------------
print("[DEBUG] Releasing stream.")

cap.release()

print("[DEBUG] Done.")
