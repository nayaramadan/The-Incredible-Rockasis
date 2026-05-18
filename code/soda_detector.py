# soda_detector.py
#
# Reusable soda can detector module
#
# Usage:
#
# from soda_detector import SodaDetector
#
# detector = SodaDetector(
#     stream_url="http://192.168.1.10:81/stream",
#     api_key="YOUR_KEY"
# )
#
# result = detector.get_offset()
#
# if result:
#     print(result["offset_x"])
#     print(result["offset_y"])
#
# detector.close()

import cv2
import time
import tempfile
import os

from inference_sdk import InferenceHTTPClient


class SodaDetector:
    def __init__(
        self,
        stream_url,
        api_key,
        workspace="label-y3rcq",
        workflow="find-soda-can",
        process_every=10,
    ):
        self.process_every = process_every
        self.frame_count = 0

        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=api_key,
        )

        self.workspace = workspace
        self.workflow = workflow

        self.cap = cv2.VideoCapture(stream_url)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open stream: {stream_url}")

    def _run_inference(self, frame):
        """
        Internal helper.
        Runs Roboflow inference on a frame.
        """

        with tempfile.NamedTemporaryFile(
            suffix=".jpg",
            delete=False
        ) as tmp:
            temp_path = tmp.name

        try:
            cv2.imwrite(temp_path, frame)

            start = time.time()

            result = self.client.run_workflow(
                workspace_name=self.workspace,
                workflow_id=self.workflow,
                images={"image": temp_path},
                use_cache=True,
            )

            inference_time = time.time() - start

            try:
                preds = result[0]["predictions"]["predictions"]
            except Exception:
                preds = []

            return preds, inference_time

        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)

    def get_offset(self):
        """
        Returns object offset from image center.

        Returns:
            None if no object detected

            OR

            {
                "offset_x": normalized_horizontal_offset,
                "offset_y": normalized_vertical_offset,

                # raw pixel offsets
                "dx": pixel_offset_x,
                "dy": pixel_offset_y,

                # object center
                "x": object_x,
                "y": object_y,

                "confidence": confidence,
                "inference_time": seconds
            }

        offset_x:
            -1.0 = far left
             0.0 = centered
            +1.0 = far right

        offset_y:
            -1.0 = top
             0.0 = centered
            +1.0 = bottom
        """

        while True:
            ret, frame = self.cap.read()

            if not ret:
                return None

            self.frame_count += 1

            # Skip frames for performance
            if self.frame_count % self.process_every != 0:
                continue

            h, w = frame.shape[:2]

            center_x = w / 2
            center_y = h / 2

            preds, inference_time = self._run_inference(frame)

            if not preds:
                return None

            # highest confidence prediction
            best = max(
                preds,
                key=lambda p: p.get("confidence", 0)
            )

            obj_x = best["x"]
            obj_y = best["y"]

            confidence = best.get("confidence", 0)

            dx = obj_x - center_x
            dy = obj_y - center_y

            # normalize to -1.0 to +1.0
            offset_x = dx / center_x
            offset_y = dy / center_y

            return {
                "offset_x": offset_x,
                "offset_y": offset_y,
                "dx": dx,
                "dy": dy,
                "x": obj_x,
                "y": obj_y,
                "confidence": confidence,
                "inference_time": inference_time,
            }

    def close(self):
        self.cap.release()
