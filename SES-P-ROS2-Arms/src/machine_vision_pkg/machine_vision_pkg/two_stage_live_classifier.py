#!/usr/bin/env python3

"""
Two-stage RealSense classification node:

  Stage 1: YOLOv8-seg detects high-level mechanical classes (gear, nut, screw).
  Stage 2: image-to-geometry classifier predicts low-level classes
           (gear_a, nut_b, screw_f, ...).

Subscribes to synchronized color + aligned depth + camera_info via
RealsenseSyncedRGBDNode. Publishes vision_msgs/Detection2DArray on
/classifier/detections (configurable). Each Detection2D carries:
    - 2D bounding box,
    - the high-level YOLO label as one ObjectHypothesisWithPose,
    - the low-level Stage-2 label as another ObjectHypothesisWithPose, with
      its `pose` filled from the depth-back-projected 3D position of the
      bbox centre when a valid depth is available.

The model definition and geometry-prototype helpers are imported from the
bundled `machine_vision_pkg.train_image_to_geometry` module, and the default
`.glb` asset directory is the package's installed `share/.../assets/` folder.
Both can still be overridden via ROS parameters.
"""

import json
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from PIL import Image as PILImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from machine_vision_pkg.realsense_synced_rgbd import RealsenseSyncedRGBDNode
from machine_vision_pkg.train_image_to_geometry import (
    ImageToGeometryModel,
    build_geometry_prototypes,
    build_transforms,
    discover_assets,
)


class TwoStageLiveClassifierNode(RealsenseSyncedRGBDNode):
    def __init__(self):
        super().__init__("two_stage_live_classifier")

        # --- Model + asset paths -----------------------------------------
        share_dir = Path(get_package_share_directory("machine_vision_pkg"))
        default_yolo_model = str(share_dir / "models" / "best_2.pt")
        default_stage2_model = str(share_dir / "models" / "image_to_geometry_best.pth")
        default_asset_dir = str(share_dir / "assets")
        self.declare_parameter("yolo_model_path", default_yolo_model)
        self.declare_parameter("stage2_model_path", default_stage2_model)
        self.declare_parameter("asset_dir", default_asset_dir)
        self.declare_parameter("summary_path", "")

        # --- YOLO settings -----------------------------------------------
        self.declare_parameter("yolo_conf_threshold", 0.35)
        self.declare_parameter("yolo_iou_threshold", 0.50)
        self.declare_parameter("yolo_image_size", 416)
        self.declare_parameter("yolo_device", "cpu")

        # --- Stage-2 settings --------------------------------------------
        default_stage2_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.declare_parameter("stage2_device", default_stage2_device)
        self.declare_parameter("classify_every_n_frames", 3)
        self.declare_parameter("min_crop_size_px", 20)
        self.declare_parameter("topk", 3)
        self.declare_parameter("image_size", 224)
        self.declare_parameter("backbone", "convnext_tiny")
        self.declare_parameter("projector_hidden_dim", 512)
        self.declare_parameter("projector_dropout", 0.1)
        self.declare_parameter("logit_scale", 20.0)
        self.declare_parameter("num_surface_points", 4096)
        self.declare_parameter("axis_bins", 12)
        self.declare_parameter("radial_bins", 12)
        self.declare_parameter("pairwise_bins", 12)

        # ROS 2 params do not natively carry str→str dicts, so HIGH_TO_LOW_PREFIX
        # is split into two parallel string arrays.
        self.declare_parameter("high_to_low_keys", ["gear", "nut", "screw"])
        self.declare_parameter("high_to_low_values", ["gear", "nut", "screw"])

        # --- I/O / display -----------------------------------------------
        self.declare_parameter("show_window", True)
        self.declare_parameter("show_camera_view_as_captured", True)
        self.declare_parameter("detections_topic", "/classifier/detections")

        # --- Temporal-consistency filter ---------------------------------
        self.declare_parameter("confirm_frames", 10)
        self.declare_parameter("track_match_distance_px", 40.0)

        # --- Read everything ---------------------------------------------
        self.yolo_model_path = str(self.get_parameter("yolo_model_path").value)
        self.stage2_model_path = str(self.get_parameter("stage2_model_path").value)
        self.asset_dir = str(self.get_parameter("asset_dir").value)
        self.summary_path = str(self.get_parameter("summary_path").value)

        self.yolo_conf_threshold = float(self.get_parameter("yolo_conf_threshold").value)
        self.yolo_iou_threshold = float(self.get_parameter("yolo_iou_threshold").value)
        self.yolo_image_size = int(self.get_parameter("yolo_image_size").value)
        self.yolo_device = str(self.get_parameter("yolo_device").value)

        requested_stage2_device = str(self.get_parameter("stage2_device").value).lower()
        if requested_stage2_device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn(
                f"stage2_device='{requested_stage2_device}' requested but CUDA is "
                "unavailable on this machine; falling back to CPU."
            )
            self.stage2_device = "cpu"
        else:
            self.stage2_device = requested_stage2_device
        self.classify_every_n_frames = int(self.get_parameter("classify_every_n_frames").value)
        self.min_crop_size_px = int(self.get_parameter("min_crop_size_px").value)
        self.topk = int(self.get_parameter("topk").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.backbone = str(self.get_parameter("backbone").value)
        self.projector_hidden_dim = int(self.get_parameter("projector_hidden_dim").value)
        self.projector_dropout = float(self.get_parameter("projector_dropout").value)
        self.logit_scale = float(self.get_parameter("logit_scale").value)
        self.num_surface_points = int(self.get_parameter("num_surface_points").value)
        self.axis_bins = int(self.get_parameter("axis_bins").value)
        self.radial_bins = int(self.get_parameter("radial_bins").value)
        self.pairwise_bins = int(self.get_parameter("pairwise_bins").value)

        keys = list(self.get_parameter("high_to_low_keys").value)
        values = list(self.get_parameter("high_to_low_values").value)
        self.high_to_low_prefix = dict(zip(keys, values))

        self.show_window = bool(self.get_parameter("show_window").value)
        self.show_as_captured = bool(self.get_parameter("show_camera_view_as_captured").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        self.confirm_frames = int(self.get_parameter("confirm_frames").value)
        self.track_match_distance_px = float(self.get_parameter("track_match_distance_px").value)

        # --- Path validation --------------------------------------------
        if not os.path.isfile(self.yolo_model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.yolo_model_path}")
        if not os.path.isfile(self.stage2_model_path):
            raise FileNotFoundError(f"Stage-2 model not found: {self.stage2_model_path}")
        if not os.path.isdir(self.asset_dir):
            raise FileNotFoundError(f"Asset directory not found: {self.asset_dir}")

        # --- Optional summary JSON overrides ----------------------------
        self._load_optional_summary()

        self.device = torch.device(self.stage2_device)
        self.get_logger().info(
            f"Stage-2 device: {self.device}, YOLO device: {self.yolo_device}"
        )

        # --- Build Stage-2 model + prototypes ---------------------------
        self.class_names = sorted(discover_assets(self.asset_dir).keys())
        self.get_logger().info(f"Low-level classes: {self.class_names}")

        self.allowed_low_level_indices = self._build_allowed_low_level_indices(
            self.class_names, self.high_to_low_prefix
        )
        for high_label, indices in self.allowed_low_level_indices.items():
            names = [self.class_names[i] for i in indices]
            self.get_logger().info(f"  {high_label} -> {names}")

        self.geometry_prototypes = build_geometry_prototypes(
            asset_dir=self.asset_dir,
            canonical_classes=self.class_names,
            num_surface_points=self.num_surface_points,
            axis_bins=self.axis_bins,
            radial_bins=self.radial_bins,
            pairwise_bins=self.pairwise_bins,
            device=torch.device("cpu"),
        ).to(self.device)

        geometry_dim = int(self.geometry_prototypes.shape[1])
        self.stage2_model = ImageToGeometryModel(
            backbone=self.backbone,
            geometry_dim=geometry_dim,
            projector_hidden_dim=self.projector_hidden_dim,
            dropout=self.projector_dropout,
        )
        state_dict = torch.load(self.stage2_model_path, map_location="cpu")
        self.stage2_model.load_state_dict(state_dict)
        self.stage2_model = self.stage2_model.to(self.device).eval()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        _, self.eval_tf = build_transforms(self.image_size, strong_aug=False)
        self.get_logger().info("Stage-2 model loaded.")

        # --- Build Stage-1 (YOLO) ---------------------------------------
        self.yolo_model = YOLO(self.yolo_model_path)
        self.yolo_model.to(self.yolo_device)
        try:
            self.yolo_model.fuse()
        except Exception:
            pass
        self.get_logger().info(f"YOLO classes: {self.yolo_model.names}")

        # --- Publisher + state ------------------------------------------
        self.det_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.frame_count = 0
        self.latest_detections = []
        self.tracks = []  # list of {"cx", "cy", "high_label", "hits", "det"}

        # On/off gate. When `active` is False, process() returns immediately,
        # skipping YOLO + Stage-2 inference. Toggle via std_srvs/SetBool on
        # ~/set_active (fully-qualified: /two_stage_live_classifier/set_active).
        self.active = True
        self.create_service(SetBool, "~/set_active", self._on_set_active)

        self.get_logger().info("Two-stage live classifier node ready.")

    def _on_set_active(self, request, response):
        new_active = bool(request.data)
        # On a False -> True edge, wipe tracker state so the confirm_frames
        # counter starts fresh every time we re-enable.
        if new_active and not self.active:
            self.tracks = []
            self.latest_detections = []
        self.active = new_active
        response.success = True
        response.message = f"classifier {'active' if new_active else 'inactive'}"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ helpers
    def _load_optional_summary(self) -> None:
        if not self.summary_path:
            return
        if not os.path.isfile(self.summary_path):
            self.get_logger().warn(f"Summary JSON not found: {self.summary_path}")
            return
        with open(self.summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
        self.backbone = summary.get("backbone", self.backbone)
        self.projector_hidden_dim = summary.get(
            "projector_hidden_dim", self.projector_hidden_dim
        )
        self.projector_dropout = summary.get(
            "projector_dropout", self.projector_dropout
        )
        self.image_size = summary.get("image_size", self.image_size)

    @staticmethod
    def _normalize_yolo_label(label: str) -> str:
        label = label.lower().strip()
        if label.endswith("s"):
            label = label[:-1]
        return label

    @staticmethod
    def _build_allowed_low_level_indices(class_names, high_to_low_prefix):
        allowed = {}
        for high_label, prefix in high_to_low_prefix.items():
            prefix_token = prefix + "_"
            indices = [
                i
                for i, name in enumerate(class_names)
                if name.lower().strip() == prefix
                or name.lower().strip().startswith(prefix_token)
            ]
            allowed[high_label] = indices
        return allowed

    @staticmethod
    def _crop_bbox(image_bgr, xyxy):
        h, w = image_bgr.shape[:2]
        x1, y1, x2, y2 = xyxy
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w))
        y2 = max(0, min(y2, h))
        if x2 <= x1 or y2 <= y1:
            return None
        return image_bgr[y1:y2, x1:x2].copy()

    def _classify_crop(self, crop_bgr, allowed_indices):
        if crop_bgr is None:
            return "invalid_crop", 0.0, []

        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(crop_rgb)
        image_tensor = self.eval_tf(pil_image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            embedding = self.stage2_model(image_tensor)
            logits = self.logit_scale * (embedding @ self.geometry_prototypes.t())
            probs = torch.softmax(logits, dim=1)[0]
            scores = logits[0]

        if allowed_indices:
            allowed_tensor = torch.tensor(
                allowed_indices, dtype=torch.long, device=self.device
            )
            allowed_logits = logits[0, allowed_tensor]
            allowed_probs = torch.softmax(allowed_logits, dim=0)
            topk = max(1, min(self.topk, len(allowed_indices)))
            top_values, top_local_indices = allowed_probs.topk(topk)
            top_predictions = []
            for prob, local_idx in zip(top_values.tolist(), top_local_indices.tolist()):
                global_idx = allowed_indices[local_idx]
                top_predictions.append(
                    (self.class_names[global_idx], prob, scores[global_idx].item())
                )
            return top_predictions[0][0], top_predictions[0][1], top_predictions

        topk = max(1, min(self.topk, len(self.class_names)))
        top_values, top_indices = probs.topk(topk)
        top_predictions = [
            (self.class_names[idx], prob, scores[idx].item())
            for prob, idx in zip(top_values.tolist(), top_indices.tolist())
        ]
        return top_predictions[0][0], top_predictions[0][1], top_predictions

    @staticmethod
    def _backproject(u, v, z_m, K):
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]
        if fx == 0.0 or fy == 0.0:
            return None
        x = (u - cx) * z_m / fx
        y = (v - cy) * z_m / fy
        return float(x), float(y), float(z_m)

    def _update_tracks(self, raw_detections):
          import math
          survivors = []
          matched = [False] * len(self.tracks)
          for det in raw_detections:
              x1, y1, x2, y2 = det["xyxy"]
              cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
              best_ti, best_d = -1, self.track_match_distance_px
              for ti, tr in enumerate(self.tracks):
                  if matched[ti] or tr["high_label"] != det["high_label"]:
                      continue
                  d = math.hypot(cx - tr["cx"], cy - tr["cy"])
                  if d <= best_d:
                      best_ti, best_d = ti, d
              if best_ti >= 0:
                  matched[best_ti] = True
                  tr = self.tracks[best_ti]
                  tr["cx"], tr["cy"], tr["det"] = cx, cy, det
                  tr["hits"] += 1
                  survivors.append(tr)
              else:
                  survivors.append({
                      "cx": cx, "cy": cy,
                      "high_label": det["high_label"],
                      "hits": 1, "det": det,
                  })
          self.tracks = survivors   # unmatched old tracks dropped → counter resets to 0
          
    # ------------------------------------------------------------------ main loop
    def process(
        self, color_bgr: np.ndarray, depth_meters: np.ndarray, camera_info: CameraInfo
    ) -> None:
        if not self.active:
            return
        inference_image = color_bgr
        display_image = color_bgr.copy()
        if not self.show_as_captured:
            display_image = cv2.flip(display_image, 1)

        if self.frame_count % self.classify_every_n_frames == 0:
            raw_detections = []

            yolo_results = self.yolo_model.predict(
                source=inference_image,
                imgsz=self.yolo_image_size,
                conf=self.yolo_conf_threshold,
                iou=self.yolo_iou_threshold,
                agnostic_nms=True,
                device=self.yolo_device,
                verbose=False,
            )
            result = yolo_results[0]
            boxes = result.boxes

            if boxes is not None and len(boxes) > 0:
                K = list(camera_info.k)
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    yolo_conf = float(box.conf[0].item())
                    high_label = self._normalize_yolo_label(self.yolo_model.names[cls_id])
                    x1, y1, x2, y2 = (box.xyxy[0].detach().cpu().numpy().astype(int).tolist())
                    if (x2 - x1) < self.min_crop_size_px or (y2 - y1) < self.min_crop_size_px:
                        continue
                    crop = self._crop_bbox(inference_image, (x1, y1, x2, y2))
                    allowed_indices = self.allowed_low_level_indices.get(high_label, [])
                    low_label, low_conf, _ = self._classify_crop(crop, allowed_indices)

                    cu = (x1 + x2) // 2
                    cv_ = (y1 + y2) // 2
                    pos_3d = None
                    if 0 <= cv_ < depth_meters.shape[0] and 0 <= cu < depth_meters.shape[1]:
                        z = float(depth_meters[cv_, cu])
                        if np.isfinite(z) and z > 0.0:
                            pos_3d = self._backproject(cu, cv_, z, K)

                    raw_detections.append(
                        {
                            "xyxy": (x1, y1, x2, y2),
                            "high_label": high_label,
                            "high_conf": yolo_conf,
                            "low_label": low_label,
                            "low_conf": low_conf,
                            "pos_3d": pos_3d,
                        }
                    )
            
            self._update_tracks(raw_detections)
            self.latest_detections = [t["det"] for t in self.tracks if t["hits"] >= self.confirm_frames]
            self._publish_detections(camera_info.header)

        self.frame_count += 1

        if self.show_window:
            self._draw_and_show(display_image)

    def _publish_detections(self, header) -> None:
        msg = Detection2DArray()
        msg.header = header
        for det in self.latest_detections:
            d = Detection2D()
            d.header = header
            x1, y1, x2, y2 = det["xyxy"]
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)

            h_hyp = ObjectHypothesisWithPose()
            h_hyp.hypothesis.class_id = det["high_label"]
            h_hyp.hypothesis.score = det["high_conf"]
            d.results.append(h_hyp)

            l_hyp = ObjectHypothesisWithPose()
            l_hyp.hypothesis.class_id = det["low_label"]
            l_hyp.hypothesis.score = det["low_conf"]
            if det["pos_3d"] is not None:
                x, y, z = det["pos_3d"]
                l_hyp.pose.pose.position.x = x
                l_hyp.pose.pose.position.y = y
                l_hyp.pose.pose.position.z = z
                l_hyp.pose.pose.orientation.w = 1.0
            d.results.append(l_hyp)

            d.id = f"{det['high_label']}/{det['low_label']}"
            msg.detections.append(d)
        self.det_pub.publish(msg)

    def _draw_and_show(self, display_image) -> None:
        for det in self.latest_detections:
            x1, y1, x2, y2 = det["xyxy"]
            high_label = det["high_label"]
            high_conf = det["high_conf"]
            low_label = det["low_label"]
            low_conf = det["low_conf"]

            if not self.show_as_captured:
                disp_h, disp_w = display_image.shape[:2]
                x1_disp = disp_w - x2
                x2_disp = disp_w - x1
                y1_disp, y2_disp = y1, y2
            else:
                x1_disp, y1_disp, x2_disp, y2_disp = x1, y1, x2, y2

            cv2.rectangle(
                display_image, (x1_disp, y1_disp), (x2_disp, y2_disp), (0, 255, 0), 2
            )
            label_text_1 = f"{high_label} ({high_conf:.2f})"
            label_text_2 = f"{low_label} ({low_conf:.2f})"
            text_x = x1_disp
            text_y = max(25, y1_disp - 10)
            cv2.putText(
                display_image, label_text_1, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1,
            )
            cv2.putText(
                display_image, label_text_2, (text_x, text_y + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1,
            )

        # Camera is mounted upside-down; rotate the final composite (image +
        # overlays) 180° purely for viewing. Inference and published bbox
        # coordinates remain in the raw sensor frame.
        display_image = cv2.rotate(display_image, cv2.ROTATE_180)
        cv2.imshow("Two-stage YOLOv8-seg + Geometry Classification", display_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("q pressed. Shutting down.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TwoStageLiveClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
