from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np


@dataclass
class ArucoInfo:
    id: int
    center: np.ndarray
    corners: np.ndarray


@dataclass
class ArucoInfoList:
    infos: List[ArucoInfo]

    def sort_by_target_id(self) -> "ArucoInfoList":
        # validate all target IDs are present
        for target_id in TARGET_IDS:
            if not any(info.id == target_id for info in self.infos):
                raise RuntimeError(
                    f"Missing ArUco id {target_id}. Found {[info.id for info in self.infos]}"
                )

        sorted_infos = [
            next(info for info in self.infos if info.id == target_id)
            for target_id in TARGET_IDS
        ]
        return ArucoInfoList(infos=sorted_infos)

    def __iter__(self):
        return iter(self.infos)

    def __getitem__(self, index: int) -> ArucoInfo:
        return self.infos[index]

    def __len__(self) -> int:
        return len(self.infos)

    def get_by_id(self, id_: int) -> Optional[ArucoInfo]:
        for info in self.infos:
            if info.id == id_:
                return info
        return None


@dataclass(frozen=True)
class DetectionConfig:
    """Overrides for cv2.aruco.DetectorParameters."""

    params: Dict[str, float]


@dataclass
class DetectionResult:
    ids: Optional[np.ndarray]  # shape (N,) or None
    corners_list: Optional[Sequence[np.ndarray]]  # list of (4,2) arrays


TARGET_IDS = (0, 1, 2, 3)

# Parameter trials to improve robustness
DETECTION_CONFIGS: List[DetectionConfig] = [
    DetectionConfig(params={}),  # default
    DetectionConfig(params={"adaptiveThreshConstant": 5}),
    DetectionConfig(params={"adaptiveThreshConstant": 10}),
    DetectionConfig(params={"minMarkerPerimeterRate": 0.02}),
    DetectionConfig(
        params={"minMarkerPerimeterRate": 0.02, "adaptiveThreshConstant": 10}
    ),
]


def run_detection_trial(
    gray: np.ndarray,
    dictionary: cv2.aruco.Dictionary,
    config: DetectionConfig,
) -> DetectionResult:
    """Run one detection attempt with a specific DetectorParameters config."""
    aruco = cv2.aruco
    params = aruco.DetectorParameters()

    for key, value in config.params.items():
        setattr(params, key, value)

    try:
        detector = aruco.ArucoDetector(dictionary, params)
        corners_list, ids, _ = detector.detectMarkers(gray)
    except Exception:
        # Fallback for older OpenCV
        corners_list, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)

    if ids is not None:
        ids = ids.flatten()

    return DetectionResult(ids=ids, corners_list=corners_list)


def score_ids(ids: Optional[np.ndarray]) -> int:
    """Score a detection by how many target IDs are present."""
    if ids is None:
        return 0
    return sum(1 for i in ids if int(i) in TARGET_IDS)


def pick_best_detection(
    gray: np.ndarray,
    dictionary: cv2.aruco.Dictionary,
) -> DetectionResult:
    """Try multiple configurations and return the best detection result."""
    best_result: DetectionResult = DetectionResult(ids=None, corners_list=None)
    best_score: int = -1

    for config in DETECTION_CONFIGS:
        result = run_detection_trial(gray, dictionary, config)
        score = score_ids(result.ids)

        if score > best_score:
            best_score = score
            best_result = result

        if score == len(TARGET_IDS):
            # Early exit: we found all markers we care about
            break

    return best_result


def build_aruco_info_list(result: DetectionResult) -> ArucoInfoList:
    """Convert detection result into the output info dict."""
    info_list: ArucoInfoList = ArucoInfoList(infos=[])

    if result.ids is None or result.corners_list is None:
        return info_list

    for corners, marker_id in zip(result.corners_list, result.ids):
        marker_id_int = int(marker_id)
        if marker_id_int not in TARGET_IDS:
            continue

        pts: np.ndarray = corners.reshape(4, 2).astype(np.float32)
        center: np.ndarray = pts.mean(axis=0)
        info_list.infos.append(ArucoInfo(id=marker_id_int, center=center, corners=pts))

    return info_list


def detect_aruco_markers(image_bgr: np.ndarray) -> ArucoInfoList:
    """
    Detect ArUco markers with IDs in TARGET_IDS using DICT_ARUCO_ORIGINAL.

    Tries multiple DetectorParameters configurations for robustness and
    returns a list of `ArucoInfo` objects.
    """
    aruco = cv2.aruco
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

    best_result = pick_best_detection(gray, dictionary)
    return build_aruco_info_list(best_result)


def get_aruco_ids(aruco_infos: ArucoInfoList) -> List[int]:
    return [info.id for info in aruco_infos]


def get_missing_target_ids(aruco_infos: ArucoInfoList) -> List[int]:
    ids = set(get_aruco_ids(aruco_infos))

    return [target_id for target_id in TARGET_IDS if target_id not in ids]
