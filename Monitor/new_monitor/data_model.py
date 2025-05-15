from dataclasses import dataclass
from typing import Dict, Any, List, Optional
from PySide6.QtGui import QImage

@dataclass
class MonitorData:
    stdout_lines: Optional[List[str]] = None    # for RollingStdout
    cv2_frame: Optional[Any] = None             # numpy array for VideoCV2
    telemetry: Optional[Dict[str, Any]] = None  # key→value text for TelemetryPanel
    map_image: Optional[Any] = None             # QImage or numpy array for MapPanel
    bombs_count: Optional[int] = 0              # integer count for BombPanel