# Single 2D vision detection from the camera/YOLO pipeline
string class_name
float32 confidence
float32[4] bbox        # x, y, width, height (pixels)
int32 track_id          # -1 if not yet tracked
