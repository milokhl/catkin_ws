# This represents a list of detections for an image

Header header
uint32 img_id # Image that detections came from.
string[] classes # Stores the string label for each detection
float32[] scores # Stores confidence for each detection in [0, 1]
BoundingBox[] boxes # Stores x1, x2, y1, y2