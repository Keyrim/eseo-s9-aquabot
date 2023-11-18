from enum import IntEnum

class Source(IntEnum):
    LIDAR = 1
    CAMERA = 2
    ACOUSTIC = 3

# class SearchedObjectInfo:
#     def __init__(self, source: Source, found: bool, angle: float, _range: float):
#         self.source = source
#         self.found = found
#         self.angle = angle
#         self.range = _range