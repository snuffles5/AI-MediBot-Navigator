from shapely.geometry import LineString, Polygon
from matplotlib.patches import Circle


def create_obstacles():
    # Define obstacles here using shapely
    return [
        # ROOM
        LineString([(0, 0), (0, 234)]),
        LineString([(0, 331), (0, 622)]),
        LineString([(0, 622), (594, 622)]),
        LineString([(594, 622), (594, 0)]),
        LineString([(594, 0), (225, 0)]),
        LineString([(184, 0), (0, 0)]),

        # N
        LineString([(0, 0), (0, 97)]),
        LineString([(0, 97), (133, 97)]),
        LineString([(133, 97), (133, 79)]),
        LineString([(133, 60), (133, 47)]),
        LineString([(133, 24), (133, 0)]),
        LineString([(133, 0), (0, 0)]),

        # F
        LineString([(0, 97), (0, 234)]),
        LineString([(0, 234), (106, 234)]),
        LineString([(127, 234), (150, 234)]),
        LineString([(150, 234), (150, 97)]),
        LineString([(150, 97), (0, 97)]),
        # A
        LineString([(0, 331), (0, 491)]),
        LineString([(0, 491), (120, 490)]),
        LineString([(120, 490), (120, 364)]),
        LineString([(120, 344), (120, 331)]),
        LineString([(120, 331), (0, 331)]),
        LineString([(0, 331), (120, 331)]),
        LineString([(120, 331), (157, 331)]),
        # C
        LineString([(157, 388), (157, 490)]),
        LineString([(157, 490), (265, 490)]),
        LineString([(265, 490), (265, 388)]),
        LineString([(265, 388), (226, 388)]),
        LineString([(204, 388), (157, 388)]),
        # D
        LineString([(265, 388), (265, 490)]),
        LineString([(265, 490), (374, 490)]),
        LineString([(374, 490), (374, 382)]),
        LineString([(374, 382), (374, 388)]),
        LineString([(374, 388), (311, 388)]),
        LineString([(291, 388), (265, 388)]),
        # E
        LineString([(374, 324), (374, 362)]),
        LineString([(374, 382), (374, 490)]),
        LineString([(374, 490), (594, 490)]),
        LineString([(594, 490), (594, 324)]),
        LineString([(594, 324), (374, 324)]),
        # B
        LineString([(60, 530), (60, 622)]),
        LineString([(60, 622), (215, 622)]),
        LineString([(215, 622), (215, 530)]),
        LineString([(215, 530), (150, 530)]),
        LineString([(130, 530), (60, 530)]),
        # O
        LineString([(261, 322), (261, 342)]),
        LineString([(277, 342), (348, 342)]),
        LineString([(348, 342), (348, 322)]),
        LineString([(348, 322), (261, 322)]),

        # J
        LineString([(490, 167), (490, 192)]),
        LineString([(490, 212), (490, 234)]),
        LineString([(490, 234), (594, 234)]),
        LineString([(594, 234), (594, 167)]),
        LineString([(594, 167), (490, 167)]),

        # K
        LineString([(490, 73), (490, 123)]),
        LineString([(490, 141), (490, 167)]),
        LineString([(490, 167), (594, 167)]),
        LineString([(594, 167), (594, 73)]),
        LineString([(594, 73), (490, 73)]),

        # L
        LineString([(460, 0), (460, 73)]),
        LineString([(460, 73), (465, 73)]),
        LineString([(485, 73), (594, 73)]),
        LineString([(594, 73), (594, 0)]),
        LineString([(594, 0), (460, 0)]),

        # M
        LineString([(380, 0), (380, 73)]),
        LineString([(380, 73), (439, 73)]),
        LineString([(456, 73), (460, 73)]),
        LineString([(460, 73), (460, 0)]),
        LineString([(460, 0), (380, 0)]),

        # I
        LineString([(282, 0), (282, 234)]),
        LineString([(282, 234), (357, 234)]),
        LineString([(377, 234), (456, 234)]),
        LineString([(456, 234), (456, 101)]),
        LineString([(456, 101), (380, 101)]),
        LineString([(380, 101), (380, 0)]),
        LineString([(380, 0), (282, 0)]),

        # G
        LineString([(242, 173), (242, 214)]),
        LineString([(242, 234), (282, 234)]),
        LineString([(282, 234), (282, 173)]),
        LineString([(282, 173), (277, 173)]),
        LineString([(257, 173), (242, 173)]),

        # H
        LineString([(242, 101), (242, 173)]),
        LineString([(242, 173), (257, 173)]),
        LineString([(277, 173), (282, 173)]),
        LineString([(282, 173), (282, 101)]),
        LineString([(282, 101), (277, 101)]),

        # ...
        LineString([(225, 0), (225, 62)]),
        Circle((25, 509), 10),
        Circle((240, 308), 10),
        Circle((93, 244), 10),
        Polygon([(142, 388), (142, 422), (157, 421), (157, 388), ]),
        Polygon([(79, 280), (79, 304), (153, 304), (154, 280), ]),
        Polygon([(178, 371), (178, 383), (203, 383), (203, 371), ]),
        Polygon([(297, 305), (297, 322), (345, 322), (345, 305), ]),
        Polygon([(383, 305), (383, 322), (432, 322), (432, 305), ]),
        Polygon([(460, 184), (475, 184), (475, 150), (460, 150), ]),
        Polygon([(407, 73), (407, 93), (439, 93), (439, 73), ]),
    ]
