from generate_npys import is_point_in_land_area, box_in_poly
from shapely.geometry import Point, Polygon

def test_box_in_poly():
	box = [Point(1,1), Point(2,2)] # bottom_left, top_right

	poly = Polygon([(0,0), (0,3), (3,3), (3,0)])

	res = box_in_poly(box, poly)

	assert res