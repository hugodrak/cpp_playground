
# NOTE: This is a script to generate the npys for the path planning
# NOTE: i think the approach should be that we should stick to coordninates for calculateing. to improve the accuracy
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt
def is_point_in_land_area(point, land_polygons):
	# this will check the center points for the boxes
	# TODO: also check the corners for edge cases
	for polygon in land_polygons:
		if polygon.contains(point):
			return True
	return False

def box_in_poly(box, poly):
	# box is a list of 4 points
	# poly is a list of points
	# returns True if the box is in the poly
	# returns False if the box is not
	cx = (box[0].x + box[1].x)/2 # center
	cy = (box[0].y + box[1].y)/2 # center
	point = Point(cx, cy)
	center_in_poly = poly.contains(point)
	if not center_in_poly:
		bottom_left = Point(box[0].x, box[0].y)
		top_right = Point(box[1].x, box[1].y)
		bottom_right = Point(box[1].x, box[0].y)
		top_left = Point(box[0].x, box[1].y)
		
		corners_in_poly = poly.contains(bottom_left) and poly.contains(top_right) and poly.contains(bottom_right) and poly.contains(top_left)
		return corners_in_poly
	return center_in_poly


def plot_box_poly(box, poly):
	bottom_left = Point(box[0].x, box[0].y)
	top_right = Point(box[1].x, box[1].y)
	bottom_right = Point(box[1].x, box[0].y)
	top_left = Point(box[0].x, box[1].y)
	xs = [bottom_left.x, bottom_right.x, top_right.x, top_left.x, bottom_left.x]
	ys = [bottom_left.y, bottom_right.y, top_right.y, top_left.y, bottom_left.y]
	plt.plot(xs, ys, c="red")
	px,py = poly.exterior.xy
	plt.plot(px, py, c="blue")

	plt.show()
	



def main():
	grid_size = 5.0 # meters h and w for all the boxes
	box = [Point(1,1), Point(2,2)] # bottom_left, top_right

	poly = Polygon([(0,0), (0,3), (3,3), (3,0)])

	res = box_in_poly(box, poly)

	print(res)
	plot_box_poly(box, poly)


if __name__ == "__main__":
	main()