import fitz  # PyMuPDF to read the PDF
import json
import re



pattern = re.compile(r"(?P<name>[^\n]+)\s*\((?P<abbr>[A-Z]+)\)\s*\n*\s*\((?P<type>[PAL,]+)\)", re.MULTILINE)
# Define a helper function to extract information
def extract_geo_objects(page_text):
	# Find all matches
	matches = pattern.finditer(page_text)
	objs = []
	# Print results
	#print("Detected Geo Objects:")
	found_any = False
	for match in matches:
		found_any = True
		name = match.group("name").strip()
		abbr = match.group("abbr")
		types = match.group("type")
		ts = types.split(",")
		# print("Name:", name)
		# print("Abbreviation:", abbr)
		# print("Type: (", types, ")")
		# print("====================================")
		obj = {
			"name": name,
			"abbr": abbr,
			"type": sorted(ts)
		}
		objs.append(obj)
	return objs


	# if not found_any:
	# 	print("No geo objects found.")
			
def test_page(page_text):
	print(page_text)


def parse_object(obj):
	nobj = {
		"area":False,
	   "desc":"N/A",
	   "line":False,
	   "point":False
	   }

	if "A" in obj["type"]:
		nobj["area"] = True
	if "L" in obj["type"]:
		nobj["line"] = True
	if "P" in obj["type"]:
		nobj["point"] = True
	nobj["desc"] = obj["name"]
	return obj["abbr"], nobj
	
	
	
	
# Loop through each page and extract information

# Initialize a dictionary to store geo objects data
geo_objects = {}
# Open the PDF document
out_file = open("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/geo_objects.txt", "w")
pdf_path = '/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/S-57 Appendix B.1 Annex A UOC Edition 4.1.0_Jan18_EN.pdf'
with fitz.open(pdf_path) as doc:  # open document
	#for page in doc:
	# geoobj and objs
		
	#text = chr(12).join([page.get_text() for page in doc])
	for page_num in range(doc.page_count):
		#if 47 < page_num < 50:
		page = doc.load_page(page_num)
		page_text = page.get_text("text")
		if "Geo objects:" in page_text or "Geo object:" in page_text:
			#g_ind = page_text.index("Geo objects:")
			#print("Page num:", page_num)
			# print(page_text)
			objs = extract_geo_objects(page_text)
			
			for obj in objs:
				n, no = parse_object(obj)
				geo_objects[n] = no
				print(obj["abbr"], obj["name"])

			# print("====================================")
		#extract_geo_objects(page_text)
		#test_page(page_text)
		#break
	sorted_geo_objects = dict(sorted(geo_objects.items(), key=lambda x: x[0]))
	jd = json.dumps(sorted_geo_objects, indent=4)
	out_file.write(jd)

# Output the result to a JSON file
# json_output = '/mnt/data/geo_objects.json'
# with open(json_output, 'w') as json_file:
#     json.dump(geo_objects, json_file, indent=4)


