import re


text = open("path_planning/work.txt", "r").read()
#text = "Geo object: Harbour facility (HRBFAC) (P,A)"
pattern = re.compile(r"Geo object:\s*(?P<name>[^\(]+)\s*\((?P<abbr>[A-Z]+)\)\s*\((?P<types>[PAL,]+)\)")
#pattern = re.compile(r"Geo object:\s*(?P<name>[^\(]+)\s*\((?P<abbr>[A-Z]+)\)\s*\((?P<types>[PAL,]+)\)\s*Attributes:\s*\n(?P<attributes>(?:\s*\b\w+\b\s*\n)+)Remarks:")

matches = pattern.findall(text)


# single geo obj
for match in matches:
	name = match[0].strip()
	abbr = match[1]
	types = match[2].split(",")
	# attributes = []
	# for x in match[3].split("\n"):
	# 	xs = x.rstrip(" ")
	# 	if xs != "":
	# 		attributes.append(xs)
	# #attributes = [x.rstrip(" ") for x in match[3].split("\n") if x != ""]
	print("Name:", name)
	print("Abbreviation:", abbr)
	print("Types:", types)
	# print("Attributes:", attributes)
	print("====================================")
	print("Name:", match[0].strip())
	print("Abbreviation:", match[1])
	print("Types:", match[2])
	print("Attributes:", match[3].split("\n"))
	print("====================================")

# print("multi objs")
# pattern2 = re.compile(r"Geo objects:\s*(?P<name>[^\(]+)\s*\((?P<abbr>[A-Z]+)\)\s*\((?P<types>[PAL,]+)\)")
# matches2 = pattern2.findall(text)

# for match in matches2:
# 	name = match[0].strip()
# 	abbr = match[1]
	types = match[2].split(",")
# 	print("Name:", name)
# 	print("Abbreviation:", abbr)
# 	print("Types:", types)
# 	# print("Attributes:", attributes)
# 	print("====================================")


# Regex pattern to match individual geo objects
pattern = re.compile(r"(?P<name>[^\n]+)\s*\((?P<abbr>[A-Z]+)\)\s*\n*\s*\((?P<type>[PAL,]+)\)", re.MULTILINE)

# Find all matches
matches = pattern.finditer(text)

# Print results
print("Detected Geo Objects:")
found_any = False
for match in matches:
    found_any = True
    name = match.group("name").strip()
    abbr = match.group("abbr")
    types = match.group("type")
    print("Name:", name)
    print("Abbreviation:", abbr)
    print("Type: (", types, ")")
    print("====================================")

if not found_any:
    print("No geo objects found.")