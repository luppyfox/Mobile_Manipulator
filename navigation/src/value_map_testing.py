import random
se = 4.9000
#se = 2.9
goal = -290
if goal < 0:
	dir = -1
else:
	dir = 1
min_error = goal - se * dir
max_error = goal + se * dir
count = 10
for i in range(0,count):
	print(random.uniform(min_error, max_error))
