import random
se = 4.9000 # Standard error that you can reciveable
#se = 2.9

goal = -290 # Range of goal value

if goal < 0: # Dir in range
	dir = -1
else:
	dir = 1

min_error = goal - se * dir
max_error = goal + se * dir

count = 10 # Number of testing

for i in range(0,count):
	print(random.uniform(min_error, max_error)) # Random process
