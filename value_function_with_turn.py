import numpy as np

grid = [[0, 1, 0, 0, 0, 0],
    	[0, 1, 1, 0, 0, 0],
    	[0, 0, 0, 0, 0, 0],
    	[0, 1, 1, 1, 0, 0],
    	[0, 1, 0, 1, 0, 0]]

value = [[[999 for i in row] for row in grid], 
		 [[999 for i in row] for row in grid], 
		 [[999 for i in row] for row in grid], 
		 [[999 for i in row] for row in grid]]

print(np.array(value))
print(np.array(value).shape)