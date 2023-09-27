"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
import matplotlib.pyplot as plt
import landMark as lm 

class GridOccupancyMap(object):
    """

    """
    def __init__(self, low=(-50, 0), high=(50, 100), res=1) -> None:
        self.map_area = [low, high]    #a rectangular area    
        self.map_size = np.array([high[0]-low[0], high[1]-low[1]])
        self.resolution = res

        self.n_grids = [ int(s//res) for s in self.map_size]

        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]), dtype=np.uint8)

        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    def in_collision(self, pos):
        """
        find if the position is occupied or not. return if the queried pos is outside the map
        """
        indices = [int((pos[i] - self.map_area[0][i]) // self.resolution) for i in range(2)]
        for i, ind in enumerate(indices):
            if ind < 0 or ind >= self.n_grids[i]:
                return 1
        
        return self.grid[indices[0], indices[1]] 

    def populate(self):
        """
        generate a grid map with some circle shaped obstacles
        """
        results, ids = lm.lookBox(-1)

        origins = []
        radius = []

        print(f"results: {results}")

        for i in range(0, len(results)):
            radians = results[i][0][0]
            degrees = np.degrees(radians) + lm.angle_error
            if (degrees < 0):
                x = int(results[i][0][2]*100 * np.sin(radians + np.deg2rad(lm.angle_error)))
            else:
                x = int(results[i][0][2]*100 *
                        np.sin(radians + np.deg2rad(lm.angle_error)))
            y = int(results[i][0][2]*100)
            origins.append([x, y])
            radius.append(0.35)

        #fill the grids by checking if the grid centroid is in any of the circle
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array([self.map_area[0][0] + self.resolution * (i+0.5), 
                                     self.map_area[0][1] + self.resolution * (j+0.5)])
                for o, r in zip(origins, radius):
                    if np.linalg.norm(centroid - o) <= r:
                        self.grid[i, j] = 1
                        break

    
    def draw_map(self):
        #note the x-y axes difference between imshow and plot
        plt.imshow(self.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=100, extent=self.extent, interpolation='none')

if __name__ == '__main__':
    map = GridOccupancyMap()
    map.populate()

    plt.clf()
    map.draw_map()
    plt.savefig("map.png")

        