import numpy


class DiscreteEnvironment(object):
    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension * [0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx]) / resolution[idx])
        # print self.num_cells


    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0
        coord = self.ConfigurationToGridCoord(config)
        # print coord
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        coord = self.NodeIdToGridCoord(nid)
        # print coord
        config = self.GridCoordToConfiguration(coord)
        return numpy.array(config)

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        # for i  in range (self.dimension):
        coord = numpy.floor((config - self.lower_limits) / self.resolution)

        return numpy.array(coord)

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        config = (coord + 0.5) * self.resolution + self.lower_limits
        return numpy.array(config)

    def GridCoordToNodeId(self, coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        node_id = 0
        mul = [1]*len(coord)
        for m in range (1,len(coord)):
            mul[m] = self.num_cells[m-1] * mul[m-1]
        for i in range(len(coord)):
            node_id = node_id + coord[i]*mul[i]
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        mul = [1] * len(coord)
        # n = numpy.floor((self.upper_limits - self.lower_limits) / self.resolution)
        for m in range (1,len(coord)):
            mul[m] = self.num_cells[m-1] * mul[m-1]
        for i in range(self.dimension - 1, -1, -1):
            coord[i] = numpy.floor(node_id / mul[i])
            node_id = numpy.mod(node_id, mul[i])

        return numpy.array(coord)
