import sys
import numpy as np


class EdgeInfo:
    _start_point: np.array = None
    _end_point: np.array = None
    _edge_vector: np.array = None

    def __init__(self, **kwargs):
        if "start_point" in kwargs:
            self.start_point = kwargs.get("start_point")
        if "end_point" in kwargs:    
            self.end_point = kwargs.get("end_point")

        self.calculate_edge_vector()

    def __str__(self):
        output: str = ""
        if self._start_point is not None:
            output = output + "Start point: " + str(self._start_point)
        if self._end_point is not None:
            output = output + " | End point: " + str(self._end_point)
        if self._edge_vector is not None:
            output = output + " | Edge vector: " + str(self._edge_vector)
        return  output

    @property
    def start_point(self):
        return self._start_point

    @start_point.setter
    def start_point(self, value: np.array):
        self._start_point = value
        self.calculate_edge_vector()

    @property
    def end_point(self):
        return self._end_point

    @end_point.setter
    def end_point(self, value: np.array):
        self._end_point = value
        self.calculate_edge_vector()

    @property
    def edge_vector(self):
        return self._edge_vector


    def calculate_edge_vector(self):
        if self._start_point is not None and self._end_point is not None:
            self._edge_vector = self._end_point - self._start_point
    
