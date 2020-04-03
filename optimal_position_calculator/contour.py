import sys

import numpy as np

class contour:
    #Member
    __corner_point_list: list
    __edge_list: list

    def __init__(self):
        self.__corner_point_list = list()
        self.__edge_list = list()
        print(self.__corner_point_list)


if __name__=='__main__':
    c = contour()
