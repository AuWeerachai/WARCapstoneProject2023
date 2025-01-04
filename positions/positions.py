import numpy as np
import json
import pyrealsense2 as rs

class LookupTable(dict):
    def __init__(self):
        super().__init__()
        self.filepath = "table.war"
        self.read()

    def read(self):
        with open(self.filepath, "r") as readfile:
            try:
                table = json.load(readfile)
            except:
                return None
            for key, value in table.items():
                super().__setitem__(key, value)
    
    def save_to_file(self):
        with open(self.filepath, "w") as file:
            json.dump(self, file, indent=2)

    def __setitem__(self, __key: np.ndarray, __value: np.ndarray) -> None:
        return super().__setitem__(str(__key), str(__value))

    def __contains__(self, __key: np.ndarray) -> bool:
        return super().__contains__(str(__key))
    
    def __getitem__(self, __key: np.ndarray) -> np.ndarray:
        value = super().__getitem__(str(__key))
        return np.fromstring(value.strip("[]"), sep=' ')
    
    def __delitem__(self, __key: np.ndarray) -> None:
        return super().__delitem__(str(__key))
    
    def clear(self) -> None:
        return super().clear()

class Coordinates(dict):
    def __init__(self):
        super().__init__()
        self.raw_points = dict()

    def __setitem__(self, __key: int, __value: tuple) -> None:
        self.raw_points[__key] = __value
    
    def add_coordinate(self, __key: int, __value: tuple) -> None:
        return super().__setitem__(__key, __value)

    def __getitem__(self, __key: int) -> tuple:
        return super().__getitem__(__key)
    
    def transform(self, depth_frame, intrinsics):
        bad_point = None
        for key, value in self.raw_points.items():
            # print(key, value)
            try:
                depth = depth_frame.get_distance(int(value[0]), int(value[1]))
                # if depth < self.high_bound and depth > self.low_bound:
                dist = rs.rs2_deproject_pixel_to_point(intrinsics, [int(value[0]), int(value[1])], depth)
                # scale = 0.88495575
                super().__setitem__(key, (dist[1], -dist[0]))
            except RuntimeError:
                bad_point = key
        if bad_point:
            self.pop(bad_point)
    
    def clear(self):
        super().clear()
        self.raw_points.clear()

if __name__ == '__main__':
    point = np.array([1, 2, 4])
    angles = np.array([4, 5, 11, 7, 8, 9])
    t = LookupTable()
    t[point] = angles
    print(t[point])
    print(type(t[point]))
    print(t[point]*2)
    print(t.keys())
    t.save_to_file()