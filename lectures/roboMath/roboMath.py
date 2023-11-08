import numpy as np

def to_deg(radians:float) -> float:
    return radians * 180/np.pi

def to_radians(degrees: float) -> float:
    return degrees * np.pi / 180

def rot_mat2(input: float, i_type: str = "rad") -> np.matrix:
    if(i_type == "deg"):
        input = to_radians(input)
    R = np.matrix([
        [np.cos(input), -np.sin(input)],
        [np.sin(input), np.cos(input)]
    ])
    return R

