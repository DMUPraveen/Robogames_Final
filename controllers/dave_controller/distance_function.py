from bisect import bisect_left
LOOKUP = [(0, 4095),
          (0.005, 2133.33),
          (0.01, 1465.73),
          (0.015, 601.46),
          (0.02, 383.84),
          (0.03, 234.93),
          (0.04, 158.03),
          (0.05, 120),
          (0.06, 104.09),
          (0.07, 67.19)]

REVERSE_LOOKUP = sorted([tuple(reversed(val)) for val in LOOKUP])


def find_range(sensor_reading):
    '''
    return  max_val,min_val
    '''
    index_range_max = bisect_left(REVERSE_LOOKUP, (sensor_reading,0))
    index_range_min = index_range_max-1
    max_val = REVERSE_LOOKUP[index_range_max]
    min_val = REVERSE_LOOKUP[index_range_min]
    # print(f"{max_val=}, {min_val=}, {sensor_reading}")
    return max_val, min_val


MIN_READING = 75
MAX_READING = 4094
WALL_NOT_DETECTED_READING = float("inf")


def interpolate(value_range, sensor_reading):
    (r_max, d_max), (r_min, d_min) = value_range
    r_sensor_reading = sensor_reading
    sensor_distance = d_max - (r_sensor_reading-r_min) * \
        (d_max-d_min)/(r_max-r_min)
    return sensor_distance

def debug_decorator(f):
    def debug_func(*args):
        a = f(*args)
        print(a,*args)
        return a
    return debug_func
    

# @debug_decorator
def transform_sensor_reading(sensor_reading):

    if sensor_reading >= MAX_READING:
        return 0
    if sensor_reading <= MIN_READING:
        return WALL_NOT_DETECTED_READING

    return interpolate(
        find_range(sensor_reading),
        sensor_reading
    )
