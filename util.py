import config as c


def required_movement_to_re_center_tracking_box(box_center, horizontal_movement=0):
    """
        returns the needed direction and size of movement to re center the tracking box to the frame center.
        should receive indication if it is horizontal (0) or vertical (1) movement.
    """

    if horizontal_movement:
        frame_center = c.CENTER_W
        threshold_for_diff = c.MIN_PAN_DIFF

    else:
        frame_center = c.CENTER_H
        threshold_for_diff = c.MIN_TILT_DIFF

    diff = frame_center - box_center
    direction = 1 if diff >= 0 else -1
    diff = abs(diff)

    if diff < threshold_for_diff or diff == 0:
        return 0

    else:
        return -int(diff / (frame_center/3) + 1) * direction


def tuple_diff(first_tup, second_tup):
    """
        returns the highest difference between the first or second item in both tuples respectively
    """
    return max(abs(second_tup[0] - first_tup[0]), abs(second_tup[1] - first_tup[1]))


def tuple_within_range(tuple, first_range, second_range):
    """
    return true if given tuple's first and second number are within first and second range respectively.
    """
    return is_within_range(tuple[0], first_range[0], first_range[1]) and \
           is_within_range(tuple[1], second_range[0], second_range[1])


def is_within_range(value, lowest_bound, highest_bound):
    """
    return true if value is between the bounds.
    """
    return lowest_bound <= value <= highest_bound


def clip_servo_pulse_width(pulse_width):
    """
    if pulse_width is below/above MIN/MAX_PULSE_WIDTH, return MIN/MAX_PULSE_WIDTH, otherwise, return pulse_width.
    """
    if pulse_width > c.MAX_PULSE_WIDTH:
        return c.MAX_PULSE_WIDTH
    if pulse_width < c.MIN_PULSE_WIDTH:
        return c.MIN_PULSE_WIDTH

    return pulse_width