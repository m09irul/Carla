def get_left_right_indexes(waypoint_list):
    indexes = []
    last_found = None
    for i, (_, road_option) in enumerate(waypoint_list):
        if road_option in ['left', 'right'] and road_option != last_found:
            indexes.append(i)
            last_found = road_option
        elif road_option not in ['left', 'right']:
            last_found = None
    return indexes

# Example usage:
waypoints = [('left', 'left'), ('left', 'left'), ('left', 'left'), ('right', 'right'),
             ('void', 'void'), ('right', 'right'), ('right', 'right'), ('left', 'left'),
             ('right', 'right'), ('left', 'left'), ('left', 'left'), ('left', 'left'),
             ('void', 'void')]
indexes = get_left_right_indexes(waypoints)
print("Indexes:", indexes)
