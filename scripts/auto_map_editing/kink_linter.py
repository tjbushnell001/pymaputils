import json
import numpy as np

base_dir = '/Users/ranreichman/'
full_brain_dir = 'src/brain/perception/lane_map_server/maps/tiled_maps/usa/tiles/'

def getAngle(p1, p2, p3):
    try:
        a = np.array(p1)
        b = np.array(p2)
        c = np.array(p3)

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)
    except:
        return 90

    return np.degrees(angle)

def lint_tile(tile_id, lane_group_ids):
    full_path = base_dir + full_brain_dir + str(tile_id) + ".json"

    with open(full_path, "r") as f:
        tile_data = json.load(f)

    used_coordinates = set()

    ANGLE_THRESHOLD_FOR_KINK_IN_DEGREES = 2

    for lane_group in tile_data['lane_groups']:
        if lane_group['id'] not in lane_group_ids:
            continue
        if not lane_group['is_ramp']:
            for lane in lane_group['lanes']:
                lane_group['boundaries'].append(lane)
                for boundary in lane_group['boundaries']:
                    if 'is_only_emergency_boundary' in boundary and boundary['is_only_emergency_boundary']:
                        continue
                    if 'is_emergency_lane' in boundary and boundary['is_emergency_lane']:
                        continue
                    boundary_coordinates = boundary['pts']
                    #if boundary_coordinates[0][0] < -113.2456903 and not (boundary_coordinates[0][0] > -122.0 and boundary_coordinates[0][1] > 34.3):
                    for i in range(len(boundary_coordinates) - 2):
                        angle = getAngle(boundary_coordinates[i], boundary_coordinates[i + 1],
                                         boundary_coordinates[i + 2])
                        if abs(180 - angle) > ANGLE_THRESHOLD_FOR_KINK_IN_DEGREES:
                            reversed_coordinate = boundary_coordinates[i + 1][::-1]
                            coordinate_str = str(reversed_coordinate)
                            if coordinate_str not in used_coordinates:
                                used_coordinates.add(coordinate_str)
                                print("angle: " + str(angle))
                                print(str(reversed_coordinate))

if __name__ == '__main__':
    tiles_and_lane_groups = {309137510: [37137537443, 37137537443], 309134520: [37147567044, 37120068502, 37120068502, 37120068502, 37120068502], 309134569: [2935771231, 2935771231, 2935771231, 2935771231, 2935771231, 2935771231, 8565008554, 8565008554, 38210041255], 309134719: [26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183, 26745067183], 309137450: [37660040646, 37660040646, 37660040646, 37660040646, 37660040646, 37660040646, 37660040646, 37660040646, 37660040646, 37660040646], 309134580: [37140038853, 37140038853, 37140038853, 37140038853, 37140038853], 309134726: [37167538179, 37535042605, 38272541549], 308788158: [37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191, 37692539191], 308788159: [37665041436, 37665041436], 309137680: [33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33102553662, 33157543738, 33157543738, 33157543738, 33157543738], 309137471: [24692631605], 309137467: [38242541838, 38242541838], 309137466: [380935037, 380935037, 380935037], 309134576: [9320009088, 9320009088], 309134577: [1855935319, 1855935319, 1855935319, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976, 37615039976], 309134736: [550562367, 550562367, 550562367, 550562367, 550562367, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813, 38217541813], 309134578: [37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37477539350, 37450039706, 37450039706, 37450039706, 37450039706, 37450039706, 37450039706], 309137500: [37650038514, 37650038514, 37650038514, 37650038514], 309137513: [38262543828, 38262543828], 309137666: [38270041695, 38270041695], 309134521: [9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 9320009397, 38270042021, 38270042021, 38270042021, 38270042021, 38270042021, 38270042021], 309134525: [9835007958, 2628270384, 2628270384, 2628270384, 2628270384, 2628270384, 2628270384, 2628270384], 309134725: [29637540437, 29637540437, 29642539119]}
    #tiles_and_lane_groups = {309134504 : [61797688831]}

    for tile_id in tiles_and_lane_groups:
        lane_groups = tiles_and_lane_groups[tile_id]
        lint_tile(tile_id, lane_groups)