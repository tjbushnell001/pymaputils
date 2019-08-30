import json


# We've removed a lot of the unimportant parts of the json including some
# structural components (such as connector inflows / outflows)
def create_tile(tile_id):
    return {
        # There's always just one connector (containing a single junction)
        "connectors": [
            {
                "id": 1,
                # There's always just one junction (the junction being tested)
                "junctions": [
                    {
                        "id": 1,
                        "inflow_refs": [],
                        "outflow_refs": [],
                        "type": "junction"
                    }
                ],
                "type": "connector"
            }
        ],
        "id": tile_id,
        "lane_groups": [],
        "type": "tile"
    }


def create_lane_group(lg_id, is_ramp=False):
    return {
        "id": lg_id,
        "is_ramp": is_ramp,
        "lanes": [],
        "type": "lane_group"
    }


def create_lane_ref(l_id, lg_id, tile_id):
    return {
        "id": l_id,
        "lane_group_id": lg_id,
        "tile_id": tile_id,
        "type": "lane_ref"
    }


def create_lane(l_id, transiton_type):
    return {
        "id": l_id,
        "lane_transition_type": transiton_type,
        "type": "lane"
    }


def add_lanes(tile, lg, count, flow_key, transiton_type):
    for _ in range(count):
        lane_ref = create_lane_ref(len(lg['lanes']) + 1, lg['id'], tile['id'])
        tile['connectors'][0]['junctions'][0][flow_key].append(lane_ref)
        lane = create_lane(len(lg['lanes']) + 1, transiton_type)
        lg['lanes'].append(lane)


def main():
    seen_cases = set()

    with open("nominal_test_cases.txt", "r") as infile:
        for _ in range(3):
            # First three lines are comments
            infile.readline()
        line = infile.readline()
        tile_id = 4
        while line:
            line = line[:-1]
            tile = create_tile(tile_id)
            tile['id'] = tile_id
            properties = line.split("\t")
            print properties
            in_lg_count = int(properties[0])
            out_lg_count = int(properties[1])

            if in_lg_count == 0 or out_lg_count == 0:
                # logic doesn't handle this case yet
                tile_id += 1
                continue

            if properties[2] in seen_cases:
                # already dealt with this case. The only difference is the number
                # of incoming / outgoing lane groups and the current notation elaves
                # ambiguity in such cases
                tile_id += 1
                continue

            seen_cases.add(properties[2])

            # --------------
            # In Lane Groups
            # --------------
            # In LG's are based on id's starting at 1
            lg_id = 1

            in_fw_lanes = sum([int(i) for i in properties[3:6]])
            if in_fw_lanes > 0:
                lg = create_lane_group(lg_id)
                lg_id += 1

                # --------------
                # Fw In Lanes
                # --------------
                add_lanes(tile, lg, int(properties[3]), "inflow_refs", "UNKNOWN")
                add_lanes(tile, lg, int(properties[4]), "inflow_refs", "MERGE")
                add_lanes(tile, lg, int(properties[5]), "inflow_refs", "SPLIT")

                tile['lane_groups'].append(lg)

            in_ramp_lanes = sum([int(i) for i in properties[6:9]])
            if in_ramp_lanes > 0:
                lg = create_lane_group(lg_id, is_ramp=True)

                # --------------
                # Ramp In Lanes
                # --------------
                add_lanes(tile, lg, int(properties[6]), "inflow_refs", "UNKNOWN")
                add_lanes(tile, lg, int(properties[7]), "inflow_refs", "MERGE")
                add_lanes(tile, lg, int(properties[8]), "inflow_refs", "SPLIT")

                tile['lane_groups'].append(lg)

            # ---------------
            # Out Lane Groups
            # ---------------
            # Out LG's are based on id's starting at 100
            lg_id = 100

            out_fw_lanes = sum([int(i) for i in properties[9:12]])
            if out_fw_lanes > 0:
                lg = create_lane_group(lg_id)
                lg_id += 1

                # --------------
                # Fw Out Lanes
                # --------------
                add_lanes(tile, lg, int(properties[9]), "outflow_refs", "UNKNOWN")
                add_lanes(tile, lg, int(properties[10]), "outflow_refs", "MERGE")
                add_lanes(tile, lg, int(properties[11]), "outflow_refs", "SPLIT")

                tile['lane_groups'].append(lg)

            out_ramp_lanes = sum([int(i) for i in properties[12:15]])
            if out_ramp_lanes > 0:
                lg = create_lane_group(lg_id, is_ramp=True)

                # --------------
                # Ramp Out Lanes
                # --------------
                add_lanes(tile, lg, int(properties[12]), "outflow_refs", "UNKNOWN")
                add_lanes(tile, lg, int(properties[13]), "outflow_refs", "MERGE")
                add_lanes(tile, lg, int(properties[14]), "outflow_refs", "SPLIT")
                tile['lane_groups'].append(lg)

            # Write the tile
            with open("{}.json".format(tile_id), "w") as outfile:
                json.dump(tile, outfile, indent=4, sort_keys=True)

            line = infile.readline()
            tile_id += 1


if __name__ == "__main__":
    main()
