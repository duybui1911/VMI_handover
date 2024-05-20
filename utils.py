from collections import defaultdict
import numpy as np
import pandas as pd



def configs_input(dist_path):
    dist_df = pd.read_excel(dist_path)
    pickup_df = pd.read_excel(dist_path)
    cfg = {
        'no_days': 7,
        'working_time': [8 * 60, 17 * 60 + 30],
        'time_windows': defaultdict(list),
        'off_times': [],
        'off_time_windows': defaultdict(list),
        'loc_name': list(set(dist_df["Code"])),
    }

    cfg['loc_name'].sort()
    cfg['depot_name'] = ['0911358800-TM']
    cfg['depot_node'] = {'0911358800-TM': 0}
    cfg['ctm_depot_name'] = {}
    for _loc, _depot in zip(list(pickup_df["Code"]), list(pickup_df["From"])):
        if _loc != _depot:
            cfg['ctm_depot_name'][_loc] = _depot
    cfg['ctm_name'] = [name for name in cfg['loc_name'] if name not in cfg['depot_name']]
    cfg['ctm_name'].sort()

    cfg['ctm_node'] = {}
    for i, loc in enumerate(cfg['ctm_name']):
        cfg['ctm_node'][loc] = i + len(cfg['depot_name'])
    cfg['loc_name'] = cfg['depot_name'] + cfg['ctm_name']
    cfg['dist_dict'] = defaultdict(dict)

    for start, end, dist in zip(
        list(dist_df["From"]), list(dist_df["Code"]), list(dist_df["Distances"])
    ):
        cfg['dist_dict'][start][end] = dist
    for start in list(dist_df["From"]):
        cfg['dist_dict'][start][start] = 0
    for name in cfg['loc_name']:
        if name in cfg['depot_name']:
            cfg['time_windows'][name] = [0, cfg['no_days'] * 24 * 60]
        else:
            cfg['time_windows'][name] = [
                cfg['working_time'][0],
                cfg['working_time'][1] + (cfg['no_days'] - 1) * 24 * 60,
            ]
    for i in range(cfg['no_days'] - 1):
        cfg['off_times'].append(
            [24 * 60 * i + cfg['working_time'][1], 24 * 60 * (i + 1) + cfg['working_time'][0]]
        )
    for name in cfg['ctm_name']:
        cfg['off_time_windows'][name] = cfg['off_times']
    return cfg



def format_time(no_mins):
    no_mins_per_day = 24 * 60
    residual_time = no_mins % no_mins_per_day
    hours = residual_time // 60
    minutes = residual_time % 60
    return '{0}h{1}'.format(hours, minutes)

def print_solution(data, manager, routing, solution, Loc):
    total_ship_cost = 0
    total_fin_cost = 0
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")

    # Display dropped nodes.
    print('Dropped nodes:', end='')
    dropped_nodes = defaultdict(list)
    for index in range(routing.Size()):
        if routing.IsStart(index) or routing.IsEnd(index):
            continue
        if solution.Value(routing.NextVar(index)) == index:
            node = manager.IndexToNode(index)
            name = data['node2loc'][node]
            depot = data['ctm_depot_name'][name]
            dropped_nodes[depot] += [node]
            # dropped_nodes_info += ' {}'.format(data['loc_name'][manager.IndexToNode(node)])
    # Adjust dropped nodes
    for parted_nodes in Loc.all_parted_nodes:
        loc_name = data['node2loc'][parted_nodes[0][0]]
        depot = data['ctm_depot_name'][loc_name]
        all_nodes = [node for nodes in parted_nodes for node in nodes]
        nodes_difference = list(set(all_nodes) - set(dropped_nodes[depot]))
        dropped_nodes[depot] = list(set(dropped_nodes[depot]) - set(all_nodes))
        if len(nodes_difference) == 0:
            dropped_nodes[depot] += all_nodes[0]

    if len(dropped_nodes) == 0:
        print(' None\n')
    else:
        print('')
        for key in dropped_nodes:
            loc_names = [data['node2loc'][node] for node in dropped_nodes[key]]
            print(key, ':', str(loc_names))
            print(key, ':', str(dropped_nodes[key]))
        print('')


    time_dimension = routing.GetDimensionOrDie('Time')
    routes = []
    total_cost = 0
    total_load = 0
    total_dist = 0
    total_time = 0
    for vehicle_id in range(data['dpc_no_vhc']):
        vhc = vehicle_id % data['no_vhc']
        day = vehicle_id // data['no_vhc']
        cur_route = []
        index = routing.Start(vehicle_id)
        # plan_output = 'Route for vehicle {} (day {}):\n'.format(vhc + 1, day + 1)
        print('Route for vehicle {} (day {}):'.format(vhc + 1, day + 1))
        load_output = ''
        time_output = ''

        route_cost = 0
        route_load = 0
        route_dist = 0
        flag = False
        while not routing.IsEnd(index):
            if not flag:
                start_time = solution.Min(time_dimension.CumulVar(index))
                flag = True
            node_index = manager.IndexToNode(index)

            cur_route.append(node_index)
            name = data['node2loc'][node_index]
            # route_load += data['demands'][name]
            route_load += data['node2demand'][node_index]
            prev_index = index

            time_var = time_dimension.CumulVar(prev_index)

            index = solution.Value(routing.NextVar(index))
            route_cost += routing.GetArcCostForVehicle(
                prev_index, index, vehicle_id)

            prev_node_index = manager.IndexToNode(prev_index)
            prev_node_name = data['node2loc'][prev_node_index]
            node = manager.IndexToNode(index)
            node_name = data['node2loc'][node]

            travel_dist = data['dist_dict'][prev_node_name][node_name]
            route_dist += travel_dist

            travel_cost = int(25 * travel_dist)
            fin_cost = Loc.node2fincost[prev_node_index]
            load_output += '{0} ({1} ~ {2}) --{3}km {4}--> '.format(name,
                                                          data['node2demand'][node_index],
                                                          fin_cost / 10000., # trăm ngàn sang triệu
                                                          np.round(travel_dist,1),
                                                          travel_cost / 10000.) # trăm ngàn sang triệu
            total_ship_cost += travel_cost / 10000.
            total_fin_cost += fin_cost / 10000.
            travel_time = np.int64(60 * travel_dist / 35)
            time_output += '{0} Time({1},{2}) --{3}phút--> '.format(
                node_index, format_time(solution.Min(time_var)),
                format_time(solution.Max(time_var)), travel_time)

            diff_length_outputs = len(load_output) - len(time_output)
            if diff_length_outputs > 0:
                time_output += ' ' * diff_length_outputs
            elif diff_length_outputs < 0:
                load_output += ' ' * -diff_length_outputs

        cur_route.append(manager.IndexToNode(index))
        routes.append(cur_route)
        name = data['node2loc'][manager.IndexToNode(index)]
        load_output += '{0} ({1} ~ {2})'.format(name,
                                                      data['node2demand'][node],
                                                      Loc.node2fincost[node] / 10000.) # trăm ngàn sang triệu
        time_var = time_dimension.CumulVar(index)
        time_output += '{0} Time({1},{2})'.format(node_index,
                                                  format_time(solution.Min(time_var)),
                                                  format_time(solution.Max(time_var)))
        stat_output = 'Cost of the route: {:,} VNĐ\n'.format(route_cost * 100)
        stat_output += 'Load of the route: {} \n'.format(route_load / 1000.)
        stat_output += 'Length of the route: {:.1f} km\n'.format(route_dist)
        stat_output += 'Time of the route: {}\n'.format(
            format_time(solution.Min(time_var) - start_time))
        print(load_output)
        print(time_output)
        print(stat_output)
        total_cost += route_cost
        total_load += route_load
        total_dist += route_dist
        total_time += solution.Min(time_var)
    print('Total cost of all routes: {:,} VNĐ'.format(total_cost * 100))
    print('Total load of all routes: {} tỷ'.format(total_load / 1000.))
    print('Total length of all routes: {:.1f} km'.format(total_dist))
    print('Total time of all routes: {} phút'.format(total_time))
    return routes, total_fin_cost, total_ship_cost