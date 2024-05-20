import numpy as np
import pandas as pd
import copy
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import utils
from loc import Loc

class VRP():
    def __init__(self, Loc, data):
        self.Loc = Loc
        self.data = data
        self.time_limit = 60
        self.lns_time_limit = 3

        ## create routing manager and Routing model
        self.manager = pywrapcp.RoutingIndexManager(
            self.Loc.no_all_nodes, 
            self.data['dpc_no_vhc'], 
            self.data['dpc_starts'], 
            data['dpc_ends']
        )
        self.routing = pywrapcp.RoutingModel(self.manager)
        self.solver = self.routing.solver()

    def run(self, ):
        self.set_cost_callback_index()
        self.add_allow_Vehicles()
        self.set_demand_callback_index()
        self.set_distance_callback_index()
        self.set_time_callback_index()
        self.set_time_window()
        self.add_fixed_cost()
        self.drop_nodes()

        solution = self.find_sol()
        self.print_sol(solution=solution)


    ## Create and register a cost transit callback.
    def cost_callback(self, from_index, to_index):
        """Returns the cost between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        from_name = self.data['node2loc'][from_node]
        to_name = self.data['node2loc'][to_node]
        # Cost = (
        #   mean of cost ALS_logistics (tạm thời lấy ở xe có tải trọng lớn nhất)*KM + fincost + time_cost
        #)
        # Do ở pha này chạy thử chưa có nghiệm nên tạm thời bỏ chi phí thời gian đi
        travel_cost = int(50000 * self.data['dist_dict'][from_name][to_name])
        fin_cost = self.Loc.node2fincost[to_node]
        return travel_cost + fin_cost

    def set_cost_callback_index(self, ):
        cost_callback_index = self.routing.RegisterTransitCallback(self.cost_callback)

        ## Create cost dimension
        cost = 'Cost'
        self.routing.AddDimension(
            cost_callback_index,
            slack_max=0,
            capacity=100000, # 10 trieu VNĐ = 100000 * 100 VNĐ
            fix_start_cumul_to_zero=True,
            name=cost)
        self.cost_dimension = self.routing.GetDimensionOrDie(cost)
        # cost_dimension.SetGlobalSpanCostCoefficient(100000)

        ## Define cost of each arc.
        for vhc_id in range(self.data['dpc_no_vhc']):
            self.routing.SetArcCostEvaluatorOfVehicle(cost_callback_index, vhc_id)
        # routing.SetArcCostEvaluatorOfAllVehicles(cost_callback_index)

    def add_allow_Vehicles(self, ):
        ## Add Allow Vehicles constraint
        for loc in self.Loc.loc2Node:
            Node = self.Loc.loc2Node[loc]
            if Node.loc_type == 'ctm':
                _depot_name = self.data['ctm_depot_name'][loc]
                if Node.is_splitted:
                    nodes = [node for parted_nodes in Node.node for node in parted_nodes]
                    for node in nodes:
                        self.routing.SetAllowedVehiclesForIndex(
                            self.data['dpc_allowed_vhc'][_depot_name], self.manager.NodeToIndex(node))
                else:
                    self.routing.SetAllowedVehiclesForIndex(
                        self.data['dpc_allowed_vhc'][_depot_name], self.manager.NodeToIndex(Node.node))

    def demand_callback(self, from_index):
        from_node = self.manager.IndexToNode(from_index)
        return self.data['node2demand'][from_node]

    def set_demand_callback_index(self, ):
        demand_callback_index = self.routing.RegisterUnaryTransitCallback(self.demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self.data['dpc_vhc_cap'],  # max vehicle capacities
            True,  # start cumul to zero
            'Capacity'
        )

    ## Create and register a distance transit callback.
    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        from_nane = self.data['node2loc'][from_node]
        to_name = self.data['node2loc'][to_node]
        
        # distance from km to hm
        return np.int64(10 * self.data['dist_dict'][from_nane][to_name])

    def set_distance_callback_index(self, ):
        distance_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)
        ## Add Distance dimension.
        dist = 'Distance'
        self.routing.AddDimension(
            distance_callback_index,
            0,  # no slack
            1200,  # vehicle maximum travel distance 1000 hm = 100 km
            True,  # start cumul to zero
            dist)
        self.distance_dimension = self.routing.GetDimensionOrDie(dist)


    ## Create and register a time transit callback.
    def time_callback(self, from_index, to_index):
        """Returns the transit time between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        from_nane = self.data['node2loc'][from_node]
        to_name = self.data['node2loc'][to_node]
        # time = 60 (hour to minute) * dist (km) / 35 (km/h) (gia su oto chay 35 km/h)
        return np.int64(60 * self.data['dist_dict'][from_nane][to_name] / 35)

    def set_time_callback_index(self, ):
        time_callback_index = self.routing.RegisterTransitCallback(self.time_callback)
        ## Add Time dimension.
        time = 'Time'
        self.routing.AddDimension(
            time_callback_index,
            30,  # no slack
            self.data['no_days'] * 24 * 60,  # maximum time
            False,  # start cumul to zero
            time)
        self.time_dimension = self.routing.GetDimensionOrDie(time)

    def set_time_window(self, ):
        # Set time window for depot
        for vhc_index in range(self.data['dpc_no_vhc']):
            depot_index = self.data['dpc_starts'][vhc_index]
            depot_name = self.data['depot_name'][depot_index]
            index = self.routing.Start(vhc_index)
            day = vhc_index // self.data['no_vhc']
            self.time_dimension.CumulVar(index).SetRange(
                day * 60 * 24, (day + 1) * 60 * 24)

        # Set time window and unload time for customer
        # also dont serve same parted demands in same day
        for loc in self.Loc.loc2Node:
            Node = self.Loc.loc2Node[loc]
            if Node.loc_type == 'ctm':
                loc_name = Node.name
                if Node.is_splitted:
                    nodes = [node for parted_nodes in Node.node for node in parted_nodes]
                    # Dont allow parted demand is served in the same day
                    for parted_nodes in Node.node:
                        for i in range(0, len(parted_nodes) - 1):
                            node_i = parted_nodes[i]
                            node_next = parted_nodes[i + 1]
                            id_i = self.manager.NodeToIndex(node_i)
                            id_next = self.manager.NodeToIndex(node_next)
                            time_between_2days = int(60 * (24 + 8.5 - 17.5))
                            self.solver.Add(
                                self.time_dimension.CumulVar(id_next) -
                                self.time_dimension.CumulVar(id_i) >= time_between_2days)
                else:
                    nodes = [Node.node]
                for node in nodes:
                    index = self.manager.NodeToIndex(node)
                    # Set time window for each node
                    self.time_dimension.CumulVar(index).SetRange(
                        self.data['time_windows'][loc_name][0],
                        self.data['time_windows'][loc_name][1])
                    # Add unload time no less than 15 minutes
                    self.solver.Add(self.time_dimension.SlackVar(index) >= 15)
                    # Remove off-time time intervals
                    for off_time in self.data['off_time_windows'][loc_name]:
                        self.time_dimension.CumulVar(index).RemoveInterval(
                            off_time[0], off_time[1])

        # Set each vehicle will not go tomorrow if to day don't go
        for vhc in range(self.data['no_vhc']):
            for day in range(self.data['no_days'] - 1):
                self.solver.Add(
                    self.routing.ActiveVehicleVar(vhc + day * self.data['no_vhc']) >=
                    self.routing.ActiveVehicleVar(vhc + (day + 1) * self.data['no_vhc']))

    def add_fixed_cost(self, ):
        # Add fixed cost for all vehicles with prefer earlier vehicles
        vhc_fixed_base_cost = 10000 # 1 trieu
        for vhc in range(self.data['dpc_no_vhc']):
            day = vhc // self.data['no_vhc']
            self.routing.SetFixedCostOfVehicle(day * vhc_fixed_base_cost, vhc) # 1 trieu

        # Add constraint to allow only one type of parted_demands each location
        # Example: parted_nodes = [[181], [182, 183], [184, 185, 186]]
        for parted_nodes in self.Loc.all_parted_nodes:
            flag = False
            for nodes in parted_nodes:
                id_0 = self.manager.NodeToIndex(nodes[0])
                if not flag:
                    active_var_sum = self.routing.ActiveVar(id_0)
                    flag = True
                else:
                    active_var_sum += self.routing.ActiveVar(id_0)
                for i in range(1, len(nodes)):
                    id_i = self.manager.NodeToIndex(nodes[i])
                    self.solver.Add(
                        self.routing.ActiveVar(id_0) == self.routing.ActiveVar(id_i))
            self.solver.Add(active_var_sum == 1)
    
    def drop_nodes(self, ):
        # Allow to drop nodes.
        penalty = 100000 # 1 trieu
        for node in range(self.Loc.no_depots - 1, self.Loc.no_all_nodes):
            self.routing.AddDisjunction([self.manager.NodeToIndex(node)], penalty)
        # for name in data['ctm_name']:
        #     node = data['ctm_node'][name]
        #     routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    def find_sol(self, ):
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_INSERTION
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        # AUTOMATIC GREEDY_DESCENT SIMULATED_ANNEALING TABU_SEARCH GENERIC_TABU_SEARCH
        search_parameters.time_limit.seconds = self.time_limit
        search_parameters.lns_time_limit.seconds = self.lns_time_limit

        # Solve the problem.
        solution = self.routing.SolveWithParameters(search_parameters)
        return solution
    
    def print_sol(self, solution):
        status = {
            0: 'ROUTING_NOT_SOLVED: Problem not solved yet.',
            1: 'ROUTING_SUCCESS: Problem solved successfully.',
            2: 'ROUTING_FAIL: No solution found to the problem.',
            3: 'ROUTING_FAIL_TIMEOUT: Time limit reached before finding a solution.',
            4: 'ROUTING_INVALID: Model, model parameters, or flags are not valid.'
        }
        print("Routing status: ", status[self.routing.status()])
        if solution:
            print('******************SOLUTION FOUND******************\n')
            routes, total_fin_cost, total_ship_cost = utils.print_solution(
                self.data, 
                self.manager, 
                self.routing, 
                solution,
                self.Loc
            )
            print('Fin_cost: ', total_fin_cost)
            print('Ship_cost: ', total_ship_cost)
            print('Total_cost: ', total_fin_cost + total_ship_cost)
        else:
            print('******************SOLUTION NOT FOUND******************')


if __name__ == '__main__':
    cfg = utils.configs_input(
        r'data/distances.xlsx',
        # r'data/ATM_pickup.csv'
    )
    items = pd.read_excel(f'data/chiphicohoi.xlsx')
    fin_cost_rate = items['Gia_ban'].values/np.sum(items['Gia_ban'].values)
    num_product = len(fin_cost_rate)
    no_vhc = 100
    vhc_cap = [130000] * no_vhc

    start_nodes = [0 for _ in range(no_vhc)]
    end_nodes = [0 for _ in range(no_vhc)]
    # Depot = 0911358800-TM
    allowed_vhc = {'0911358800-TM': [i for i in range(no_vhc)]}

    dpc_no_vhc = no_vhc * cfg['no_days']
    dpc_vhc_cap = vhc_cap * cfg['no_days']
    dpc_start_nodes = start_nodes * cfg['no_days']
    dpc_end_nodes = end_nodes * cfg['no_days']
    dpc_allowed_vhc = copy.deepcopy(allowed_vhc)

    for key in allowed_vhc:
        for day in range(1, cfg['no_days']):
            dpc_allowed_vhc[key] += [vhc + no_vhc * day for vhc in allowed_vhc[key]]
    Loc.no_depots = len(cfg['depot_name'])
    Loc.depot_name = cfg['depot_name']
    Loc.depot_node = cfg['depot_node']
    Loc.no_all_nodes += Loc.no_depots

    # Get demand for NODE
    for name in cfg['loc_name']:
        if name in cfg['depot_name']:
            new_loc = Loc(name=name, demand=np.zeros(num_product), loc_type='depot')
            Loc.loc2Node[name] = new_loc
        else:
            demand = np.array([np.random.randint(30, 150) for _ in range(num_product)]) # Do data chưa có nhu cầu của đại lý bán lẻ
            new_loc = Loc(name=name, demand=demand, loc_type='ctm')
            Loc.loc2Node[name] = new_loc

    Loc.no_raw_nodes = len(Loc.loc2Node)

    for key in Loc.loc2Node:
        Loc.loc2Node[key].get_finance_cost(
            fin_cost_rate,
            cfg['no_days']
        )

    data = {}
    data['dist_dict'] = cfg['dist_dict']
    data['node2demand'] = Loc.node2demand
    data['vhc_cap'] = vhc_cap
    data['no_vhc'] = no_vhc
    data['starts'] = start_nodes
    data['ends'] = end_nodes
    data['allowed_vhc'] = allowed_vhc
    data['dpc_no_vhc'] = dpc_no_vhc
    data['dpc_vhc_cap'] = dpc_vhc_cap
    data['dpc_starts'] = dpc_start_nodes
    data['dpc_ends'] = dpc_end_nodes
    data['dpc_allowed_vhc'] = dpc_allowed_vhc

    data['node2loc'] = Loc.node2loc
    data['depot_name'] = cfg['depot_name']
    data['ctm_name'] = cfg['ctm_name']
    data['depot_node'] = cfg['depot_node']
    data['ctm_node'] = cfg['ctm_node']
    data['ctm_depot_name'] = cfg['ctm_depot_name']

    data['no_days'] = cfg['no_days']
    data['working_time'] = cfg['working_time']
    data['time_windows'] = cfg['time_windows']
    data['off_time_windows'] = cfg['off_time_windows']

    model = VRP(Loc, data)
    model.run()