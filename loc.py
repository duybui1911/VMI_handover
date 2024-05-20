from collections import defaultdict
import numpy as np

class Loc:
    loc2Node = defaultdict()
    node2loc = defaultdict()
    node2demand = defaultdict()
    node2fincost = defaultdict()
    no_depots = 0
    depot_name = None
    depot_node = None
    no_raw_nodes = 0
    no_all_nodes = 0
    demand_sum = 0
    all_parted_nodes = []

    def __init__(self, name, demand, loc_type):
        self.name = name
        self.loc_type = loc_type
        self.demand = demand
        Loc.demand_sum += demand.sum()
        self.is_splitted = None
        self.node = None

    def get_finance_cost(self, interest_rate, no_days):
        if self.loc_type == 'depot':
            self.node = Loc.depot_node[self.name]
            Loc.node2loc[self.node] = self.name
            Loc.node2demand[self.node] = self.demand
            Loc.node2fincost[self.node] = 0
        else:
            self.node = Loc.no_all_nodes
            Loc.no_all_nodes += 1
            Loc.node2loc[self.node] = self.name
            Loc.node2demand[self.node] = self.demand
        self.calc_finance_cost(interest_rate, no_days)

    def calc_finance_cost(self, interest_rate, no_days):
        fin_cost = np.array(self.demand * interest_rate / 365.0 * no_days * 10000, dtype=int).sum()
        Loc.node2fincost[self.node] = fin_cost
