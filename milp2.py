import json
import sys
import gurobipy as gp
from gurobipy import GRB


INF = float('inf')

def save_solution(path, m, tr_ops, tr_op_sucs, x, y, t):
        sol = {
            "status": m.Status,
            "objective": m.ObjVal,
            "x": [[tr, op, round(t[(tr, op)].X)]
                for (tr, op) in tr_ops if x[(tr, op)].X > 0.5],
            "y": [[tr, a, b]
                for (tr, a, b) in tr_op_sucs if y[(tr, a, b)].X > 0.5],
        }
        with open(path, "w") as f:
            json.dump(sol, f)

def get_all_edges(tr_op, trains):
    tr_op_suc = []
    for tr_op_val in tr_op:
        successors = trains[tr_op_val[0]][tr_op_val[1]]['sucs']
        for suc in successors:
            tr_op_suc.append((tr_op_val[0], tr_op_val[1], suc))
    return tr_op_suc

def get_prev_ops(tr_ops, tr_op_sucs):
    prev_ops = {}
    for (tr, op) in tr_ops:
        prev_ops[(tr, op)] = []
    for (tr, op, suc) in tr_op_sucs:
        prev_ops[(tr, suc)].append(op)
    return prev_ops

def get_big_m(trains):
    max_lb, max_release_time, min_duration_sum = 0, 0, 0
    for ops in trains:
        for op in ops:
            max_lb = max(op['lb'], max_lb)
            min_duration_sum += op['min_duration']
            for res_time in op['res'].values():
                max_release_time = max(max_release_time, res_time)
    return max_lb + len(trains) * max_release_time + min_duration_sum



# the list of pairs of 2 train operations which share same resource 
def get_resource_ops(trains):
    resource_tr_ops = {}
    z = set()
    res_ops = set()
    for train_id, train in enumerate(trains):
        for op_id, op in enumerate(train):
            for r_name in op['res']:
                resource_tr_ops.setdefault(r_name, []).append((train_id, op_id))
    for res, lst in resource_tr_ops.items():
        for (tr1, op1) in lst:
            for (tr2, op2) in lst:
                if tr1 < tr2:
                    z.add((tr1, op1, tr2, op2))
                    z.add((tr2, op2, tr1, op1))
                    res_ops.add((tr1, op1, tr2, op2))
    return sorted(z), res_ops

def detect_conflicts(tr_op_sucs, y, trains, t):
    res_set = {}
    for (tr, op1, op2) in tr_op_sucs:
        if y[(tr, op1, op2)].X >= 0.5:
            for res_id, res_val in trains[tr][op1]['res'].items():
                start_t = round(t[(tr, op1)].X)
                end_t = round(t[(tr, op2)].X) + res_val
                res_set.setdefault(res_id, []).append((tr, op1, start_t, end_t))
    z_container = set()
    for res_list in res_set.values():
        for (tr1, op1, st1, ed1) in res_list:
            for (tr2, op2, st2, ed2) in res_list:
                if tr1 < tr2 and st1 < ed2 and st2 < ed1:
                    z_container.add((tr1, op1, tr2, op2))
    return sorted(z_container)

def add_resource_constraints(conflicts, trains, m, t, x, z, u, M_u, M, tr_op_sucs, y):
    for (tr1, op1, tr2, op2) in conflicts:
        z[(tr1, op1, tr2, op2)] = m.addVar(vtype=GRB.BINARY)
        z[(tr2, op2, tr1, op1)] = m.addVar(vtype=GRB.BINARY)
        ops1 = trains[tr1][op1]
        ops2 = trains[tr2][op2]
        shared_res = ops1['res'].keys() & ops2['res'].keys()
        for res_id in shared_res:
            for op_suc1 in ops1['sucs']:
                m.addConstr(t[(tr1, op_suc1)] + ops1['res'][res_id] <= t[(tr2, op2)] + M * (1 - z[(tr1, op1, tr2, op2)]))  # + 1 - y[(tr1, op1, op_suc1)] + 
            for op_suc2 in ops2['sucs']:
                m.addConstr(t[(tr2, op_suc2)] + ops2['res'][res_id] <= t[(tr1, op1)] + M * (1 - z[(tr2, op2, tr1, op1)]))  # 1 - y[(tr2, op2, op_suc2)]

        for op_suc1 in ops1['sucs']:
            m.addConstr(u[(tr1, op_suc1)] + 1 <= u[(tr2, op2)] + M_u * (1 - z[(tr1, op1, tr2, op2)]))
        for op_suc2 in ops2['sucs']:
            m.addConstr(u[(tr2, op_suc2)] + 1 <= u[(tr1, op1)] + M_u * (1 - z[(tr2, op2, tr1, op1)]))
        m.addConstr(x[(tr1, op1)] >= z[(tr1, op1, tr2, op2)] + z[(tr2, op2, tr1, op1)]) # Zab+Zba is at most 1
        m.addConstr(x[(tr2, op2)] >= z[(tr1, op1, tr2, op2)] + z[(tr2, op2, tr1, op1)])
        m.addConstr(x[(tr1, op1)] + x[(tr2, op2)] - 1 <= z[(tr1, op1, tr2, op2)] + z[(tr2, op2, tr1, op1)]) # if both x are 1, it forces one of the z be 1


def parse(path):
    prob = path
    with open(prob) as f:
        raw = json.load(f)
    trains = []
    last_tr_ops = []
    tr_ops = []
    for tr_ind, tr in enumerate(raw['trains']):
        ops = []
        for op_ind, op in enumerate(tr):
            ops.append({
                'lb': op.get('start_lb', 0),
                'ub': op.get('start_ub', INF),
                'min_duration': op.get('min_duration', 0),
                'res': {r['resource']: r.get('release_time', 0) for r in op.get('resources', [])},
                'sucs': op.get('successors', [])
            })
            tr_ops.append((tr_ind, op_ind))
            if len(ops[-1]['sucs']) == 0:
                last_tr_ops.append((tr_ind, op_ind))
        trains.append(ops)
    objectives = raw['objective']
    objs = []
    for obj in objectives:
        objs.append({
            'tr': obj['train'],
            'op': obj['operation'],
            'threshold': obj.get('threshold', 0),
            'coeff': obj.get('coeff', 0)
        })
    return trains, objs, tr_ops, last_tr_ops

def solve(path):
    trains, objs, tr_ops, last_tr_ops = parse(path)
    tr_op_sucs = get_all_edges(tr_ops, trains)
    prev_ops = get_prev_ops(tr_ops, tr_op_sucs)
    m = gp.Model('DISPLIB_MILP')
    m.Params.OutputFlag = 0
    M = get_big_m(trains)
    M_u = len(tr_ops)
    t = m.addVars(tr_ops, vtype=GRB.INTEGER, name="time")
    x = m.addVars(tr_ops, vtype=GRB.BINARY, name="operation")
    y = m.addVars(tr_op_sucs, vtype=GRB.BINARY, name="edge")
    w = m.addVars(len(objs), lb = 0, vtype=GRB.CONTINUOUS, name="penalty")
    z = {}
    u = m.addVars(tr_ops, vtype=GRB.INTEGER, name="order")

    # first operation (start)
    for tr_ind in range(len(trains)):
        m.addConstr(gp.quicksum(y[(tr_ind, 0, suc)] for suc in trains[tr_ind][0]['sucs']) == 1)

    # last operation constraint (end)
    for (last_tr, last_op) in last_tr_ops:
        m.addConstr(gp.quicksum(y[(last_tr, pred, last_op)] for pred in prev_ops[(last_tr, last_op)]) == 1) 

    # routing constraint
    for (tr, op, suc) in tr_op_sucs:
        yab = y[(tr, op, suc)]
        xa = x[(tr,op)]
        xb = x[(tr, suc)]
        m.addConstr(yab <= xa)
        m.addConstr(yab <= xb)
        m.addConstr(xa+xb-1<=yab)
        m.addConstr(t[(tr, suc)] >= t[(tr, op)] + trains[tr][op]['min_duration'] * yab)
        m.addConstr(u[(tr, op)] + 1 <= u[(tr, suc)] + M_u * (1 - y[(tr, op, suc)])) # if successive operation is next it has to be have order number more than previous

    # objective
    for k in range(len(objs)):
        tr = objs[k]['tr']
        op = objs[k]['op']
        m.addConstr(w[k] >= objs[k]['coeff'] * (t[(tr, op)] - objs[k]['threshold']))

    # flow constraint (in-out) + time_constraint (start_lb <= time <= start_ub)
    for (tr, op) in tr_ops:
        t[(tr, op)].LB = trains[tr][op]['lb']
        if trains[tr][op]['ub'] != INF:
            t[(tr, op)].UB = trains[tr][op]['ub']
        if len(trains[tr][op]['sucs']) == 0 or len(prev_ops[(tr, op)]) == 0:
            continue
        prev = prev_ops[(tr, op)]
        post = trains[tr][op]['sucs']
        # prev - list of previous operations, post - list of successor operations
        m.addConstr(gp.quicksum(y[tr, prev_op, op] for prev_op in prev) == gp.quicksum(y[tr, op, post_op] for post_op in post)) 

    # resource constraints
    

    m.setObjective(gp.quicksum(w[i] for i in range(len(objs))), GRB.MINIMIZE)


    m.Params.MIPGap = 0.05      # stop when proven within 5%
    m.Params.TimeLimit = 300    # only 300 seconds for run
    added = set()
    while True:
        bug = False
        m.optimize()
        if m.Status == GRB.INFEASIBLE or m.SolCount == 0:
            print('🚨 ERROR: The model is mathematically impossible (Infeasible). Check your constraints!')
            break
        conflicts = detect_conflicts(tr_op_sucs, y, trains, t)
        print(f'first solution goes with {len(conflicts)} conflicts')
        save_solution("solution.json", m, tr_ops, tr_op_sucs, x, y, t)
        if len(conflicts) == 0:
            print('found optimal solution')
            break
        for conflict in conflicts:
            if conflict in added:
                print('bug')
                bug = True
                break
        if bug:
            break
        add_resource_constraints(conflicts, trains, m, t, x, z, u, M_u, M, tr_op_sucs, y)
        added.update(conflicts) 
        



if __name__ == '__main__':
    solve(sys.argv[1])
    
