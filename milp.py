import json
import sys
import gurobipy as gp
from gurobipy import GRB


INF = float('inf')


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
    t = m.addVars(tr_ops, vtype=GRB.INTEGER, name="time")
    x = m.addVars(tr_ops, vtype=GRB.BINARY, name="operation")
    y = m.addVars(tr_op_sucs, vtype=GRB.BINARY, name="edge")
    w = m.addVars(len(objs), lb = 0, vtype=GRB.CONTINUOUS, name="penalty")
        
    for tr_ind in range(len(trains)):
        m.addConstr(gp.quicksum(y[(tr_ind, 0, suc)] for suc in trains[tr_ind][0]['sucs']) == 1)

    for (last_tr, last_op) in last_tr_ops:
        m.addConstr(gp.quicksum(y[(last_tr, pred, last_op)] for pred in prev_ops[(last_tr, last_op)]) == 1) 

    for (tr, op, suc) in tr_op_sucs:
        yab = y[(tr, op, suc)]
        xa = x[(tr,op)]
        xb = x[(tr, suc)]
        m.addConstr(yab <= xa)
        m.addConstr(yab <= xb)
        m.addConstr(xa+xb-1<=yab)
        m.addConstr(t[(tr, suc)] >= t[(tr, op)] + trains[tr][op]['min_duration'] * yab)

    for k in range(len(objs)):
        tr = objs[k]['tr']
        op = objs[k]['op']
        m.addConstr(w[k] >= objs[k]['coeff'] * (t[(tr, op)] - objs[k]['threshold']))


    # todo: flow constraint in == out

    m.setObjective(gp.quicksum(w[i] for i in range(len(objs))), GRB.MINIMIZE)

    m.optimize()

    if m.Status == GRB.OPTIMAL:
        for (tr, op) in tr_ops:
            if x[(tr,op)].X > 0.5:
                print(x[(tr,op)], t[(tr, op)])
        for (tr, op, suc) in tr_op_sucs:
            if y[(tr, op, suc)].X > 0.5:
                print(y[(tr, op, suc)])
    elif m.Status == GRB.INFEASIBLE:
        print('🚨 ERROR: The model is mathematically impossible (Infeasible). Check your constraints!')
    

    



if __name__ == '__main__':
    solve(sys.argv[1])
    
