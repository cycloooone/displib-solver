import json
import os  


'''
Naive route building by selecting FIRST successor from the list
'''
def build_naive_route(train):
    route = [0]
    index = 0
    while train[index]['successors']:     # last operation has no successors
        index = train[index]['successors'][0]
        route.append(index)
    return route


def get_departure_time(train_id, operation, resource_avaiable_time, train_prev_time, train_prev_duration):
    resources = operation.get('resources', [])
    time = max(operation.get('start_lb', 0), train_prev_time + train_prev_duration)
    for resource in resources:
        resource_index = resource['resource']
        resource_time_info = resource_avaiable_time.get(resource_index, (0, 0))
        if resource_time_info[0] != train_id:
            time = max(time, resource_time_info[1])
    return time

def depart_train(trains, routes, free_at, position, last_time):
    min_time = float('inf')
    winner = -1

    for i, train in enumerate(trains):
        pos = position[i]
        if pos == len(routes[i]):
            continue                      # train has reached its exit

        op_index = routes[i][pos]
        prev_duration = 0
        if pos != 0:
            prev_op_index = routes[i][pos-1]
            prev_duration = train[prev_op_index].get('min_duration', 0)

        t = get_departure_time(i, train[op_index], free_at, last_time[i], prev_duration)
        if t > 1e9:
            print(t)
        if t < min_time:
            min_time = t
            winner = i

    if min_time == float('inf'):
        print('strage', winner)

    if winner == -1:
        print("no winner; positions:", position, "route lengths:", [len(r) for r in routes])
        print('r5 ->', free_at.get('r5'))
        print('r6 ->', free_at.get('r6'))
        for i, ind in enumerate(position):
            if ind < len(routes[i]) - 1:
                print(i, routes[i][ind])
                print(trains[i][routes[i][ind]])
                print()
            else:
                print(i, routes[i][ind - 1])
                print(trains[i][routes[i][ind - 1]])
                print('finished')
                print()
                
        return (0, {})
    pos = position[winner]
    position[winner] = pos + 1
    op_index = routes[winner][pos]

    if pos != 0:
            prev_op = routes[winner][pos - 1]
            for r in trains[winner][prev_op].get('resources', []):
                free_at[r['resource']] = (winner, min_time + r.get('release_time', 0))

    for r in trains[winner][op_index].get('resources', []):
        free_at[r['resource']] = (winner, float('inf'))

    last_time[winner] = min_time
    



    

    event = {'time': min_time, 'train': winner, 'operation': op_index}
    return (1, event)



def schedule_naive_route(train, train_id, naive_route, min_start):
    events = []
    time = 0
    prev_duration = 0
    for operation_index in naive_route:
        operation = train[operation_index]
        earliest = time + prev_duration
        if operation_index != 0:
            earliest = max(earliest, min_start)
        time = max(earliest, operation.get('start_lb', 0))
        prev_duration = operation.get('min_duration', 0)
        event = {"time": time, "train": train_id, "operation": operation_index}
        events.append(event)
    return events

def count_objective(event_lookup, objective_data): 
    objective_sum = 0
    for objective in objective_data:
        event_time = event_lookup.get((objective['train'], objective['operation']))
        threshold = objective.get('threshold', 0)
        if event_time is not None and threshold <= event_time:
            objective_sum += objective.get("coeff", 0) * (event_time - threshold) + objective.get("increment", 0)
    return objective_sum


directory = 'problems'
os.makedirs('solutions_mine', exist_ok=True)
for entry in os.scandir(directory):  
    if entry.is_file():  
        if entry.name != 'nor1_critical_4.json':
            continue
        with open(entry) as f:
            data = json.load(f)
            events = []
            min_start = 0
            train_routes = []
            free_at = {}
            position = [0] * len(data['trains'])
            last_time = [0] * len(data['trains'])
            for train_id, train in enumerate(data['trains']):
                naive_route = build_naive_route(train)
                train_routes.append(naive_route)

            for i in train_routes:
                for j in i:
                    event = depart_train(data['trains'], train_routes, free_at, position, last_time)
                    if event[0] == 1:
                        events.append(event[1])
            total = sum(len(r) for r in train_routes)

            
            
            
            events.sort(key=lambda e: e["time"])
            event_lookup = {(e['train'], e['operation']): e['time'] for e in events}
            objective = count_objective(event_lookup, data['objective'])
            stem = entry.name.replace('.json', '')
            out_path = os.path.join('solutions_mine', stem + '.json')
            json.dump({"objective_value": objective, "events": events}, open(out_path, 'w'))


