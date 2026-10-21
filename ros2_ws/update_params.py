import os, yaml

param_file = os.path.expanduser('~/Robotics-Midterm/ros2_ws/src/team2/config/nav2_params.yaml')
with open(param_file, 'r') as f:
data = yaml.safe_load(f)

def update_inflation(d):
if isinstance(d, dict):
    for k, v in d.items():
        if k == 'inflation_layer':
            d[k]['inflation_radius'] = 0.25
            d[k]['cost_scaling_factor'] = 10.0
        else:
            update_inflation(v)

update_inflation(data)
with open(param_file, 'w') as f:
yaml.dump(data, f)
print('nav2_params.yaml updated successfully.')
