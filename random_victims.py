import numpy as np
import math
import yaml
grid = 35
length = 300
height = 20
angle = 7
depth = 5
slope = math.tan(math.radians(-angle))
n_victims = 10

#generate the waypoints of searching field,current field [-385,0][0,300]
x = -grid*np.arange(0,12)
x = np.repeat(x,2)
y = np.array([1,0])
y = length*y
y = np.repeat(y,2)
y = np.tile(y,6)
for i in range(len(y)-1, -1, -1):
    y[i] = y[i-1]

coordinate = np.zeros((len(x), 3))
coordinate[:,0] = x
coordinate[:,1] = y
b = height - slope*coordinate[0,0]
coordinate[:,2] = slope*x + b
coordinate[0,1] = -35.0
coordinate = coordinate.tolist()
# print(coordinate)

#write field waypoints
with open("./src/basic_waypoint_pkg/config/trajectory_config.yaml", "r") as yaml_file:
    yaml_obj = yaml.safe_load(yaml_file.read())
    previous_coordinates = yaml_obj["waypoints"]['position']
    # print(previous_coordinates)
yaml_file.close()

with open("./src/basic_waypoint_pkg/config/trajectory_config.yaml", "w") as yaml_file:
    yaml_obj["waypoints"]['position'] = coordinate
    yaml.dump(yaml_obj, yaml_file)
yaml_file.close()

#generate victim positions, victims are spread over a larger area than the field
victim_coordiantes = np.zeros((n_victims,3))
victim_x = 415*np.random.rand(n_victims) - 400 #15,-400
victim_y = 330*np.random.rand(n_victims) - 15 #-15,315
victim_z = slope*victim_x + b - depth
victim_coordiantes[:,0] = victim_x
victim_coordiantes[:,1] = victim_y
victim_coordiantes[:,2] = victim_z
victim_coordiantes = victim_coordiantes.tolist()

#write victim postions
with open("./src/sensor_model/config/victims_location.yaml", "r") as yaml_file:
    yaml_obj = yaml.safe_load(yaml_file.read())
    previous_coordinates = yaml_obj["victims"]['position']

with open("./src/sensor_model/config/victims_location.yaml", "w") as yaml_file:
    yaml_obj["victims"]['position'] = victim_coordiantes
    yaml.dump(yaml_obj, yaml_file)

with open("./src/basic_waypoint_pkg/config/trajectory_config.yaml", "r") as yaml_file:
    yaml_obj = yaml.safe_load(yaml_file.read())
    previous_coordinates = yaml_obj["victims"]['position']
    # print(previous_coordinates)

with open("./src/basic_waypoint_pkg/config/trajectory_config.yaml", "w") as yaml_file:
    yaml_obj["victims"]['position'] = victim_coordiantes
    yaml.dump(yaml_obj, yaml_file)
