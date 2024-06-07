import matplotlib.pyplot as plt

# USE WITH 
# top -d 5 -n 10 -i -b>top.txt

# -d 5: 5 seconds delay
# -n 10: 10 times
# -i: ignore idle processes


# Define the file path
file_path = "/home/sentry_ws/top.txt"

proccess_name=["dll_node","livox_ros_drive","bt_navigator","controller_serv","pointlio","bt_navigator","terrainAnalysis","lifecycle_manag","planner_server","behavior_server","smoother_server"]

all_cpu_usage = []
all_mem_usage = []

with open(file_path, "r") as file:
    lines = file.readlines()
    
    for name in proccess_name:
        cpu_usage = []
        mem_usage = []
        for line in lines:
            if name in line:
                values = line.split()
                cpu_usage.append(float(values[8]))
                mem_usage.append(float(values[9]))
        
        all_cpu_usage.append(cpu_usage)
        all_mem_usage.append(mem_usage)
        

plt.figure(figsize=(10, 5))
for i in range(len(proccess_name)):
    plt.plot(all_cpu_usage[i], label=proccess_name[i]+' CPU')    
    
plt.xlabel("Time")
plt.ylabel("CPU Usage (%)")
plt.title("CPU Usage")
plt.legend()
plt.show()

plt.figure(figsize=(10, 5))
for i in range(len(proccess_name)):
    plt.plot(all_mem_usage[i], label=proccess_name[i]+' Memory')
    
plt.xlabel("Time")
plt.ylabel("Memory Usage (MB)")
plt.title("Memory Usage")
plt.legend()
plt.show()
