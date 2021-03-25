import yaml
import os

class Trajectory:
    def __init__(self, trajectory_name):
        self.trajectory_name = trajectory_name.lower()

        with open("trajectories_conf.yaml", 'r') as conf_file_reader:
            self.trajectories_paths = yaml.safe_load(conf_file_reader) 

    def start(self):
        print("Start")
        print(f"Name: {self.trajectory_name}")
        if self.trajectory_name in self.trajectories_paths:
            trajectory_filename = self.trajectories_paths['basepath'] + self.trajectories_paths[self.trajectory_name]
           
#            with open(trajectory_filename, 'rb') as script:
#               code = compile(script.read(), trajectory_filename, 'exec')
#            exec(code)
            os.system(f"python3 {trajectory_filename}")
        else:
            print("ERROR. Unknown trajectory")

if __name__ == "__main__":
    test = Trajectory("Test")
    test.start()
