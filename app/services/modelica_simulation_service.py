import json
import os
from buildingspy.simulate.Simulator import Simulator
from buildingspy.io.outputfile import Reader

def extract_components_from_data(data,wanted_systems):

    components = []
    
    for sys in wanted_systems:
        components += data["system"]["SubSystems"][sys]["Components"]
    
    return components

def simulate_modelica_model(days,output_dir,package_path, solver="dassl",model = "Auto_Generated.Model"):
    # Parameters:
    duration = 24*60*60*days # set duration in seconds based on input
    intervals = duration/(3600/6) # define number of intervals in output - every 10 minutes

    s=Simulator(model, "dymola",outputDirectory=output_dir,packagePath=package_path) # instantiate model for simulation

    s.setStartTime(-24*3600) # set start time to -1 days for initialization
    s.setStopTime(duration) # set stop time of simulation
    s.setSolver(solver) # set solver - default is DASSL
    s.setNumberOfIntervals(int(intervals)) # define number of intervals in output
    s.showGUI(show=True)  # show Dymola GUI when simulating
    print("Starting simulation")
    s.simulate() # simulate model
    print("Simulation done!")
    return

def read_simulation_results(system,mat_file,output_dir):
    # Read results from
    r = Reader(mat_file, "dymola")

    # Import needed results keys:    
    fp = os.path.join(os.path.dirname(__file__), '..\\static\\result_keys.json')
    with open(fp, "r") as res_keys_file:
        res_keys = json.load(res_keys_file)
        res_keys_file.close()

    results = {}
    i = 0
    for comp in system:
        # get needed results for component based on it's class
        if comp["ComponentType"] != "FlowController" and comp["ComponentType"] != "FlowMovingDevice":
            keys = res_keys[comp["ComponentType"]]
        elif comp["ComponentType"] == "FlowMovingDevice":
            keys = res_keys[comp["ComponentType"]][comp["control_type"]]
        elif comp["ComponentType"] == "FlowController":
            keys = res_keys[comp["ComponentType"]][comp["ValveType"]]
        
        comp_results = {}
        for key in keys:
            (t,y) = r.values(f'''c{comp["Tag"]}.{key}''')
            comp_results[key] = dict(zip(t.tolist(),y.tolist()))
        
        results[comp["Tag"]] = comp_results
        
        i+=1
    results_json = json.dumps(results, indent = 4)
    return results_json