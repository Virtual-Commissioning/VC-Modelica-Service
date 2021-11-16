import json
import os
import time
from app.mappers.modelica_mapper import map_to_modelica_model, create_modelica_package
from buildingspy.simulate.Simulator import Simulator
from buildingspy.io.outputfile import Reader


def convert_simulate_modelica(data):
    
    start_time = time.perf_counter() # Timer

    # Get data
    data_parsed = json.loads(data)
    system = extract_components_from_data(data_parsed,["heating"]) # Extract system from data

    # Needs info on package/model name and simulation parameters (days)
    package_name = "Auto_Generated"
    model_name = "Model"
    days = 1

    modelica_package = create_modelica_package(package_name)
    modelica_model = map_to_modelica_model(system,days,package_name,model_name)
    
    pa_path = f"temp\\{package_name}"
    if not os.path.exists(pa_path):
        os.makedirs(pa_path)
    pa_fp = os.path.join(pa_path,"package.mo")
    with open(pa_fp, "w") as pa_file:
        pa_file.write(modelica_package)
        pa_file.close()

    mo_fp = os.path.join(pa_path,f"{model_name}.mo")
    with open(mo_fp, "w") as mo_file:
        mo_file.write(modelica_model)
        mo_file.close()

    model_creation_time = time.perf_counter()- start_time

    # Simulate model
    solver = "dassl"
    simulation_model = f"{package_name}.{model_name}"
    output_dir = f"temp\\{package_name}_Output"
    package_path = f"temp\\{package_name}"

    simulate_modelica_model(days,output_dir,package_path,solver,simulation_model)

    model_simulation_time = time.perf_counter() - start_time - model_creation_time
    # Read results file
    mat_file = os.path.join(output_dir,f"{model_name}.mat")
    
    results = read_simulation_results(system,mat_file,output_dir)
    results_reading_time = time.perf_counter() - start_time - model_simulation_time
    total_time = time.perf_counter() - start_time
    print(f"Model creation time: \t {model_creation_time}")
    print(f"Model simulation time: \t {model_simulation_time}")
    print(f"Results reading time: \t {results_reading_time}")
    print(f"Total time: \t {total_time}")
    
    return results

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