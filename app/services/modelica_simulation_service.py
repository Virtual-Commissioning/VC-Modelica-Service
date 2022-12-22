import json
import os
from buildingspy.simulate.Simulator import Simulator
from buildingspy.io.outputfile import Reader

from app.mappers.modelica_component_classes import ModelicaModel, MS4VCObject, Room

def extract_components_from_data(data,wanted_systems):

    components = []
    
    for sys in wanted_systems:
        components += data["system"]["SubSystems"][sys]["Components"]
    
    return components

def simulate_modelica_model(sim_start,sim_stop,output_dir,package_path, solver="dassl",model = "Auto_Generated.Model"):
    '''
    sim_start:      start day of simulation - can be set to negative, if an initialization period is wanted
    sim_stop:       duration of simulation in days
    output_dir:     directory of simulation results
    package_path:   path of package
    solver:         choose numerical solver (default: dassl)
    model:          name of simulated model (default: Auto_Generated.Model)
    '''
    # Parameters:
    duration = 24*60*60*(sim_stop-sim_start) # set duration in seconds based on input
    intervals = duration/(3600/6) # define number of intervals in output - every 10 minutes

    s=Simulator(model, "dymola",outputDirectory=output_dir,packagePath=package_path) # instantiate model for simulation

    s.setStartTime(sim_start*24*3600) # set start time of simulation
    s.setStopTime(sim_stop*24*3600) # set stop time of simulation
    s.setSolver(solver) # set solver - default is DASSL
    s.setNumberOfIntervals(int(intervals)) # define number of intervals in output
    s.showGUI(show=True)  # show Dymola GUI when simulating
    print("Starting simulation")
    s.simulate() # simulate model
    print("Simulation done!")
    return

def read_simulation_results(system: ModelicaModel,mat_file):
    # Read results from
    r = Reader(mat_file, "dymola")

    # Import needed results keys:    
    fp = os.path.join(os.path.dirname(__file__), '..\\static\\result_keys.json')
    with open(fp, "r") as res_keys_file:
        res_keys = json.load(res_keys_file)
        res_keys_file.close()

    results = {}
    i = 0
    for object in system.components.values():
        object: MS4VCObject
        # get needed results for component based on it's class
        FSC_object = object.FSC_object
        if FSC_object == None:
            continue
        
        keys = res_keys[FSC_object["ComponentType"]]
        
        comp_results = {}
        for key in keys:
            (t,y) = r.values(f'''{object.modelica_name}.{key}''')
            comp_results[key] = dict(zip(t.tolist(),y.tolist()))
        
        results[FSC_object["Tag"]] = comp_results
        
        i+=1
    for room in system.rooms.values():
        room: Room
        keys = res_keys["Room"]
        comp_results = {}
        for key in keys:
            (t,y) = r.values(f'''{room.modelica_name}.{key}''')
            comp_results[key] = dict(zip(t.tolist(),y.tolist()))
        
        results[room.name] = comp_results
    results_json = json.dumps(results, indent = 4)
    return results