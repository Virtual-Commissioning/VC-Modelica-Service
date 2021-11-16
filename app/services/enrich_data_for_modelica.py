import json
import math

def add_properties(system,fp_prop = "Data\properties.json"):

    with open(fp_prop, "r") as prop_file:
        properties = json.load(prop_file)
        prop_file.close()

    classes = properties.keys()

    # Write default properties to system
    for component in system: # Iterate components in system
        for c in classes: # Iterate classes from properties file
            if component["ComponentType"] == c: # If component matches class
                for prop in properties[c]: # Iterate properties
                    if prop not in component.keys() or component[prop] == None: # If property is not already in component or it is null, create it with default value.
                        component[prop] = properties[c][prop]
    return system
def add_lengths(system):
    '''
    Calculates length of components in system, that include the key Length.
    '''
    for component in system:
        if "length" in component.keys() and component["length"] == None and None not in component["ConnectedWith"]:
            len_X = component["ConnectedWith"][0]["Coordinates"]["X"] - \
                component["ConnectedWith"][1]["Coordinates"]["X"]
            len_Y = component["ConnectedWith"][0]["Coordinates"]["Y"] - \
                component["ConnectedWith"][1]["Coordinates"]["Y"]
            len_Z = component["ConnectedWith"][0]["Coordinates"]["Z"] - \
                component["ConnectedWith"][1]["Coordinates"]["Z"]
            component["length"] = round(math.sqrt(len_X**2+len_Y**2+len_Z**2),2)
        elif "length" in component.keys() and component["length"] == None and None in component["ConnectedWith"]:
            component["length"] = 0.125
    return system
def add_port_names(system):
    '''
    Adds name of in- and out ports to components and their connectors.
    '''
    # Add info on class and port name in components
    for component in system:
        if component["ComponentType"] == "Tee":
            component["inport_name"] = "port_1"
            component["secondaryport_name"] = "port_3"
            component["outport_name"] = "port_2"
        elif component["ComponentType"] == "HeatExchanger":
            component["inport_name"] = "port_a2"
            component["outport_name"] = "port_b2"
        else:
            component["inport_name"] = "port_a"
            component["outport_name"] = "port_b"

    # Add port names in connectors
    for component in system:
        for con in component["ConnectedWith"]:
            if con != None:
                if con["Tag"] != "Not connected":
                    connector_type = [c["ComponentType"] for c in system if c["Tag"] == con["Tag"]][0]
                    if connector_type == "Tee":
                        con["inport_name"] = "port_1"
                        con["secondaryport_name"] = "port_3"
                        con["outport_name"] = "port_2"
                    elif connector_type == "HeatExchanger":
                        con["inport_name"] = "port_a2"
                        con["outport_name"] = "port_b2"
                    else:
                        con["inport_name"] = "port_a"
                        con["outport_name"] = "port_b"
                else:
                    con["inport_name"] = None
                    con["outport_name"] = None
    return system
def enrich(system):
    # system = add_properties(system)
    system = add_lengths(system)
    system = add_port_names(system)
    return system