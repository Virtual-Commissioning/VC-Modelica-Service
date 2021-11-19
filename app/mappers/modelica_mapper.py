import app.mappers.modelica_component_mapper as comp_mapper
import app.mappers.modelica_connector_mapper as conn_mapper
from app.services.enrich_data_for_modelica import enrich

def create_modelica_package(package_name = "Auto_Generated"):
    s = f'''within ;
package {package_name}
annotation (uses(Buildings(version="7.0.1"), Modelica(version="3.2.3"),ToolchainLib));
end {package_name};'''
    return s

def fill_connected_port_names(system):
    """
    Fills the port names of all connections in connections
    """
    for comp in system:
        for con in comp["ConnectedWith"]:
            if con != None and con["Tag"] != "Not connected" and con["Tag"] in [comp["Tag"] for comp in system]:
                s = "hello"
                try:
                    con["PortNames"], con["ComponentType"] = [(comp["PortNames"], comp["ComponentType"]) for comp in system if comp["Tag"] == con["Tag"]][0]
                except:
                    print(f"Port names for component {con['Tag']} could not be found.")

            else:
                con["PortNames"] = None
                con["ComponentType"] = None
    return system
        

def map_to_modelica_model(system,days,package_name = "Auto_Generated", model_name = "Model"):
    comp_mapper.model_name = model_name
    comp_mapper.package_name = package_name

    # Enrich/manipulate data:
    # system = enrich(system)

    component_string = ''
    connector_string = ''

    counter = 0 # Counter for distribution of components
    gridwidth = 9 # Width of the visual distribution of the components
    for component in system:
        comp_mapper.x_pos = counter % gridwidth
        comp_mapper.y_pos = int((counter - comp_mapper.x_pos)/gridwidth)

        if "heating" in component["SystemType"]:
            comp_mapper.medium = "MediumHeating"
        elif "ventilation" in component["SystemType"]:
            comp_mapper.medium = "MediumVentilation"
        elif "cooling" in component["SystemType"]:
            comp_mapper.medium = "MediumCooling"

        if component["ComponentType"] == "FlowSegment":
            string, port_names = comp_mapper.segment(component)
            component_string += string
            component["PortNames"] = port_names

            # conn_mapper.mo_file = connector_string # Used for identifying previous connections
            # connector_string += conn_mapper.connector(component)

        elif component["ComponentType"] == "Pump":
            string, port_names = comp_mapper.pump(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "Radiator":
            string, port_names = comp_mapper.radiator(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "HeatExchanger":
            string, port_names = comp_mapper.heaCoil(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "Bend":
            string, port_names = comp_mapper.bend(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "Tee":
            string, port_names = comp_mapper.tee(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "BalancingValve":
            string, port_names = comp_mapper.valve_balancing(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "MotorizedValve":
            string, port_names = comp_mapper.valve_motorized(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "ShuntValve":
            string, port_names = comp_mapper.valve_shunt(component)
            component_string += string
            component["PortNames"] = port_names
            
        elif component["ComponentType"] == "Reduction":
            string, port_names = comp_mapper.reduction(component)
            component_string += string
            component["PortNames"] = port_names
            
        else:
            component_string += f'''
            // Component with Tag {component["Tag"]} of type {component["ComponentType"]} not recognized.'''
        counter += 1
    
    system = fill_connected_port_names(system)

    # Create modelica model file

    # Write first lines:
    mo_file = comp_mapper.model_start()

    mo_file += component_string
    # Instantiation of plant
    # mo_file += comp_mapper.plant(system)
    
    # Instantiation of room
    # mo_file += comp_mapper.room()

    # Switch to equation part of model file
    mo_file+= '''
    equation'''
    
    # mo_file += connector_string

    # Create connectors
    # for component in system:
    #     conn_mapper.mo_file = mo_file # Used for identifying previous connections
    #     mo_file += conn_mapper.connector(component)
    
    # Add end of model
    mo_file+= comp_mapper.model_end(days)
    return mo_file

