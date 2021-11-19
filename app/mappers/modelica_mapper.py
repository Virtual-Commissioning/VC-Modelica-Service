import app.mappers.modelica_component_mapper as comp_mapper
import app.mappers.modelica_connector_mapper as conn_mapper
from app.services.enrich_data_for_modelica import enrich

def create_modelica_package(package_name = "Auto_Generated"):
    s = f'''within ;
package {package_name}
annotation (uses(Buildings(version="7.0.1"), Modelica(version="3.2.3"),ToolchainLib));
end {package_name};'''
    return s

def map_to_modelica_model(system,days,package_name = "Auto_Generated", model_name = "Model"):
    comp_mapper.model_name = model_name
    comp_mapper.package_name = package_name

    # Enrich/manipulate data:
    # system = enrich(system)

    component_string = ''

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
            component_string += comp_mapper.segment(component)
            #     conn_mapper.mo_file = mo_file # Used for identifying previous connections
            #     mo_file += conn_mapper.connector(component)
        elif component["ComponentType"] == "Pump":
            component_string += comp_mapper.pump(component)
        elif component["ComponentType"] == "Radiator":
            component_string += comp_mapper.radiator(component)
        elif component["ComponentType"] == "HeatExchanger":
            component_string += comp_mapper.heaCoil(component)
        elif component["ComponentType"] == "Bend":
            component_string += comp_mapper.bend(component)
        elif component["ComponentType"] == "Tee":
            component_string += comp_mapper.tee(component)
        elif component["ComponentType"] == "BalancingValve":
            component_string += comp_mapper.valve_balancing(component)
        elif component["ComponentType"] == "MotorizedValve":
            component_string += comp_mapper.valve_motorized(component)
        elif component["ComponentType"] == "ShuntValve":
            component_string += comp_mapper.valve_shunt(component)
        elif component["ComponentType"] == "Reduction":
            component_string += comp_mapper.reduction(component)
        else:
            component_string += f'''
            // Component with Tag {component["Tag"]} of type {component["ComponentType"]} not recognized.'''
        counter += 1
    
    
    # Create modelica model file

    # Write first lines:
    mo_file = comp_mapper.model_start()

    mo_file += component_string
    # Instantiation of plant
    # mo_file += comp_mapper.plant(system)
    
    # Instantiation of room
    # mo_file += comp_mapper.room()

    # Switch to connections
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

