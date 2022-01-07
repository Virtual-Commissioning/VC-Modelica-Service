import app.mappers.modelica_component_classes as cl

def map_to_modelica_model(system,rooms,days,package_name = "Auto_Generated", model_name = "Model"):
   
    model = cl.ModelicaModel(package_name, model_name)

    model.add_components(system)
    
    model.add_rooms(rooms)

    model.create_modelica_model()

    mo_file = model.model_string

    pa_file = model.package_string

    return mo_file, pa_file, model