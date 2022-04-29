def map_to_modelica_model(system,wanted_systems,rooms,start_day,stop_day,package_name = "Auto_Generated", model_name = "Model"):
    import app.mappers.modelica_component_classes as cl
   
    model = cl.ModelicaModel(package_name, model_name,start_day,stop_day)

    model.add_components(system,wanted_systems)
    
    model.add_rooms(rooms)

    model.create_modelica_model()

    mo_file = model.model_string

    pa_file = model.package_string

    return mo_file, pa_file, model

def map_to_soep(system,wanted_systems,spaces,start_day,stop_day,package_name = "Auto_Generated", model_name = "Model", epw_path="Temp/DNK_SL_Copenhagen-Roskilde.AP.061700_TMYx.mos"):
    import app.mappers.modelica_templates_soep as cl
   
    idf_path = f"Temp/{model_name}.idf"
    model = cl.ModelicaModel(package_name, model_name,start_day,stop_day)

    model.add_components(system,wanted_systems,epw_path)
    
    model.add_rooms(spaces,idf_path,epw_path)

    model.create_modelica_model()

    mo_file = model.model_string

    pa_file = model.package_string

    return mo_file, pa_file, model