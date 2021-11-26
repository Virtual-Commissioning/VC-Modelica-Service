import json
import os
from flask import request
import time

from app import app
from app.services import modelica_simulation_service
from app.mappers.modelica_mapper import map_to_modelica_model

@app.route('/')
@app.route('/index')
def index():
    return '''
<html>
    <head>
        <title>MSS4VC</title>
    </head>
    <body>
        <h2>Welcome to MSS4VC! Find the source code on <a href="https://github.com/Virtual-Commissioning/VC-Modelica-Service/">GitHub</a>!</h2>
    </body>
</html>'''


@app.route('/create_modelica_model', methods=['POST'])
def create_modelica_model():
    start_time = time.perf_counter()
    data = request.get_data()

    # Get data
    data_parsed = json.loads(data)
    wanted_systems = data_parsed["systems"]
    system = modelica_simulation_service.extract_components_from_data(data_parsed, wanted_systems) # Extract system from data
    
    # Needs info on package/model name and simulation parameters (days)
    package_name = "Auto_Generated"
    model_name = "Model"
    days = 1

    # modelica_package = create_modelica_package(package_name)
    modelica_model, modelica_package = map_to_modelica_model(system,days,package_name,model_name)

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

    duration = round(time.perf_counter() - start_time,2)
    dict_output = {
        "build_time [s]" : duration,
        "package": modelica_package,
        "model": modelica_model
    }

    json_output = json.dumps(dict_output,indent=4)
    return json_output