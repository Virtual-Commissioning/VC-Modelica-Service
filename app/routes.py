import json
from flask import request

from app import app
from app.services import modelica_simulation_service


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

@app.route('/run_modelica_simulation', methods=['POST'])
def run_modelica_simulation():
    data = request.get_data()
    results_from_modelica_simulation = modelica_simulation_service.convert_simulate_modelica(data)
    return results_from_modelica_simulation


@app.route('/create_modelica_model', methods=['POST'])
def create_modelica_model():
    
    data = request.get_data()

    # Get data
    data_parsed = json.loads(data)
    system = modelica_simulation_service.extract_components_from_data(data_parsed) # Extract system from data

    # Needs info on package/model name and simulation parameters (days)
    package_name = "Auto_Generated"
    model_name = "Model"
    days = 1

    results_from_modelica_simulation = modelica_simulation_service.convert_simulate_modelica(data)
    return results_from_modelica_simulation