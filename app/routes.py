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