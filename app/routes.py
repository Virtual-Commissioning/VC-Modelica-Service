from app import app

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