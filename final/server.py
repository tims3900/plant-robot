from flask import Flask
from flask import request

app = Flask(__name__)

@app.route("/")

def print_light():
        print(request.args.get("light"))
        return "The max light value  is: "+str(request.args.get("light"))
