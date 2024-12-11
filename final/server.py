import matplotlib.pyplot as plt
from flask import Flask, request, make_response
import io
import base64

app = Flask(__name__)

x = []
y1 = []
y2 = []
y3 = []

i = 0
latest_light_value = 0.0

@app.route("/data")
def data():
    
    plt.figure()
    plt.plot(x, y1, label = "Photoresistor 1")
    plt.plot(x, y2, label = "Photoresistor 2")
    plt.plot(x, y3, label = "Photoresistor 3")
    plt.legend()
    plt.xlabel("Time Elapsed in Seconds")
    plt.ylabel("Light Value")

    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    buf.seek(0)
    plt.close()
    
    # Encode the image to base64 and embed in HTML response
    encoded_image = base64.b64encode(buf.read()).decode('utf-8')
    image_html = f'<img src="data:image/png;base64,{encoded_image}"/>'

    return  f"<h1>The max light value is: {latest_light_value}</h1>{image_html}"

@app.route("/")
def print_light():
    global latest_light_value
    # Get the light parameter from the URL
    light1 = float(request.args.get("light1"))
    light2 = float(request.args.get("light2"))
    light3 = float(request.args.get("light3"))

    try:
        latest_light_value = max(light1, light2, light3) 
    except ValueError:
        return "<h1>Invalid light value. Please provide a numeric value.</h1>"
    
    # Add light values to list
    global i
    x.append(i)
    y1.append(light1)
    y2.append(light2)
    y3.append(light3)  
    i += .3

    if len(x) > 100:
        x.pop()
        y1.pop()
        y2.pop()
        y3.pop()

    return  f"<h1>The max light value is: {latest_light_value}"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
