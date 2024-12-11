import matplotlib.pyplot as plt
from flask import Flask, request, make_response
import io
import base64

app = Flask(__name__)

x = []
y = []
i = 1

@app.route("/")
def print_light():
    # Get the light parameter from the URL
    light_value = request.args.get("light", "0")  # Default to "0" if not provided
    try:
        light_value = float(light_value)  # Convert to a number for plotting
    except ValueError:
        return "<h1>Invalid light value. Please provide a numeric value.</h1>"
    
    # Create the plot based on the light value
    global i
    x.append(i)  # Example x-axis data
    y.append(light_value)# Scale y-axis data by the light value
    i += 1
    
    plt.figure()
    plt.plot(x, y)
    plt.title("Max Light Plot")
    plt.xlabel("Time Elapsed")
    plt.ylabel("Light Value")
    
    # Save the plot to a BytesIO buffer
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    buf.seek(0)
    plt.close()
    
    # Encode the image to base64 and embed in HTML response
    encoded_image = base64.b64encode(buf.read()).decode('utf-8')
    image_html = f'<img src="data:image/png;base64,{encoded_image}"/>'
    
    return f"<h1>The max light value is: {light_value}</h1>{image_html}"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
