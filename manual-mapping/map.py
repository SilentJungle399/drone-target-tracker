from flask import Flask, render_template
from flask_socketio import SocketIO
import folium
import json
from geopy.distance import geodesic

app = Flask(__name__)
socketio = SocketIO(app)

gps_path = []  # Store GPS coordinates to draw the track
with open("gps_path.json") as f:
    gps_path = json.load(f)

last = [29.9495986, 76.8161272]

# if len(gps_path) == 0:
#     gps_path = [[29.9495986, 76.8161272]]  # Default to origin

def create_map():
    m = folium.Map(location=[29.9495986, 76.8161272], zoom_start=19)  # Default until first GPS fix
    for coord in gps_path:
        folium.Marker(coord[:2], popup=str(coord[:2]), icon=folium.Icon(color="blue" if coord[2] == False else "black")).add_to(m)
    return m

@app.route('/')
def index():
    m = create_map()
    m.save("templates/map.html")
    return render_template('index.html')

@app.route('/map')
def map():
    # m = create_map()
    # m.save("templates/map.html")
    # return render_template('map.html')
    with open("templates/map.html") as f:
        return f.read()

@socketio.on('connect')
def handle_connect():
    print("Client Connected")

@socketio.on('new_gps')
def handle_new_gps(data):
    global gps_path, last
    lat, lon, det = data["lat"], data["lon"], data["bullseye"]
    print(f"Received GPS: {lat}, {lon}", end=" ")

    m = folium.Map(location=[lat, lon], zoom_start=19)

    if len(gps_path) == 0:
        gps_path.append([lat, lon])

    folium.Marker(gps_path[0][:2], popup="Start Position", icon=folium.Icon(color="lightgreen")).add_to(m)
    folium.Marker((lat, lon), popup="Current Position", icon=folium.Icon(color="red")).add_to(m)

    for coord in gps_path[1:]:
        folium.Marker(coord[:2], popup=str(coord[:2]), icon=folium.Icon(color="blue" if coord[2] == False else "black")).add_to(m)

    # if len(gps_path) > 1:
    #     folium.PolyLine(gps_path, color="blue", weight=3).add_to(m)

    m.save("templates/map.html")

    if len(gps_path) > 0:
        # distance = geodesic(last, (lat, lon)).meters
        distance = geodesic(gps_path[-1][:2], (lat, lon)).meters
        print(f"| Distance: {distance:.2f} meters")

        if distance >= 3 and det:
            gps_path.append([lat, lon, det])

    with open("gps_path.json", "w") as f:
        json.dump(gps_path, f, indent=4)

    last = [lat, lon]

    # Notify clients to refresh
    socketio.emit('update_map')

if __name__ == '__main__':
    print("Running at 0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
