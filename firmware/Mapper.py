import folium 
from folium.plugins import HeatMap
import pandas as pd

def create_map (map_file:str) :
    datafile = pd.read_csv(map_file)
    datafile = datafile.dropna(subset=["lat","long","intensity"])
    heat_data = datafile[["lat","long","intensity"]].values.tolist()
    map_center = [datafile["lat"].mean(), datafile["long"].mean()]
    visual_map = folium.Map(location=map_center,zoom_start=9,tiles="openstreetmap")

    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Light_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        attr="Esri, HERE, Garmin",
        name="Esri Light Gray",
    ).add_to(visual_map)
    
    HeatMap(
        data=heat_data,
        radius = 15,
        blur = 10,
        max = float(datafile["intensity"].max())
        ).add_to(visual_map)

    visual_map.save("heatmap.html")

create_map("log.csv")
print("Running.....")
