import pandas as pd
import numpy as np
import requests

file_path = 'data/distances.xlsx'
df = pd.read_excel(file_path)

addresses = df['Address'].tolist()
def get_coordinates(address):
    url = f"https://maps.googleapis.com/maps/api/geocode/json?address={address}&key={api_key}"
    response = requests.get(url)
    if response.status_code == 200:
        results = response.json().get('results')
        print(results)
        if results:
            location = results[0]['geometry']['location']
            return (location['lat'], location['lng'])
    return None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0  
    lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = R * c
    return distance

# df['Latitude'], df['Longitude'] = zip(*df['Address'].apply(get_coordinates))

target_lat = 21.01466
target_lon = 105.8518
api_key = 'MAPS API KEY'

df['Coordinates'] = df['Address'].apply(get_coordinates)
df['Distances'] = df['Coordinates'].apply(lambda x: haversine(x[1], x[0], target_lat, target_lon))
df[['Address', 'Coordinates', 'Distances']].head()
output_path = 'data/distances.csv'
df.to_csv(output_path, index=False)
