# v2
import pandas as pd
import json
import numpy as np

# Read the Excel workbook with multiple sheets.
sheets = pd.read_excel('/home/byron/Desktop/HDMap/excel_files_from_HD_Map/DYOC_lanesv2.xlsx', sheet_name=None)

result = {"lanes": []}

# Process each sheet (each lane)
for sheet_name, df in sheets.items():
    # Meta data (assumed to be the same for the entire sheet) are extracted from the first row.
    meta = df.iloc[0]
    
    # Dynamically extract coordinate columns without assuming a fixed length.
    left_boundary_lats = df['left_boundary_lats'].dropna().tolist()
    left_boundary_lons = df['left_boundary_lons'].dropna().tolist()
    right_boundary_lats = df['right_boundary_lats'].dropna().tolist()
    right_boundary_lons = df['right_boundary_lons'].dropna().tolist()
    center_lats = df['center_lats'].dropna().tolist()
    center_lons = df['center_lons'].dropna().tolist()
    
    # Combine the individual lists into lists of [longitude, latitude] pairs.
    left_boundary_coords = [[lon, lat] for lon, lat in zip(left_boundary_lons, left_boundary_lats)]
    right_boundary_coords = [[lon, lat] for lon, lat in zip(right_boundary_lons, right_boundary_lats)]
    center_coords = [[lon, lat] for lon, lat in zip(center_lons, center_lats)]
    
    # Handle meta fields. For "connected_to", check for missing values.
    if pd.isna(meta['connected_to']):
        connected_to_val = []
    else:
        connected_to_val = [int(meta['connected_to'])]
    
    lane_data = {
        "path_id": int(meta['path_id']),
        "turn": int(meta['turn']),
        "direction": int(meta['direction']),
        "adas": {
            "tl": [int(meta['tl_0']), int(meta['tl_1'])]
        },
        "connected_to": connected_to_val,
        "from_link_id": [int(meta['from_link_id'])],
        "to_link_id": [int(meta['to_link_id'])],
        # "start" and "end" are taken here from the left boundary.
        "start": center_coords[0],
        "end": center_coords[-1],
        "left_boundary": {
            "type": meta['left_boundary_type'],
            "boundary": left_boundary_coords
        },
        "right_boundary": {
            "type": meta['right_boundary_type'],
            "boundary": right_boundary_coords
        },
        "center": center_coords  # Handles varying numbers of coordinate pairs for the centerline.
    }
    
    result["lanes"].append(lane_data)

# Define a custom converter to handle NumPy types during JSON serialization.
def default_converter(o):
    if isinstance(o, (np.int64, np.int32)):
        return int(o)
    if isinstance(o, (np.float64, np.float32)):
        return float(o)
    raise TypeError(f"Object of type {o.__class__.__name__} is not JSON serializable")

# Write the result to a JSON file.
with open('result_e2j.json', 'w') as f:
    json.dump(result, f, indent=4, default=default_converter)

print("The data has been successfully saved to result_e2j.json")