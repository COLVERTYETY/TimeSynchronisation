import pandas as pd
import numpy as np
import itertools
import matplotlib.pyplot as plt
from tqdm import tqdm

# Load CSV Data
def load_data(csv_path):
    df = pd.read_csv(csv_path)
    df = df[df["/SN"] > 0]  # Filter out invalid exchanges
    return df

# Reconstruct Exchanges
def reconstruct_exchanges(df):
    exchanges = {}
    
    for sn, group in df.groupby("/SN"):
        if len(group["/AN"].unique()) != 2:
            continue  # Ensure exchanges involve exactly two devices
        
        poll_device = group[group["/AN"] == group["/AN"].min()]
        resp_device = group[group["/AN"] == group["/AN"].max()]
        
        if poll_device.empty or resp_device.empty:
            continue
        
        exchange = {
            "poll_tx": poll_device["/uwb_poll_tx"].values[0],
            "resp_rx": resp_device["/uwb_resp_rx"].values[0],
            "resp_tx": resp_device["/uwb_resp_tx"].values[0],
            "poll_rx": poll_device["/uwb_poll_rx"].values[0],
            "poll_mcu_tx": poll_device["/mcu_poll_tx"].values[0],
            "poll_mcu_rx": poll_device["/mcu_poll_rx"].values[0],
            "resp_mcu_rx": resp_device["/mcu_resp_rx"].values[0],
            "resp_mcu_tx": resp_device["/mcu_resp_tx"].values[0],
        }
        
        exchanges[sn] = exchange
    
    return exchanges

# Estimate Time Offset
def estimate_offset(exchanges):
    offsets = []
    for sn, ex in exchanges.items():
        if all(ex[key] > 0 for key in ex):
            uwb_offset = (ex["poll_rx"] - ex["poll_tx"]) - (ex["resp_tx"] - ex["resp_rx"])
            mcu_offset = (ex["poll_mcu_rx"] - ex["poll_mcu_tx"]) - (ex["resp_mcu_tx"] - ex["resp_mcu_rx"])
            offsets.append((sn, uwb_offset, mcu_offset))
    return offsets

# Main Function
def main(csv_path):
    df = load_data(csv_path)
    exchanges = reconstruct_exchanges(df)
    offsets = estimate_offset(exchanges)
    
    # Convert to DataFrame for visualization
    offset_df = pd.DataFrame(offsets, columns=["/SN", "UWB_Offset", "MCU_Offset"])
    
    # Plot offsets
    plt.figure(figsize=(10, 5))
    plt.plot(offset_df["/SN"], offset_df["UWB_Offset"], label="UWB Offset", marker="o")
    plt.plot(offset_df["/SN"], offset_df["MCU_Offset"], label="MCU Offset", marker="s")
    plt.xlabel("Exchange SN")
    plt.ylabel("Offset (time units)")
    plt.title("Time Offset Comparison between UWB and MCU Domains")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    csv_file_path = "timesync.csv"  # Replace with actual path
    main(csv_file_path)
