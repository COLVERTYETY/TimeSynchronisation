import json
import re
import numpy as np
import matplotlib.pyplot as plt

# def load_data_points(file_path):
#     """
#     Reads a log file, strips the log prefix (e.g., "I (201841) SYNC: "),
#     assembles complete JSON blocks (handling nested braces), and returns a
#     dictionary of data points keyed by the 'counter' field.
#     """
#     data_points = {}
#     with open(file_path, 'r') as f:
#         lines = f.readlines()

#     current_json_lines = []
#     inside_json = False
#     open_braces = 0

#     for line in lines:
#         # Remove the log prefix
#         cleaned_line = re.sub(r'^I \(\d+\) SYNC: ', '', line).strip()
        
#         # Skip lines like "GT Task"
#         if cleaned_line.startswith("GT Task"):
#             continue

#         # Detect start of a JSON block
#         if cleaned_line.startswith("{") and not inside_json:
#             inside_json = True
#             current_json_lines = [cleaned_line]
#             open_braces = cleaned_line.count("{") - cleaned_line.count("}")
#             continue

#         if inside_json:
#             current_json_lines.append(cleaned_line)
#             open_braces += cleaned_line.count("{") - cleaned_line.count("}")
            
#             # When the braces balance, we have a complete JSON block.
#             if open_braces == 0:
#                 json_text = " ".join(current_json_lines)
#                 try:
#                     data = json.loads(json_text)
#                     counter = data.get("counter")
#                     if counter is not None:
#                         data_points[counter] = data
#                         # Convert mcu_ts to correct units
#                         # data["mcu_ts"] = data["mcu_ts"] / 80 
#                 except Exception as e:
#                     print(f"Error parsing JSON block: {e}\nText: {json_text}")
#                 inside_json = False
#                 current_json_lines = []
#     return data_points
def load_data_points(file_path):
    """
    Reads a log file, strips the log prefix (e.g., "I (201841) SYNC: "),
    assembles complete JSON blocks (handling nested braces), and returns a
    dictionary of data points keyed by the 'counter' field.
    """
    data_points = {}
    with open(file_path, 'r') as f:
        lines = f.readlines()

    current_json_lines = []
    inside_json = False
    open_braces = 0

    for line in lines:
        # Remove the log prefix
        cleaned_line = re.sub(r'^I \(\d+\) SYNC: ', '', line).strip()

        # Skip lines like "GT Task"
        if cleaned_line.startswith("GT Task"):
            continue

        # Detect start of a JSON block
        if cleaned_line.startswith("{") and not inside_json:
            inside_json = True
            current_json_lines = [cleaned_line]
            open_braces = cleaned_line.count("{") - cleaned_line.count("}")
            continue

        if inside_json:
            current_json_lines.append(cleaned_line)
            open_braces += cleaned_line.count("{") - cleaned_line.count("}")

            # When the braces balance, we have a complete JSON block.
            if open_braces == 0:
                json_text = " ".join(current_json_lines)

                # ---------- NEW FIX: Remove trailing commas before '}' or ']' ----------
                # This regex finds a comma optionally followed by whitespace,
                # then a closing brace/bracket. It removes the comma.
                json_text = re.sub(r',\s*([}\]])', r'\1', json_text)
                # -----------------------------------------------------------------------

                try:
                    data = json.loads(json_text)
                    counter = data.get("counter")
                    if counter is not None:
                        data_points[counter] = data
                        # print(data)
                        # If you need to adjust the MCU TS scale:
                        # data_points[counter]["mcu_ts"] = data["mcu_ts"] / 80.0
                except Exception as e:
                    print(f"Error parsing JSON block: {e}\nText: {json_text}")

                inside_json = False
                current_json_lines = []
    return data_points

def predict_remote_ts(data_point):
    """
    Given a data point, use its mixed-effects estimator (from the "MixedResults")
    and its local mcu_ts to predict the remote device's local timestamp.
    
    The prediction is:
       predicted_remote_ts = beta0 + beta1 * local_ts + avg_u
    where avg_u is the average of the two group offsets (u_hat_0 and u_hat_1).
    """
    estimator = data_point.get("MixedResults")
    if not estimator:
        raise ValueError("Missing MixedResults in data point")
    beta0 = estimator.get("beta0", 0.0)
    beta1 = estimator.get("beta1", 0.0)
    u_hat0 = estimator.get("u_hat_0", 0.0)
    u_hat1 = estimator.get("u_hat_1", 0.0)
    avg_u = (u_hat0 + u_hat1) / 2.0
    local_ts = data_point.get("mcu_ts", 0.0)
    slope = data_point.get("slope", 1.0)
    intercept = data_point.get("intercept", 0.0)
    res1 = beta0 + beta1 * local_ts
    res2 = slope * local_ts + intercept
    # print(f"beta0: {beta0}, beta1: {beta1}, u_hat0: {u_hat0}, u_hat1: {u_hat1}, avg_u: {avg_u}, local_ts: {local_ts}, slope: {slope}, intercept: {intercept}")
    # return beta0 + beta1 * local_ts #+ np.sign(intercept) * 155123770.25675675
    # return slope * local_ts + intercept
    return ( res1, res2)

def compute_sync_stats(data_points1, data_points2):
    """
    For each common data point (matched by 'counter') between the two files,
    compute the error in the estimated remote timestamp from each perspective.
    
    From file1's perspective:
      - Use file1's estimator with its mcu_ts to predict file2's mcu_ts.
    
    From file2's perspective:
      - Use file2's estimator with its mcu_ts to predict file1's mcu_ts.
    
    Returns a dictionary with the following structure:
    
      {
         "file1_to_file2": {
              "mean_error": ...,
              "mean_abs_error": ...,
              "std_error": ...,
              "n_points": ...
         },
         "file2_to_file1": { ... same fields ... }
      }
    """
    errors_1 = []  # errors when using file1's estimator to predict file2's ts
    errors_2 = []  # errors when using file2's estimator to predict file1's ts
    common_counters = sorted(set(data_points1.keys()).intersection(set(data_points2.keys())))
    avg_ts_offset = 0
    avg_ts_diff_1 = 0
    avg_ts_diff_2 = 0
    prev_ts1 = -1
    prev_ts2 = -1
    # common_counters = common_counters[:10]
    for counter in common_counters:
        dp1 = data_points1[counter]
        dp2 = data_points2[counter]
        ts1 = dp1.get("mcu_ts")
        ts2 = dp2.get("mcu_ts")
        if prev_ts1 != -1 and prev_ts2 != -1:
            avg_ts_offset += ts1 - ts2
            dts1 = ts1 - prev_ts1
            avg_ts_diff_1 += dts1
            dts2 = ts2 - prev_ts2
            avg_ts_diff_2 += dts2
            prev_ts1 = ts1
            prev_ts2 = ts2
            # print(f"Counter: {counter}, ts1: {ts1}, ts2: {ts2}, dts1: {dts1}, dts2: {dts2}")
        else:
            prev_ts1 = ts1
            prev_ts2 = ts2
        # Prediction from file1's estimator:
        try:
            pred2_from_1_1, pred2_from_1_2 = predict_remote_ts(dp1)
            actual_ts2 = dp2.get("mcu_ts")
            if actual_ts2 is not None:
                error1_1 = pred2_from_1_1 - actual_ts2
                error1_2 = pred2_from_1_2 - actual_ts2
                dp1["error1"] = error1_1
                dp1["error2"] = error1_2
                print(f"side: dp1 Counter: {counter}, error1: {error1_1}, error2: {error1_2}, pred1: {pred2_from_1_1}, pred2: {pred2_from_1_1}, actual: {actual_ts2}")
                errors_1.append(error1_1)
        except Exception as e:
            print(f"Error predicting from file1 for counter {counter}: {e}")
        
        # Prediction from file2's estimator:
        try:
            pred1_from_2_1, pred1_from_2_2 = predict_remote_ts(dp2)
            actual_ts1 = dp1.get("mcu_ts")
            if actual_ts1 is not None:
                error2_1 = pred1_from_2_1 - actual_ts1
                error2_2 = pred1_from_2_2 - actual_ts1
                dp2["error1"] = error2_1
                dp2["error2"] = error2_2
                print(f"side: dp2 Counter: {counter}, error1: {error2_1}, error2: {error2_2}, pred1: {pred1_from_2_1}, pred2: {pred1_from_2_2}, actual: {actual_ts1}")
                errors_2.append(error2_1)
        except Exception as e:
            print(f"Error predicting from file2 for counter {counter}: {e}")
    
    errors_1 = np.array(errors_1)
    errors_2 = np.array(errors_2)
    
    stats = {
        "file1_to_file2": {
            "mean_error": float(np.mean(errors_1)) if errors_1.size > 0 else None,
            "mean_abs_error": float(np.mean(np.abs(errors_1))) if errors_1.size > 0 else None,
            "std_error": float(np.std(errors_1)) if errors_1.size > 0 else None,
            "n_points": int(errors_1.size)
        },
        "file2_to_file1": {
            "mean_error": float(np.mean(errors_2)) if errors_2.size > 0 else None,
            "mean_abs_error": float(np.mean(np.abs(errors_2))) if errors_2.size > 0 else None,
            "std_error": float(np.std(errors_2)) if errors_2.size > 0 else None,
            "n_points": int(errors_2.size)
        }
    }
    avg_ts_offset /= len(common_counters)
    avg_ts_diff_1 /= len(common_counters)
    avg_ts_diff_2 /= len(common_counters)
    expected_diff = 50000
    print(f"Average timestamp difference 1: {avg_ts_diff_1} err:{abs(avg_ts_diff_1 - expected_diff)}")
    print(f"Average timestamp difference 2: {avg_ts_diff_2} err:{abs(avg_ts_diff_2 - expected_diff)}")
    print(f"Average timestamp offset: {avg_ts_offset}")
    return stats

def plot_sync_arrows(data_points1, data_points2):
    """
    Generates a matplotlib graph with two horizontal levels representing File 1 and File 2.
    
    For each common data point (matched by counter):
      - From File 1: Draw an arrow from its actual mcu_ts (y=1) to the predicted File 2 mcu_ts (y=2)
      - From File 2: Draw an arrow from its actual mcu_ts (y=2) to the predicted File 1 mcu_ts (y=1)
    """
    # Find common counters
    common_counters = sorted(set(data_points1.keys()).intersection(data_points2.keys()))
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(20, 20))
    
    # Define y-levels for the two files
    y_file1 = 1
    y_file2 = 2
    
    errors_1 = []
    errors_1_2 = []
    errors_2_2 = []
    errors_2 = []
    counters = []
    temp1 = []
    temp2 = []
    # Loop through each common data point
    for counter in common_counters:
        dp1 = data_points1[counter]
        dp2 = data_points2[counter]
        
        ts1 = dp1.get("mcu_ts")
        ts2 = dp2.get("mcu_ts")
        
        if ts1 is None or ts2 is None:
            continue
        
        try:
            # Use File 1's estimator to predict File 2's mcu_ts
            pred2_from_1,_ = predict_remote_ts(dp1)
            # Use File 2's estimator to predict File 1's mcu_ts
            pred1_from_2,_ = predict_remote_ts(dp2)
        except Exception as e:
            print(f"Error predicting for counter {counter}: {e}")
            continue

        # Plot the actual timestamps as points
        ax1.plot(ts1, y_file1, 'o', color='blue', label='File 1 Actual' if counter == common_counters[0] else "")
        ax1.plot(ts2, y_file2, 'o', color='red', label='File 2 Actual' if counter == common_counters[0] else "")
        ax1.text(ts1, y_file1-0.02, f"{counter}", fontsize=3, ha='right', va='bottom')
        ax1.text(ts2, y_file2+0.02, f"{counter}", fontsize=3, ha='right', va='bottom')
        
        # Draw arrow from File 1 (actual) to File 2 (predicted) 
        ax1.annotate('', xy=(pred2_from_1, y_file2), xytext=(ts1, y_file1),
                    arrowprops=dict(arrowstyle='->', color='green', lw=1))
        
        # Draw arrow from File 2 (actual) to File 1 (predicted)
        ax1.annotate('', xy=(pred1_from_2, y_file1), xytext=(ts2, y_file2),
                    arrowprops=dict(arrowstyle='->', color='purple', lw=1))
        
        # Collect errors for plotting
        errors_1.append(dp1.get("error1", 0))
        errors_2.append(dp2.get("error1", 0))
        errors_1_2.append(dp1.get("error2", 0))
        errors_2_2.append(dp2.get("error2", 0))
        counters.append(counter)
        temp1.append(dp1.get("temperature", 0))
        temp2.append(dp2.get("temperature", 0))
    
    ax1.set_xlabel("MCU Timestamp")
    ax1.set_yticks([y_file1, y_file2])
    ax1.set_yticklabels(["File 1", "File 2"])
    ax1.set_title("Time Synchronisation: Actual vs. Predicted MCU Timestamps")
    ax1.legend()
    ax1.grid(True)

    # Plot errors
    ax2.scatter(counters, errors_1, color='green', label='File 1 to File 2 Error type1')
    ax2.scatter(counters, errors_2, color='purple', label='File 2 to File 1 Error type1')
    ax2.scatter(counters, errors_1_2, color='red', label='File 1 to File 2 Error type2')
    ax2.scatter(counters, errors_2_2, color='orange', label='File 2 to File 1 Error type2')

    # # Create a secondary axis for the histogram
    # ax_histy = ax2.twinx()
    # ax_histy.hist(errors_1, bins=20, alpha=0.3, color='green', orientation='horizontal')
    # ax_histy.hist(errors_2, bins=20, alpha=0.3, color='purple', orientation='horizontal')
    # ax_histy.hist(errors_1_2, bins=20, alpha=0.3, color='red', orientation='horizontal')
    # ax_histy.hist(errors_2_2, bins=20, alpha=0.3, color='orange', orientation='horizontal')

    ax2.set_xlabel("Counter")
    ax2.set_ylabel("Error (MCU Timestamp)")
    ax2.set_title("Prediction Errors")
    ax2.legend()
    ax2.grid(True)

    # Plot temperature data
    ax3.plot(counters, temp1, 'o-', color='blue', label='File 1 Temperature')
    ax3.plot(counters, temp2, 'o-', color='red', label='File 2 Temperature')
    ax3.set_xlabel("Counter")
    ax3.set_ylabel("Temperature (C)")
    ax3.set_title("Temperature Data")
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig('sync.png', dpi=300)

# --------------------------
# Example usage:
# --------------------------

# Replace with the actual paths to your log files from device A and device B.
file1 = 'shortlonguwb_1.txt'
file2 = 'shortlonguwb_0.txt'

# Load data points from both files.
data_points1 = load_data_points(file1)
data_points2 = load_data_points(file2)

# Verify that both files have the same set of data points (by 'counter').
if set(data_points1.keys()) != set(data_points2.keys()):
    print("Mismatch in data points between files!")
else:
    # Compute the synchronization error statistics from both perspectives.
    sync_stats = compute_sync_stats(data_points1, data_points2)
    
    # Display the results.
    print("Synchronization Quality Analysis:")
    print("\nUsing file1's estimator to predict file2's mcu_ts:")
    print(f"  Number of data points: {sync_stats['file1_to_file2']['n_points']}")
    print(f"  Mean error: {sync_stats['file1_to_file2']['mean_error']}")
    print(f"  Mean absolute error: {sync_stats['file1_to_file2']['mean_abs_error']}")
    print(f"  Standard deviation of error: {sync_stats['file1_to_file2']['std_error']}")
    
    print("\nUsing file2's estimator to predict file1's mcu_ts:")
    print(f"  Number of data points: {sync_stats['file2_to_file1']['n_points']}")
    print(f"  Mean error: {sync_stats['file2_to_file1']['mean_error']}")
    print(f"  Mean absolute error: {sync_stats['file2_to_file1']['mean_abs_error']}")
    print(f"  Standard deviation of error: {sync_stats['file2_to_file1']['std_error']}")
    plot_sync_arrows(data_points1, data_points2)