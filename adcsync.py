import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D projection

def decode_12bit_twos_complement(value):
    """
    Interprets a 16-bit integer as if only the lower 12 bits are valid
    and it is stored in two's complement for a 12-bit ADC.
    
    For a 12-bit range:
      - Values 0..2047   remain  0..2047
      - Values 2048..4095 become -2048..-1
    """
    value = value & 0xFFF  # mask to 12 bits
    # If you needed actual signed interpretation:
    # if value & 0x800:
    #     value -= 0x1000
    return value

def parse_adc_blocks(filename):
    """
    Reads the given log file line by line, extracts any JSON blocks
    that start with '{' and end with '}' under the 'I (xxx) ADC:' prefix,
    and returns them as a list of dicts.
    """
    blocks = []
    in_block = False
    block_lines = []

    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            # We only care about lines that contain " ADC:"
            if " ADC:" not in line:
                continue

            # Split at " ADC: " once
            parts = line.split(" ADC: ", 1)
            if len(parts) < 2:
                continue  # not a well-formed line; skip

            # The actual JSON-ish content is after "ADC: "
            content = parts[1].strip()

            # Check for start/end of block
            if content == '{':
                in_block = True
                block_lines = ['{']
            elif content == '}':
                in_block = False
                block_lines.append('}')
                # Attempt to parse the accumulated lines as JSON
                block_str = "\n".join(block_lines)
                try:
                    parsed = json.loads(block_str)
                    blocks.append(parsed)
                except json.JSONDecodeError as e:
                    print("JSON parse error:", e)
            else:
                # If we are in the middle of an ADC block, accumulate lines
                if in_block:
                    block_lines.append(content)

    return blocks

def plot_adc_data(ax, blocks, file_label):
    """
    Plots ADC data on the given axis.
    
    Parameters:
    ax (matplotlib.axes.Axes): The axis to plot on.
    blocks (list): The list of ADC data blocks.
    file_label (str): The label for the file being plotted.
    """
    ax.set_title(f"ADC dumps from {file_label}")
    ax.set_xlabel("Sample index")
    ax.set_ylabel("ADC value (decoded)")
    ax.grid(True)

    for idx, block in enumerate(blocks):
        counter = block.get("counter", 0)
        adc_data = block.get("adc_data", [])
        # decode each ADC sample
        decoded = [decode_12bit_twos_complement(x) for x in adc_data]
        x = range(len(decoded))
        ax.plot(x, decoded, label=f"Dump {counter}", alpha=0.5)
        # plot ts
        ts_data = block.get("adc_ts", [])
        x_ = np.arange(len(decoded)//len(ts_data), len(decoded), len(decoded)//len(ts_data))
        y_ = np.array(sorted(ts_data))
        for i in range(len(x_)):
            ax.axvline(x=x_[i], color='r', linestyle='--')
            y = 400 if i % 2 == 0 else 2500
            offset = idx * 40
            y += offset if i % 2 == 0 else -offset
            ax.text(x_[i], y, f"ts {counter}: {y_[i]}", fontsize=8)

    ax.legend()

def main():
    # Parse out all ADC JSON blocks from both raw logs
    file1 = "adc6uwb_1.txt"
    file2 = "adc6uwb_0.txt"
    blocks1 = parse_adc_blocks(file1)
    blocks2 = parse_adc_blocks(file2)

    # Prepare a big figure containing 3 subplots:
    #   - ax1 and ax2 for time-domain plots from each file
    #   - ax3 for 3D correlation plots
    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(2, 2, 1)  # top-left
    ax2 = fig.add_subplot(2, 2, 2)  # top-right
    ax3 = fig.add_subplot(2, 1, 2)


    # Plot ADC data for both files
    plot_adc_data(ax1, blocks1, file1)
    plot_adc_data(ax2, blocks2, file2)

    # -------------------------------------------------
    # CORRELATION PLOT (2D): Matching 'counter' fields
    # -------------------------------------------------
    ax3.set_title("Cross-correlation (via convolution) for matching counters")
    ax3.set_xlabel("Lag index")
    ax3.set_ylabel("Correlation amplitude")
    ax3.grid(True)

    # Build dictionaries keyed by 'counter' for quick matching
    dict1 = { block["counter"]: block for block in blocks1 if "counter" in block }
    dict2 = { block["counter"]: block for block in blocks2 if "counter" in block }

    # Find all matching counters
    matching_counters = sorted(set(dict1.keys()) & set(dict2.keys()))
    if not matching_counters:
        print("No matching counters found between the two log files.")

    counter2ts = dict()
    # For each match, compute correlation by convolution and plot
    for i,c in enumerate(matching_counters):
        b1 = dict1[c]
        b2 = dict2[c]

        decoded1 = [decode_12bit_twos_complement(x) for x in b1.get("adc_data", [])]
        decoded2 = [decode_12bit_twos_complement(x) for x in b2.get("adc_data", [])]

        # Convolution as a simple cross-correlation measure
        corr = np.convolve(decoded1, decoded2, mode='full')
        peak = np.max(corr)
        peakx = np.argmax(corr)

        # x-axis: sample/lag indices
        x_vals = np.arange(len(corr))
        samplerate = 80000
        offset = len(decoded1) - 1
        x_vals = (x_vals - offset) / samplerate
        # Convert to seconds
        x_vals = x_vals.astype(np.float64)
        # Plot the correlation
        # ax3.plot(x_vals, corr, label=f"Counter={c}")
        peak_time = (peakx - offset) / samplerate
        n_samples = (peakx - offset)
        counter2ts[c] = (peak_time, n_samples)
        # what time difference does the peak represent?
        # Plot each correlation in 2D, superimposing them with some alpha
        ax3.plot(x_vals, corr, label=f"{c}: {(1e6)*peak_time:.3e}us or ticks: {n_samples}", alpha=0.6)
        # Annotate the peak
        # ax3.annotate(f"Peak at {peak_time:.4f}s", xy=(peakx, peak), xytext=(peakx+0.01, np.mean(corr)-1000*i),
        #              arrowprops=dict(facecolor='black', shrink=0.05))
        # ax3.text(peakx, peak, f"Peak at {peak_time:.4f}s", fontsize=8)

    if matching_counters:
        ax3.legend()

    plt.tight_layout()
    fig.tight_layout()

    # -------------------------------------------------
    # Print the time differences for matching counters
    print("Time differences for matching counters:")
    for c in matching_counters:
        ts1 = sorted(dict1[c].get("adc_ts", []))
        ts2 = sorted(dict2[c].get("adc_ts", []))
        if ts1 and ts2:
            ts_diff = np.array(ts2) - np.array(ts1)
            print(f"Counter {c}: diff: {counter2ts[c][0]*1e6} us |Time differences (file2 - file1) = {np.mean(ts_diff):.3f} us, stddev = {np.std(ts_diff):.3f} us, adjusted = {np.mean(ts_diff)-counter2ts[c][0]*1e6} us")

    # Save or show
    fig.savefig("adc_data_plots.png", dpi=300)
    # fig_corr.savefig("adc_correlations.png", dpi=300)
    plt.show()


if __name__ == "__main__":
    main()
