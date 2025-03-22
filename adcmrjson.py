import json
import numpy as np
import matplotlib.pyplot as plt

def decode_12bit_twos_complement(value):
    """
    Interprets a 16-bit integer as if only the lower 12 bits are valid
    and it is stored in two's complement for a 12-bit ADC.
    """
    value = value & 0xFFF  # mask to 12 bits
    # If you needed actual signed interpretation:
    # if value & 0x800:
    #     value -= 0x1000
    return value


def parse_log_blocks(filename):
    """
    General parser that reads the given log file line by line, extracting
    both 'MRJSON' and 'ADC' blocks that start with '{' and end with '}'.

    Returns two lists: (mrjson_blocks, adc_blocks).
    Each block is a dictionary with:
       {
         "log_time": <int from "I (xxx) ...">,
         "type": "MRJSON" or "ADC",
         ... plus the parsed JSON fields ...
       }
    """

    mrjson_blocks = []
    adc_blocks = []

    in_block = False
    block_lines = []
    block_type = None
    block_start_time = None  # the integer from "I (xxx) Something:"

    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            # Grab the log_time from lines that look like:  I (121201) MRJSON: {
            # or I (111511) ADC: {
            # We'll attempt to parse that out each time
            if line.startswith("I ("):
                # e.g. line = 'I (121201) MRJSON: {'
                # strip "I (" -> remainder: '121201) MRJSON: {'
                # first token up to ')' is the integer
                try:
                    after_open_paren = line[3:]  # skip "I ("
                    tokens = after_open_paren.split(")", 1)
                    log_time_str = tokens[0].strip()
                    block_start_time = int(log_time_str)
                except Exception:
                    block_start_time = None

            # Decide if we start capturing an MRJSON or ADC block
            if " MRJSON:" in line or " ADC:" in line:
                # We only care about lines that contain " MRJSON:" or " ADC:"
                # We'll separate the prefix from the content
                if " MRJSON:" in line:
                    block_type = "MRJSON"
                    prefix_str = " MRJSON: "
                else:
                    block_type = "ADC"
                    prefix_str = " ADC: "

                parts = line.split(prefix_str, 1)
                if len(parts) < 2:
                    continue  # not well-formed

                content = parts[1].strip()
                # Check for start
                if content == '{':
                    in_block = True
                    block_lines = ['{']
                elif content == '}':
                    # if it happens to appear on the same line
                    # that would be an empty block, but let's handle it anyway
                    in_block = False
                    block_lines.append('}')
                    # attempt parse
                    block_str = "\n".join(block_lines)
                    try:
                        parsed = json.loads(block_str)
                        block_dict = {"log_time": block_start_time, "type": block_type}
                        block_dict.update(parsed)  # add all JSON fields
                        if block_type == "MRJSON":
                            mrjson_blocks.append(block_dict)
                        else:
                            adc_blocks.append(block_dict)
                    except json.JSONDecodeError as e:
                        print("JSON parse error:", e)
                else:
                    # if we are in the middle of an MRJSON or ADC block
                    if in_block:
                        block_lines.append(content)

            else:
                # If we are in_block, check if line closes it, or is more content
                if in_block:
                    # Some lines might literally be 'I (nnn) MRJSON: }'
                    # or they might only contain partial JSON. We have to check carefully.
                    trimmed = line.strip()
                    if trimmed.endswith('}'):
                        # We might have partial plus a closing brace
                        # e.g.   I (121232) MRJSON: }
                        # or something like: 'I (121232) MRJSON: "offset": 3 }'
                        # So let's see if there's any text before the brace
                        before, sep, after = trimmed.rpartition('}')
                        before = before.strip()
                        if before:
                            block_lines.append(before)
                        block_lines.append('}')
                        in_block = False
                        # Attempt parse
                        block_str = "\n".join(block_lines)
                        try:
                            parsed = json.loads(block_str)
                            block_dict = {"log_time": block_start_time, "type": block_type}
                            block_dict.update(parsed)
                            if block_type == "MRJSON":
                                mrjson_blocks.append(block_dict)
                            else:
                                adc_blocks.append(block_dict)
                        except json.JSONDecodeError as e:
                            print("JSON parse error:", e)
                    else:
                        # just accumulate the line
                        block_lines.append(trimmed)

    return mrjson_blocks, adc_blocks


def plot_adc_data(ax, blocks, file_label):
    """
    Same as before, but now 'blocks' are presumably just the ADC blocks 
    from parse_log_blocks().
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
        if ts_data:
            # naive approach: mark vertical lines spaced out along x
            step = max(1, len(decoded)//max(1,len(ts_data)))
            x_ = np.arange(step, len(decoded), step)
            y_ = np.array(sorted(ts_data))
            for i in range(len(x_)):
                ax.axvline(x=x_[i], color='r', linestyle='--')
                # lightly annotate
                y = 400 if i % 2 == 0 else 2500
                offset = idx * 40
                y += offset if i % 2 == 0 else -offset
                ax.text(x_[i], y, f"ts {counter}: {y_[i]}", fontsize=8)

    ax.legend()


def main():
    file1 = "adc6uwb_1.txt"
    file2 = "adc6uwb_0.txt"

    # Now parse *both* MRJSON and ADC blocks
    mrjson1, adc1 = parse_log_blocks(file1)
    mrjson2, adc2 = parse_log_blocks(file2)

    # Prepare a big figure: 2 subplots for the ADC waveforms,
    # and 1 subplot for correlation
    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(2, 2, 1)  # top-left
    ax2 = fig.add_subplot(2, 2, 2)  # top-right
    ax3 = fig.add_subplot(2, 1, 2)

    # Plot ADC data
    plot_adc_data(ax1, adc1, file1)
    plot_adc_data(ax2, adc2, file2)

    # Matching counters (for correlation)
    dict1 = {b["counter"]: b for b in adc1 if "counter" in b}
    dict2 = {b["counter"]: b for b in adc2 if "counter" in b}
    matching_counters = sorted(set(dict1.keys()) & set(dict2.keys()))

    ax3.set_title("Cross-correlation (via convolution) for matching counters")
    ax3.set_xlabel("Lag (seconds, approx)")
    ax3.set_ylabel("Correlation amplitude")
    ax3.grid(True)

    samplerate = 80000  # just an example
    counter2ts = {}
    for c in matching_counters:
        b1 = dict1[c]
        b2 = dict2[c]

        decoded1 = [decode_12bit_twos_complement(x) for x in b1.get("adc_data", [])]
        decoded2 = [decode_12bit_twos_complement(x) for x in b2.get("adc_data", [])]

        corr = np.convolve(decoded1, decoded2, mode='full')
        peak = np.max(corr)
        peakx = np.argmax(corr)

        offset = len(decoded1) - 1
        x_vals = np.arange(len(corr)) - offset
        x_vals_s = x_vals / samplerate  # convert to seconds
        peak_time = (peakx - offset) / samplerate
        n_samples = (peakx - offset)
        counter2ts[c] = (peak_time, n_samples)

        ax3.plot(x_vals_s, corr, label=f"ctr={c} => {peak_time*1e6:.1f}us", alpha=0.6)

    if matching_counters:
        ax3.legend()

    plt.tight_layout()

    # ---------------------------------------------------------------------
    # Now illustrate how you'd match each ADC block to a "nearby" MRJSON:
    # ---------------------------------------------------------------------
    # We can build an index of MRJSON blocks keyed by counter if they have a "counter".
    mrjson_dict1 = {m["counter"]: m for m in mrjson1 if "counter" in m}
    mrjson_dict2 = {m["counter"]: m for m in mrjson2 if "counter" in m}

    print("----- Combined Analysis of ADC correlation & MRJSON offsets -----")
    for c in matching_counters:
        # Suppose we have an MRJSON block with the same 'counter'
        if c in mrjson_dict1 and c in mrjson_dict2:
            # For example:
            #   slope, intercept from each side
            slope1 = mrjson_dict1[c].get("slope", None)
            intercept1 = mrjson_dict1[c].get("intercept", None)
            stderr1 = mrjson_dict1[c].get("residual_variance", None)
            slope2 = mrjson_dict2[c].get("slope", None)
            intercept2 = mrjson_dict2[c].get("intercept", None)
            stderr2 = mrjson_dict2[c].get("residual_variance", None)

            # And the cross-corr offset from the ADC analysis
            peak_time_us = counter2ts[c][0] * 1e6

            print(f"Counter={c} => CrossCorr offset: {peak_time_us:.2f} us")
            print(f"   MRJSON1 slope={slope1}, intercept={intercept1} (stderr={stderr1})")
            print(f"   MRJSON2 slope={slope2}, intercept={intercept2} (stderr={stderr2})")
        else:
            print(f"Counter={c} has no matching MRJSON data in one or both logs.")

    # Save or show
    plt.savefig("adc_data_plots.png", dpi=300)
    plt.show()


if __name__ == "__main__":
    main()
