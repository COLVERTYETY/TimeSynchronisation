#!/usr/bin/env python3
import json
import re
import sys
from dataclasses import dataclass, asdict
from typing import List
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class TimeUS:
    mcu: float  # the tx event timestamp
    uwb: float  # the rx event timestamp

@dataclass
class EventPair:
    tx: TimeUS
    adjusted_tx: TimeUS
    rx: TimeUS
    adjusted_rx: TimeUS
    tof_dtu: float
    is_init: bool
    is_SS: bool

@dataclass
class EventDump:
    index: int
    slope: float
    intercept: float
    std_err: float
    events: List[EventPair]


def graph_event_table(last_dump, index):
    """
    Produces 4 subplots arranged in a 2x2 grid:
      - Top-left: TX & RX (MCU timestamps, solid lines)
      - Top-right: TX & RX (UWB timestamps, dashed lines)
      - Bottom-left: Adjusted TX & RX (MCU timestamps, solid lines)
      - Bottom-right: Adjusted TX & RX (UWB timestamps, dashed lines)
    
    The figure suptitle records the event count, slope, intercept, and std_err.
    Colors are consistent:
      - TX: Blue
      - RX: Red
      - Adjusted TX: Green
      - Adjusted RX: Magenta
    """
    events = last_dump.events
    x = list(range(len(events)))
    
    # TX and RX timestamps for MCU and UWB
    tx_mcu = [e.tx.mcu for e in events]
    rx_mcu = [e.rx.mcu for e in events]
    tx_uwb = [e.tx.uwb for e in events]
    rx_uwb = [e.rx.uwb for e in events]
    
    # Adjusted TX and RX timestamps for MCU and UWB
    adj_tx_mcu = [e.adjusted_tx.mcu for e in events]
    adj_rx_mcu = [e.adjusted_rx.mcu for e in events]
    adj_tx_uwb = [e.adjusted_tx.uwb for e in events]
    adj_rx_uwb = [e.adjusted_rx.uwb for e in events]
    x_mcu = []
    y_mcu = []
    for i in range(len(events)):
        if events[i].is_init:
            x_mcu.append(events[i].tx.mcu)
            y_mcu.append(events[i].adjusted_rx.mcu)
        else:
            y_mcu.append(events[i].tx.mcu)
            x_mcu.append(events[i].adjusted_rx.mcu)
    tof = [e.tof_dtu for e in events]
    is_init = [e.is_init for e in events]
    dist = [0.5 * 299792458 * t * 1e-6 for t in tof]  # Speed of light in m/us
    # for each abs(tof) > 1 print teh data 
    invalide = []
    for i, (t,e) in enumerate(zip(tof, events)):
        if abs(t) > 1:
            invalide.append(i)
            print(f"{i} tof: {t:.3f} is_init: {e.is_init} is_SS: {e.is_SS}")
    max_y = max(max(tx_mcu), max(rx_mcu), max(tx_uwb), max(rx_uwb))
    min_y = min(min(tx_mcu), min(rx_mcu), min(tx_uwb), min(rx_uwb))
    
    # Create a 2x2 grid of subplots
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))#, sharex=True)
    
    # Top-left: TX & RX (MCU)
    axs[0, 0].plot(x, tx_mcu, 'b-', label="TX MCU")
    axs[0, 0].plot(x, rx_mcu, 'r-', label="RX MCU")
    for inv in invalide:
        axs[0, 0].axvline(x=inv, color='k', linestyle='--', label="Invalid TOF")
    axs[0, 0].set_title("TX & RX (MCU)")
    axs[0, 0].set_ylabel("Timestamp")
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    
    # axs[0, 0].set_ylim(min_y, max_y)
    
    # avg offset 
    offset = np.mean([abs(rx - tx) for tx, rx in zip(tx_mcu, rx_mcu)])
    print(f"avg offset: {offset:.3f}")
    offset_diff = abs(offset - abs(last_dump.intercept))
    slope_lsb = 1-abs(last_dump.slope)
    slope_perc = slope_lsb * offset

    # Top-right: TX & RX (UWB)
    avg_tof = np.mean(tof)
    tof_spread = np.max(tof) - np.min(tof)
    var = 100*(tof_spread / avg_tof)
    none_null_dist = [d for d, i in zip(dist, is_init) if d != 0]
    avg_dist = np.mean(none_null_dist) if none_null_dist else 0
    dist_spread = np.max(none_null_dist) - np.min(none_null_dist)
    dist_var = 100*(dist_spread / avg_dist) if avg_dist != 0 else 0
    # Compute the regression line over the range of adjusted_tx_mcu values.
    x_line = np.linspace(min(x_mcu), max(x_mcu), 100)
    y_line = last_dump.slope * x_line + last_dump.intercept
    # Compute the confidence interval band as ± std_err.
    y_lower = y_line - 10*last_dump.std_err
    y_upper = y_line + 10*last_dump.std_err
    axs[0, 1].plot(x, tof, 'b--', label=f"tof var: {var:.3f}% dist spread: {dist_spread:.3f}m")
    # axs[0, 1].plot(x_mcu, y_mcu, 'b-', label="mcu 2 mcu")
    # axs[0, 1].plot(x_line, y_line, 'g-', label="Regression Line")
    # axs[0, 1].fill_between(x_line, y_lower, y_upper, color='green', alpha=0.3, label="Confidence Interval")
    # axs[0, 1].plot(x, dist, 'b--', label=f"tof var: {var:.3f}% dist var: {dist_var:.3f}%")
    # axs[0, 1].plot(rx_mcu, rx_uwb, 'r--', label="RX UWB")
    # axs[0, 1].axvline(x=invalide, color='k', linestyle='--', label="Invalid TOF")
    for inv in invalide:
        axs[0, 1].axvline(x=inv, color='k', linestyle='--', label="Invalid TOF")
    axs[0, 1].set_title("tof")

    axs[0, 1].legend()
    axs[0, 1].grid(True)
    # axs[0, 1].set_ylim(min_y, max_y)
    
    
    # Bottom-left: Adjusted TX & RX (MCU)
    # axs[1, 0].plot(x_line, y_line, 'g-', label="Regression Line")
    # axs[1, 0].fill_between(x_line, y_lower, y_upper, color='green', alpha=0.3, label="Confidence Interval")
    axs[1, 0].plot(x, adj_tx_mcu, 'g-', label="Adjusted TX MCU")
    axs[1, 0].plot(x, adj_rx_mcu, 'm-', label="Adjusted RX MCU")
    # axs[1, 0].axvline(x=invalide, color ='k', linestyle='--', label="Invalid TOF")
    for inv in invalide:
        axs[1, 0].axvline(x=inv, color='k', linestyle='--', label="Invalid TOF")
    axs[1, 0].set_title("Adjusted TX & RX (MCU)")
    axs[1, 0].set_xlabel("Event Count")
    axs[1, 0].set_ylabel("Timestamp")
    axs[1, 0].legend()
    axs[1, 0].grid(True)
    # axs[1, 0].set_ylim(min_y, max_y)
    
    # Bottom-right: Adjusted TX & RX (UWB)
    axs[1, 1].plot(x, adj_tx_uwb, 'g--', label="Adjusted TX UWB")
    axs[1, 1].plot(x, adj_rx_uwb, 'm--', label="Adjusted RX UWB")
    # axs[1, 1].axvlines(x=invalide, color ='k', linestyle='--', label="Invalid TOF")
    for inv in invalide:
        axs[1, 1].axvline(x=inv, color='k', linestyle='--', label="Invalid TOF")
    axs[1, 1].set_title("Adjusted TX & RX (UWB)")
    axs[1, 1].set_xlabel("Event Count")
    axs[1, 1].legend()
    axs[1, 1].grid(True)
    # axs[1, 1].set_ylim(min_y, max_y)
    
    # Set the overall figure title with count, slope, intercept, and std_err
    fig.suptitle(
        f" index: {index}, Count: {last_dump.index}, Slope: {last_dump.slope:.6f}, Intercept: {last_dump.intercept:.6f}, Std Err: {last_dump.std_err:.6f} \n avg offset: {offset:.3f} offset diff: {offset_diff} slope_perc: {abs(slope_perc)}\n avg tof: {avg_tof:.3f} \n avg dist: {avg_dist:.3f}m \n dist var: {dist_var:.3f}% \n",
        fontsize=14
    )
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(f"images/events_{index}.png")
    plt.show()


def parse_event(event_dict):
    tx = event_dict.get("TX", {})
    rx = event_dict.get("RX", {})
    adjusted_tx = event_dict.get("Adjusted TX", {})
    adjusted_rx = event_dict.get("Adjusted RX", {})
    return EventPair(
        tx=TimeUS(mcu=tx.get("MCU", 0.0), uwb=tx.get("UWB", 0.0)),
        rx=TimeUS(mcu=rx.get("MCU", 0.0), uwb=rx.get("UWB", 0.0)),
        adjusted_tx=TimeUS(mcu=adjusted_tx.get("MCU", 0.0), uwb=adjusted_tx.get("UWB", 0.0)),
        adjusted_rx=TimeUS(mcu=adjusted_rx.get("MCU", 0.0), uwb=adjusted_rx.get("UWB", 0.0)),
        tof_dtu=event_dict.get("ToF", 0.0),
        is_init=bool(event_dict.get("Is Init", 0)),
        is_SS=bool(event_dict.get("Is SS", 0))
    )

def parse_dump(dump_dict):
    index = dump_dict.get("index", 0)
    slope = dump_dict.get("slope", 0.0)
    intercept = dump_dict.get("intercept", 0.0)
    std_err = dump_dict.get("std_err", 0.0)
    events_list = dump_dict.get("EventBuffer", [])
    events = [parse_event(ev) for ev in events_list]
    return EventDump(index=index, slope=slope, intercept=intercept, std_err=std_err, events=events)

def clean_line(line):
    """
    Remove the log prefix (e.g., 'I (56317) SYNC: ') and return the rest of the line.
    """
    m = re.search(r'SYNC:\s*(.*)', line)
    if m:
        return m.group(1).strip()
    return line.strip()

def extract_json_blocks(lines):
    """
    Extract JSON blocks corresponding to event dumps.
    A dump is assumed to start after a "Printing event buffer" message when we see an opening '{'
    and ends when we see "End of event buffer". Only lines with 'SYNC:' are considered.
    """
    dumps = []
    collecting = False
    buffer_lines = []
    for line in lines:
        # Check for the start indicator; reset for a new dump.
        if "Printing event buffer" in line:
            collecting = False
            buffer_lines = []
            continue
        # If not yet collecting, check for the beginning of a JSON block.
        if not collecting:
            stripped = clean_line(line)
            if stripped.startswith("{"):
                collecting = True
                buffer_lines.append(stripped)
                continue
        else:
            # When the dump ends, try to parse the collected lines.
            if "End of event buffer" in line:
                json_text = "\n".join(buffer_lines)
                try:
                    dumps.append(json.loads(json_text))
                except Exception as e:
                    print("Error parsing JSON block:")
                    for i, l in enumerate(buffer_lines, start=1):
                        print(f"Line {i}: {l}")
                    print("Exception:", e)
                collecting = False
                buffer_lines = []
            else:
                # Only add lines that come from SYNC (skip other log lines)
                if "SYNC:" in line:
                    buffer_lines.append(clean_line(line))
    return dumps

def main():
    if len(sys.argv) != 2:
        print("Usage: python parse_dump.py <dump_file.txt>")
        sys.exit(1)

    dump_file = sys.argv[1]
    # index = int(sys.argv[2])
    with open(dump_file, "r") as f:
        lines = f.readlines()

    json_blocks = extract_json_blocks(lines)
    event_dumps = []
    for jb in json_blocks:
        try:
            event_dumps.append(parse_dump(jb))
        except Exception as e:
            print("Error parsing dump block:", e)

    print(f"Number of event tables recovered: {len(event_dumps)}")
    if event_dumps:
        last_dump = event_dumps[-1]
        print("Last event table (dump):")
        print(json.dumps(asdict(last_dump), indent=2))
        # graph_event_table(event_dumps[index])
        for i, dump in enumerate(event_dumps):
            graph_event_table(dump, i)
    print(f"Number of event tables recovered: {len(event_dumps)}")

if __name__ == '__main__':
    main()
