#!/usr/bin/env python3
"""
A script to parse MCU app trace dumps, build call‐graphs, and detect unique call
structures while accounting for discontinuities in the ring‐buffer.

The trace file contains dump sections whose headers include the expected number
of events. For example:

    Dump iteration 1 for function 'actual_interrupt': Dumping 10 events (total events recorded = 10)
    cpu cycles     Type  Function pointer    Call site pointer
    ---------------------------------------------------------------
           448718142 E    0x4200a95c <tx_conf_cb(dwt_cb_data_t const*)>    0x4200c7cd <dwt_isr()+173>
           448720148 E    0x4200a8dc <tx_notify(dwt_cb_data_t const*)>    0x4200a988 <tx_conf_cb(dwt_cb_data_t const*)+44>
           448721510 X    0x4200a8dc <tx_notify(dwt_cb_data_t const*)>    0x4200a988 <tx_conf_cb(dwt_cb_data_t const*)+44>
           448721920 X    0x4200a95c <tx_conf_cb(dwt_cb_data_t const*)>    0x4200c7cd <dwt_isr()+173>
           448742865 E    0x4200abf4 <rx_ok_cb(dwt_cb_data_t const*)>    0x4200c959 <dwt_isr()+569>
           448744179 E    0x4200ab04 <Frame_Router(dwt_cb_data_t const*)>    0x4200ac20 <rx_ok_cb(dwt_cb_data_t const*)+44>
           448756338 E    0x4200a9f0 <Respond_TWR_WAIT()>    0x4200ab8b <Frame_Router(dwt_cb_data_t const*)+135>
           448813948 X    0x4200a9f0 <Respond_TWR_WAIT()>    0x4200ab8b <Frame_Router(dwt_cb_data_t const*)+135>
           448814979 X    0x4200ab04 <Frame_Router(dwt_cb_data_t const*)>    0x4200ac20 <rx_ok_cb(dwt_cb_data_t const*)+44>
           448815072 X    0x4200abf4 <rx_ok_cb(dwt_cb_data_t const*)>    0x4200c959 <dwt_isr()+569>

Because of a ring‐buffer configuration issue, there may be “jumps” in the timestamp.
The continuity filter (see function filter_continuous_section()) will find the longest
continuous section (i.e. the one without gaps exceeding 200 µs) and use that section only.
Note that timestamps in the file are in CPU cycles; the conversion factor is 1 µs ≅ 240 cycles.
"""

import re
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional

# -----------------------------------------------------------------------------
# Data classes and node definitions
# -----------------------------------------------------------------------------

@dataclass
class Event:
    timestamp: float  # in microseconds
    type: str         # 'E' for enter, 'X' for exit
    func_name: str

class Node:
    """Node in the call tree."""
    def __init__(self, name: str):
        self.name = name
        self.children: List['Node'] = []

    def __repr__(self):
        return f"Node({self.name!r}, children={self.children!r})"

# -----------------------------------------------------------------------------
# Parsing functions
# -----------------------------------------------------------------------------

def parse_event_line(line: str) -> Optional[Event]:
    """
    Parse one line containing an event.
    
    Expected format (spaces may vary):
       <timestamp> <Type> 0x<addr> <function details> 0x<addr> <call site details>
    For example:
       448718142 E    0x4200a95c <tx_conf_cb(dwt_cb_data_t const*)>    0x4200c7cd <dwt_isr()+173>
    
    The function name is extracted from the text in the angle brackets
    (taking only the part before the '(').
    
    The timestamp is given in CPU cycles; it is converted to µs by dividing by 240.
    """
    pattern = re.compile(
        r"\s*(\d+)\s+([EX])\s+0x[\da-fA-F]+\s+<([^>]+)>\s+0x[\da-fA-F]+\s+<([^>]+)>"
    )
    match = pattern.match(line)
    if not match:
        return None

    timestamp_cycles = int(match.group(1))
    timestamp_us = timestamp_cycles / 240.0  # convert cycles to microseconds
    ev_type = match.group(2)
    func_detail = match.group(3)
    func_name = func_detail.split('(')[0].strip()

    return Event(timestamp=timestamp_us, type=ev_type, func_name=func_name)

def parse_dump(dump_lines: List[str]) -> List[Event]:
    """
    Given a list of event lines for one dump, parse and return the corresponding list of Event objects.
    """
    events = []
    for line in dump_lines:
        ev = parse_event_line(line)
        if ev is not None:
            events.append(ev)
    return events

def parse_file(file_path: str) -> List[List[Event]]:
    """
    Parse the trace file into a list of dumps.
    
    This parser uses the dump header to determine how many events to read.
    It recognizes headers of the form:
    
        Dump iteration 1 for function 'actual_interrupt': Dumping 10 events (total events recorded = 10)
    
    or
    
        Dumping 128 events (total events recorded = 128)
    
    After the header, it skips over any column headings or dashed separator lines,
    then reads exactly the number of event lines specified in the header.
    """
    dumps: List[List[Event]] = []
    header_pattern = re.compile(
        r"^Dump(?: iteration \d+ for function '.*?')?:\s*Dumping\s+(\d+)\s+events"
    )

    with open(file_path, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue

        header_match = header_pattern.match(line)
        if header_match:
            expected_count = int(header_match.group(1))
            i += 1
            # Skip over column headers and dashed lines.
            while i < len(lines):
                ln = lines[i].strip()
                if not ln:
                    i += 1
                    continue
                if ln.startswith("cpu cycles") or ln.startswith("Timestamp") or set(ln) == {"-"}:
                    i += 1
                else:
                    break

            dump_event_lines = []
            count = 0
            while i < len(lines) and count < expected_count:
                ln = lines[i].strip()
                if ln:
                    dump_event_lines.append(ln)
                    count += 1
                i += 1
            events = parse_dump(dump_event_lines)
            if len(events) != expected_count:
                print(f"Warning: Expected {expected_count} events, but parsed {len(events)} events.")
            dumps.append(events)
        else:
            i += 1

    return dumps

# -----------------------------------------------------------------------------
# Filtering for continuous sections (handling multiple gaps)
# -----------------------------------------------------------------------------

def filter_continuous_section(events: List[Event], max_gap_us: float = 200.0) -> List[Event]:
    """
    Given a list of events (assumed to be in timestamp order),
    return the longest continuous section (without any gap exceeding max_gap_us).
    
    If no gap is detected, all events are returned.
    """
    if not events:
        return events

    longest_section = []
    current_section = [events[0]]

    for prev, curr in zip(events, events[1:]):
        if (curr.timestamp - prev.timestamp) > max_gap_us:
            if len(current_section) > len(longest_section):
                longest_section = current_section
            current_section = [curr]
        else:
            current_section.append(curr)
    if len(current_section) > len(longest_section):
        longest_section = current_section

    return longest_section

# -----------------------------------------------------------------------------
# Building the call tree
# -----------------------------------------------------------------------------

def build_call_tree(events: List[Event]) -> List[Node]:
    """
    Build the call tree (or forest) from the list of events.
    
    Each 'E' event creates a node (pushed on a stack); each 'X' event pops from the stack.
    If there’s a mismatch in function names, a warning is printed.
    
    Returns a list of root nodes (top‐level calls).
    """
    roots: List[Node] = []
    stack: List[Node] = []

    for ev in events:
        if ev.type == 'E':
            node = Node(ev.func_name)
            if stack:
                stack[-1].children.append(node)
            else:
                roots.append(node)
            stack.append(node)
        elif ev.type == 'X':
            if not stack:
                print(f"Warning: Found exit event {ev} without a matching entry.")
                continue
            node = stack.pop()
            if node.name != ev.func_name:
                print(f"Warning: Mismatched exit. Expected exit for '{node.name}' but got exit for '{ev.func_name}'.")
    if stack:
        print("Warning: Some functions were entered but not exited:")
        for node in stack:
            print(f"  - {node.name}")
    return roots

def canonicalize_tree(nodes: List[Node]) -> Tuple:
    """
    Convert a list of nodes (the call tree) into a canonical nested tuple.
    Each node is represented as (function_name, (child1, child2, ...)).
    """
    return tuple(canonicalize_node(node) for node in nodes)

def canonicalize_node(node: Node) -> Tuple:
    return (node.name, canonicalize_tree(node.children))

# -----------------------------------------------------------------------------
# Pretty-printing
# -----------------------------------------------------------------------------

def pretty_print_tree(nodes: List[Node], indent: int = 0):
    """
    Recursively print the call tree in a human‐readable format.
    """
    for node in nodes:
        print("  " * indent + node.name)
        pretty_print_tree(node.children, indent + 1)

# -----------------------------------------------------------------------------
# Main routine
# -----------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python trace_parser.py <trace_file>")
        sys.exit(1)

    file_path = sys.argv[1]
    dumps = parse_file(file_path)
    if not dumps:
        print("No valid dumps were found in the file.")
        sys.exit(1)

    unique_branches = {}  # canonical tree -> list of dump indices using that tree

    for dump_index, events in enumerate(dumps[:2]):
        print(events)
        original_count = len(events)
        filtered_events = filter_continuous_section(events, max_gap_us=3000.0)
        if original_count != len(filtered_events):
            print(f"Dump {dump_index}: Truncated from {original_count} to {len(filtered_events)} events due to jump detection.")
        if not filtered_events:
            print(f"Dump {dump_index}: No continuous events found.")
            continue

        tree = build_call_tree(filtered_events)
        canon = canonicalize_tree(tree)
        unique_branches.setdefault(canon, []).append(dump_index)

    print(f"Found {len(unique_branches)} unique branch structure(s) out of {len(dumps)} dump(s).")
    print("=" * 40)
    for branch_num, (canon, dump_indices) in enumerate(unique_branches.items(), start=1):
        print(f"Unique Branch {branch_num} (occurrences: {len(dump_indices)}; dumps: {dump_indices}):")
        # Rebuild one call tree from the first dump with this branch to pretty-print.
        tree = build_call_tree(dumps[dump_indices[0]])
        pretty_print_tree(tree)
        print("-" * 40)

if __name__ == '__main__':
    main()
