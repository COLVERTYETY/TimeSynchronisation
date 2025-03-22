import re
import graphviz
import os
from collections import defaultdict
import colorsys
import zlib  # For computing a CRC32 hash

def name_to_color(name, s=0.7, v=0.95):
    """
    Compute a unique color for a given name using a CRC32 hash.
    """
    hue = zlib.crc32(name.encode('utf-8')) % 360
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, s, v)
    return f'#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}'

def parse_backtrace_log(file_path):
    """
    Parse the log file and return a mapping from each final function (from "Function:" lines)
    to a set of call paths. Each call path is a tuple of function names in call order (from the
    ultimate caller to the final function).
    """
    with open(file_path, 'r') as f:
        lines = f.readlines()

    call_paths = defaultdict(set)
    current_path = []
    current_function = None
    in_trace = False

    # This regex matches lines like:
    #   "#0  readfromspi (headerLength=68, ...)"
    #   "#1  0x4200b321 in dwt_xfer3000 (regFileID=68, ...)"
    frame_regex = re.compile(r"#\d+\s+(?:0x[\da-fA-F]+\s+in\s+)?(\w+)")

    for line in lines:
        line = line.strip()

        # Look for the function under trace.
        if line.startswith("Function:"):
            m = re.match(r"Function:\s*(\w+)", line)
            if m:
                current_function = m.group(1)
            continue

        # Start of a backtrace block.
        if line.startswith("Backtrace:"):
            in_trace = True
            current_path = []  # Reset the current call chain
            continue

        # Process frames within the backtrace.
        if in_trace and re.match(r"#\d+", line):
            m = frame_regex.search(line)
            if m:
                current_path.append(m.group(1))
            continue

        # A blank (or non-matching) line signals the end of the current trace.
        if in_trace and not line:
            if current_function and current_path:
                # The log prints the innermost function first so reverse to get caller-to-callee order.
                reversed_path = list(reversed(current_path))
                call_paths[current_function].add(tuple(reversed_path))
            in_trace = False

    # In case the file does not end with a blank line.
    if in_trace and current_function and current_path:
        reversed_path = list(reversed(current_path))
        call_paths[current_function].add(tuple(reversed_path))

    return call_paths

def print_unique_call_paths(call_paths):
    for final_func, paths in call_paths.items():
        print(f"Call paths leading to {final_func}:")
        for i, path in enumerate(sorted(paths), 1):
            print(f"Path {i}:")
            for func in path:
                print(f"  -> {func}")
            print()

def generate_graph(call_paths):
    dot = graphviz.Digraph(format='pdf')
    dot.attr(dpi='300')

    # Build the edges, as well as parent and child mappings.
    edges = set()
    parents = defaultdict(set)   # For each node, set of callers.
    children = defaultdict(set)  # For each node, set of callees.

    # For each final function and each call path (already in caller->callee order)
    for final_func, paths in call_paths.items():
        for path in paths:
            for i in range(len(path) - 1):
                caller = path[i]
                callee = path[i + 1]
                edges.add((caller, callee))
                parents[callee].add(caller)
                children[caller].add(callee)

    # Build the complete set of nodes.
    all_nodes = set()
    for a, b in edges:
        all_nodes.add(a)
        all_nodes.add(b)

    # Identify root nodes (those with no incoming edges) and end nodes (final functions).
    root_nodes = {node for node in all_nodes if not parents[node]}
    end_nodes = set(call_paths.keys())

    # --- SPECIAL BRANCH PROPAGATION ---
    # We wish to highlight all nodes on the special call path, starting at the critical
    # function "handle_dw3000_interrupt_task" and propagating downward (to the leaf).
    special_condition = "handle_dw3000_interrupt_task"
    special_set = set()
    if special_condition in all_nodes:
        to_visit = [special_condition]
        while to_visit:
            node = to_visit.pop()
            if node in special_set:
                continue
            special_set.add(node)
            for child in children[node]:
                if child not in special_set:
                    to_visit.append(child)
    # --- END SPECIAL PROPAGATION ---

    # --- COLOR ASSIGNMENT ---
    # We assign colors so that:
    #   - Root and end nodes get a fixed "lightblue" color.
    #   - Nodes with multiple parents get a unique color computed from their name.
    #   - Nodes with a single parent inherit their caller's color.
    node_color = {}
    def get_color(node, visited=None):
        if visited is None:
            visited = set()
        if node in node_color:
            return node_color[node]
        # Root and final nodes get the fixed color.
        if node in root_nodes or node in end_nodes:
            node_color[node] = "lightblue"
            return "lightblue"
        # If the node has more than one parent, assign a unique color.
        if len(parents[node]) > 1:
            node_color[node] = name_to_color(node)
            return node_color[node]
        # Otherwise (exactly one parent), inherit the parent's color.
        parent = next(iter(parents[node]))
        if parent in visited or parent in root_nodes:
            # To avoid cycles, fallback to a unique color.
            node_color[node] = name_to_color(node)
            return node_color[node]
        visited.add(parent)
        col = get_color(parent, visited)
        node_color[node] = col
        return col

    for node in all_nodes:
        get_color(node)
    # --- END COLOR ASSIGNMENT ---

    # --- ADD NODES TO THE GRAPH ---
    for node in all_nodes:
        # If the node is on the special branch, draw it with a bold hexagon.
        if node in special_set:
            dot.node(node, shape='hexagon', style='filled,bold', fillcolor=node_color.get(node, "black"))
        # Final functions (leaf nodes) are drawn as boxes.
        elif node in end_nodes:
            dot.node(node, shape='box', style='filled', fillcolor=node_color.get(node, "black"))
        else:
            dot.node(node, style='filled', fillcolor=node_color.get(node, "black"))

    # --- ADD EDGES TO THE GRAPH ---
    for caller, callee in edges:
        dot.edge(caller, callee)

    dot.render("call_graph")
    print("Graph saved as call_graph.pdf")
    os.system("xdg-open call_graph.pdf")

if __name__ == "__main__":
    log_file = "backtrace_trace.txt"  # Update with your actual log file path
    unique_paths = parse_backtrace_log(log_file)
    print_unique_call_paths(unique_paths)
    generate_graph(unique_paths)
