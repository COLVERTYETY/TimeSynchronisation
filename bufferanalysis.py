#!/usr/bin/env python3
import re
from collections import defaultdict
import colorsys
import zlib 
from graphviz import Digraph

# ============================
# Global Constants and Regexes
# ============================
CPU_FREQ = 240e6  # 240 MHz
CYCLE_TO_NS = 1e9 / CPU_FREQ
CYCLE_TO_US = 1e6 / CPU_FREQ

header_re = re.compile(
    r"Dump iteration (\d+) for function '([^']+)': Dumping (\d+) events \(total events recorded = (\d+)\)"
)
event_re = re.compile(r"^\s*(\d+)\s+([EX])\s+0x[0-9a-fA-F]+\s+<(\w+)\(")

def name_to_color(name, s=0.7, v=0.95):
    """
    Compute a unique color for a given name using a CRC32 hash.
    """
    hue = zlib.crc32(name.encode('utf-8')) % 360
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, s, v)
    return f'#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}'

# ============================
# Global Debug Table Helpers
# ============================

def collect_global_debug_rows(scenario_groups):
    """
    Traverse every aggregated call tree from every scenario and group timing statistics
    by unique function name. Returns a list of rows. Each row will have:
      [Function, Avg (us), Min (us), Max (us), Spread (us)]
    """
    global_stats = {}
    def traverse(node):
        name = node['func']
        if name not in global_stats:
            global_stats[name] = {"avg": [], "min": [], "max": []}
        global_stats[name]["avg"].append(node["avg_duration"])
        global_stats[name]["min"].append(node["min_duration"])
        global_stats[name]["max"].append(node["max_duration"])
        for child in node["children"]:
            traverse(child)
    # Loop over all scenarios
    for group in scenario_groups.values():
        aggregated_roots = aggregate_scenario(group["graphs"])
        for root in aggregated_roots:
            traverse(root)
    # Create rows: header plus one row per unique function.
    rows = []
    rows.append(["Function", "Avg (us)", "Min (us)", "Max (us)", "Spread (us)"])
    for func, stats in global_stats.items():
        avg = sum(stats["avg"]) / len(stats["avg"]) / 1000.0
        min_val = min(stats["min"]) / 1000.0
        max_val = max(stats["max"]) / 1000.0
        spread = max_val - min_val
        rows.append([func, f"{avg:.2f}", f"{min_val:.2f}", f"{max_val:.2f}", f"{spread:.2f}"])
    #  sort according to avg duration
    rows[1:] = sorted(rows[1:], key=lambda x: float(x[1]))
    return rows

def generate_global_debug_table_html(scenario_groups):
    """
    Generate an HTML table (Graphviz HTML-like label) containing runtime statistics for each unique function.
    """
    rows = collect_global_debug_rows(scenario_groups)
    html = '<<TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">'
    # Header row.
    html += '<TR>'
    for col in rows[0]:
        html += f'<TD BGCOLOR="lightgrey"><B>{col}</B></TD>'
    html += '</TR>'
    # Data rows.
    for row in rows[1:]:
        html += '<TR>'
        for col in row:
            html += f'<TD>{col}</TD>'
        html += '</TR>'
    html += '</TABLE>>'
    return html

def generate_global_debug_table_pretty(scenario_groups):
    """
    Generate a pretty-printed table containing runtime statistics for each unique function.
    """
    rows = collect_global_debug_rows(scenario_groups)
    col_widths = [max(len(str(cell)) for cell in col) for col in zip(*rows)]
    table = []

    # Header row.
    header = " | ".join(f"{col:<{col_widths[i]}}" for i, col in enumerate(rows[0]))
    table.append(header)
    table.append("-+-".join('-' * width for width in col_widths))

    # Data rows.
    for row in rows[1:]:
        table.append(" | ".join(f"{col:<{col_widths[i]}}" for i, col in enumerate(row)))

    return "\n".join(table)

# ============================
# Visualization Function
# ============================
def visualize_scenarios(scenario_groups, output_filename="scenarios"):
    """
    Create a Graphviz visualization for all unique scenarios.
    For each scenario group, overall timing (from first and last event) is aggregated
    and a detailed call-graph (with per-node aggregated statistics) is drawn.
    A global debug table (with one row per unique function) is appended as an HTML node.
    """
    dot = Digraph(comment='Unique Scenarios', format='pdf')
    scenario_index = 1

    # Process each scenario group.
    for canon, group in scenario_groups.items():
        graphs = group["graphs"]
        total_times = group["total_times"]
        if not total_times:
            continue
        avg_total = sum(total_times) / len(total_times)
        min_total = min(total_times)
        max_total = max(total_times)
        spread = max_total - min_total

        aggregated_roots = aggregate_scenario(graphs)
        print(f"Scenario {scenario_index} has {len(aggregated_roots)} root nodes.")
        number_appearances = len(group["graphs"])
        number_of_nodes = sum(len(agg['children']) for agg in aggregated_roots)
        cluster_name = f'cluster_{scenario_index}'
        with dot.subgraph(name=cluster_name) as sub:
            sub.attr(label=f"Scenario {scenario_index}\nTotal Avg: {avg_total/CYCLE_TO_NS/1000:.2f} us, Spread: {spread/CYCLE_TO_NS/1000:.2f} us, Appearances: {number_appearances}, N_nodes: {number_of_nodes}",
                     style='filled', color='lightgrey', fontsize='12')
            
            node_counter = [0]
            def add_node_recursive(agg_node, parent_id=None):
                current_id = f"s{scenario_index}_n{node_counter[0]}"
                node_counter[0] += 1
                label = (f"{agg_node['func']}\n"
                         f"avg: {agg_node['avg_duration']/1000:.2f} us\n"
                         f"spread: {(agg_node['max_duration']-agg_node['min_duration'])/1000:.2f} us")
                color = name_to_color(agg_node['func'])
                sub.node(current_id, label=label, style='filled', fillcolor=color)
                if parent_id is not None:
                    sub.edge(parent_id, current_id)
                child_ids = []
                for child in agg_node['children']:
                    cid = add_node_recursive(child, current_id)
                    child_ids.append(cid)
                for i in range(1, len(child_ids)):
                    sub.edge(child_ids[i-1], child_ids[i], label=f"{i+1}", style="dashed", color="blue")
                return current_id

            root_ids = []
            for agg in aggregated_roots:
                rid = add_node_recursive(agg)
                root_ids.append(rid)
            for i in range(1, len(root_ids)):
                sub.edge(root_ids[i-1], root_ids[i], label=f"{i+1}", style="dashed", color="blue")
        scenario_index += 1

    # add global debug table
    debug_table_html = generate_global_debug_table_html(scenario_groups)
    dot.node('global_debug_table', label=debug_table_html, shape='plaintext', fontsize='20')

    # Generate and save the global debug table to a separate file.
    # debug_table_html = generate_global_debug_table_html(scenario_groups)
    # with open("global_debug_table.html", "w") as f:
    #     f.write(debug_table_html)
    # print("Global debug table saved as global_debug_table.html")
    print(generate_global_debug_table_pretty(scenario_groups))
    
    dot.render(output_filename, view=True)
    print("Graphviz PDF generated as", output_filename + ".pdf")

# ============================
# Parsing Functions
# ============================
def parse_log(log_content):
    dumps = []
    lines = log_content.splitlines()
    current_dump = None
    prev_event_count = 0
    collecting_events = False

    for line in lines:
        header_match = header_re.match(line)
        if header_match:
            if current_dump is not None:
                total_events = current_dump['total_events']
                new_events = current_dump['events'][prev_event_count:]
                current_dump['new_events'] = new_events
                prev_event_count = total_events
                dumps.append(current_dump)
            dump_number = int(header_match.group(1))
            critical_func = header_match.group(2)
            dumping_events = int(header_match.group(3))
            current_dump = {
                'dump_number': dump_number,
                'critical_func': critical_func,
                'total_events': dumping_events,
                'events': []
            }
            collecting_events = False
            continue

        if line.startswith("cpu cycles") or line.startswith("---"):
            collecting_events = True
            continue

        if collecting_events and current_dump is not None:
            if not line.strip():
                continue
            m = event_re.search(line)
            if m:
                cycles = int(m.group(1))
                event_type = m.group(2)
                func_name = m.group(3)
                event = {
                    'cycles': cycles,
                    'time_ns': cycles * CYCLE_TO_NS,
                    'type': event_type,
                    'func': func_name
                }
                current_dump['events'].append(event)
            else:
                parts = line.strip().split()
                if len(parts) >= 4:
                    try:
                        cycles = int(parts[0])
                        event_type = parts[1]
                        m2 = re.search(r'<(\w+)\(', line)
                        func_name = m2.group(1) if m2 else parts[2]
                        event = {
                            'cycles': cycles,
                            'time_ns': cycles * CYCLE_TO_NS,
                            'type': event_type,
                            'func': func_name
                        }
                        current_dump['events'].append(event)
                    except Exception as e:
                        print(f"Error parsing line: {line}")
                else:
                    print(f"Unrecognized line format: {line}")
    if current_dump is not None:
        total_events = current_dump['total_events']
        new_events = current_dump['events'][prev_event_count:]
        current_dump['new_events'] = new_events
        dumps.append(current_dump)
    print(f"Parsed {len(dumps)} dumps.")
    return dumps

# ============================
# Building the Call-Graph
# ============================
def build_call_graph(events):
    stack = []
    roots = []
    for ev in events:
        if ev['type'] == 'E':
            node = {
                'func': ev['func'],
                'start_time_ns': ev['time_ns'],
                'end_time_ns': None,
                'duration_ns': None,
                'children': []
            }
            if stack:
                stack[-1]['children'].append(node)
            else:
                roots.append(node)
            stack.append(node)
        elif ev['type'] == 'X':
            if not stack:
                print("Warning: exit event encountered with empty call stack!")
                continue
            node = stack.pop()
            node['end_time_ns'] = ev['time_ns']
            node['duration_ns'] = node['end_time_ns'] - node['start_time_ns']
        else:
            print("Unknown event type:", ev['type'])
    if stack:
        print("Warning: call stack is not empty at the end of the dump!")
    return roots

def print_graph(nodes, indent=0):
    for node in nodes:
        ind = "  " * indent
        dur = node['duration_ns'] if node['duration_ns'] is not None else 0
        print(f"{ind}{node['func']} (duration: {dur:.2f} ns)")
        if node['children']:
            print_graph(node['children'], indent + 1)

# ============================
# Scenario Comparison and Aggregation
# ============================
def canonicalize_nodes(nodes):
    canon_list = []
    for node in nodes:
        children_canon = canonicalize_nodes(node['children'])
        canon_list.append((node['func'], tuple(children_canon)))
    return tuple(canon_list)

def aggregate_tree(nodes_list):
    func = nodes_list[0]['func']
    durations = [node['duration_ns'] for node in nodes_list if node['duration_ns'] is not None]
    if durations:
        avg_duration = sum(durations) / len(durations)
        min_duration = min(durations)
        max_duration = max(durations)
    else:
        avg_duration = min_duration = max_duration = 0

    aggregated_children = []
    num_children = len(nodes_list[0]['children'])
    for i in range(num_children):
        child_nodes = [node['children'][i] for node in nodes_list]
        aggregated_children.append(aggregate_tree(child_nodes))

    total_avg_duration = avg_duration
    total_min_duration = min_duration
    total_max_duration = max_duration

    return {
        'func': func,
        'avg_duration': avg_duration,
        'min_duration': min_duration,
        'max_duration': max_duration,
        'total_avg_duration': total_avg_duration,
        'total_min_duration': total_min_duration,
        'total_max_duration': total_max_duration,
        'children': aggregated_children
    }

def aggregate_scenario(dump_graphs):
    num_roots = len(dump_graphs[0])
    aggregated = []
    for i in range(num_roots):
        nodes_list = [graph[i] for graph in dump_graphs]
        aggregated.append(aggregate_tree(nodes_list))
    return aggregated

def print_aggregated_tree(agg, indent=0):
    ind = "  " * indent
    if indent == 1:
        print(f"{ind}[total avg: {agg['total_avg_duration']/1000:.2f} us, total spread: {(agg['total_max_duration']-agg['total_min_duration'])/1000:.2f} us]")
    print(f"{ind}{agg['func']} (avg: {agg['avg_duration']/1000:.2f} us, spread: {(agg['max_duration']-agg['min_duration'])/1000:.2f} us)")
    for child in agg['children']:
        print_aggregated_tree(child, indent + 1)

# ============================
# Main Processing
# ============================
def main():
    with open("ring_buffer_dump.txt", "r") as file:
        log_content = file.read()

    dumps = parse_log(log_content)
    for dump in dumps:
        dump['call_graph'] = build_call_graph(dump['new_events'])
        if dump['new_events']:
            dump['scenario_start'] = dump['new_events'][0]['time_ns']
            dump['scenario_end'] = dump['new_events'][-1]['time_ns']
            dump['scenario_total_time'] = dump['scenario_end'] - dump['scenario_start']
        else:
            dump['scenario_total_time'] = 0

    if dumps:
        print("Call Graph for Dump 1:")
        print_graph(dumps[0]['call_graph'])
    if len(dumps) >= 2:
        print("\nCall Graph for Dump 2:")
        print_graph(dumps[1]['call_graph'])

    scenario_groups = defaultdict(lambda: {"graphs": [], "total_times": []})
    for dump in dumps:
        canon = canonicalize_nodes(dump['call_graph'])
        scenario_groups[canon]["graphs"].append(dump['call_graph'])
        scenario_groups[canon]["total_times"].append(dump['scenario_total_time'])
    
    print("\nUnique Scenarios Detected:", len(scenario_groups))
    scenario_idx = 1
    for canon, group in scenario_groups.items():
        print(f"\nScenario {scenario_idx}:")
        print(canon)
        print("Aggregated Runtime Statistics:")
        aggregated = aggregate_scenario(group["graphs"])
        for agg in aggregated:
            print_aggregated_tree(agg, indent=1)
        scenario_idx += 1

    visualize_scenarios(scenario_groups, output_filename="scenarios")

if __name__ == '__main__':
    main()
