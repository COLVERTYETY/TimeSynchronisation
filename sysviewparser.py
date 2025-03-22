import pandas as pd
import re
from graphviz import Digraph
import colorsys  # For HSV -> RGB conversion
import zlib      # For computing a CRC32 hash

# Regular expressions for extracting timing and resource info.
pattern_run   = re.compile(r"Runs for\s+([\d\.]+)\s*us", flags=re.IGNORECASE)
pattern_after = re.compile(r"runs after\s+([\d\.]+)\s*us", flags=re.IGNORECASE)
pattern_res   = re.compile(r"xQueue=([^\s]+)")
pattern_prio = re.compile(r"Priority=(\d+)", flags=re.IGNORECASE)

def compute_stats(values):
    """Compute average, min, max and spread from a list of numeric values."""
    if not values:
        return None, None, None, None
    avg = sum(values) / len(values)
    minimum = min(values)
    maximum = max(values)
    spread = maximum - minimum
    return avg, minimum, maximum, spread

def name_to_color(name, s=0.7, v=0.95):
    """
    Given a node name, compute a unique color.
    
    - Compute a CRC32 hash of the name, then take it modulo 360 to get a hue value.
    - Convert from HSV to RGB and format as a hex color string.
    """
    hue = zlib.crc32(name.encode('utf-8')) % 360
    r, g, b = colorsys.hsv_to_rgb(hue/360.0, s, v)
    return f'#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}'

# Dictionaries to collect timing stats per task (for "Runs for")
exec_times = {}   # key: task name, value: list of "Runs for" times (in us)

# We record two groups of edges:
#   - normal_edges: direct call from a source task to a target task.
#   - resource_edges: when "xQueue" is present, we create edges:
#         (a) from the calling task to a resource node,
#         (b) from that resource node to the target task.
normal_edges = []  # each entry: {src, target, event, delays (list)}
res_edges_src = []     # edge from calling task to resource: {src, resource, event}
res_edges_target = []  # edge from resource to target task: {resource, target, event, delays (list)}

# Dictionary to keep resource nodes.
resource_nodes = {}

# Some events or tasks we don't want to display.
no_display_contexts = ["Idle", "SysTick"]
no_display_events = ["ISR Exit", "Stack Info", "Log", "Task Info", "Task Create"]

if __name__ == "__main__":
    # ------------------------------------------------------------------------
    # 1. Load and integrate data from both cores.
    # csv_files = ["expdata_core1.csv", "expdata_core2.csv"]
    # csv_files = ["uwb_core1.csv", "uwb_core2.csv"]
    # csv_files = ["noprint_core1.csv", "noprint_core2.csv"]
    # csv_files = ["notify_core1.csv", "notify_core2.csv"]
    csv_files = ["full_core1.csv", "full_core2.csv"]

    df_list = []
    for path in csv_files:
        temp_df = pd.read_csv(path)
        # Add a 'core' column based on the filename.
        if "core1" in path:
            temp_df['core'] = "core1"
        elif "core2" in path:
            temp_df['core'] = "core2"
        else:
            temp_df['core'] = "unknown"
        df_list.append(temp_df)
    # Concatenate the data from both cores.
    df = pd.concat(df_list, ignore_index=True)

    # ------------------------------------------------------------------------
    # 2. Build a mapping of task names to the set of cores where they appear.
    #    We do not change the task name; we record core info separately.
    task_cores = {}
    for idx, row in df.iterrows():
        task = row['context']  # task name remains unchanged
        core = row['core']
        task_cores.setdefault(task, set()).add(core)
    
    task_priority = {}
    for idx, row in df.iterrows():
        context = row['context']
        event = row['event']
        detail = str(row['detail'])  # ensure detail is a string
        if context == "Idle" and event == "Task Info":
            # print(detail)
            m_prio = pattern_prio.search(detail)
            # print(m_prio)
            if m_prio:
                try:
                    priority = int(m_prio.group(1))
                    src = detail.split(" (")[0]
                    task_priority.setdefault(src, []).append(priority)
                except ValueError:
                    pass
    print(task_priority)


    # Use the unique task names as nodes.
    tasks = df['context'].unique()

    # ------------------------------------------------------------------------
    # 3. Process every row to extract timing and edge info.
    for idx, row in df.iterrows():
        src = row['context']   # task name without core info
        if src in no_display_contexts:
            continue

        event  = row['event']
        if event in no_display_events:
            continue

        detail = str(row['detail'])  # ensure detail is a string

        # 3a. Extract execution timing ("Runs for").
        m_run = pattern_run.search(detail)
        if m_run:
            try:
                runtime = float(m_run.group(1))
                exec_times.setdefault(src, []).append(runtime)
            except ValueError:
                pass

        # 3b. Check for "runs after ..." delay.
        m_after = pattern_after.search(detail)
        delay_value = None
        if m_after:
            try:
                delay_value = float(m_after.group(1))
            except ValueError:
                pass

        # 3c. Determine if any target tasks are mentioned in the detail.
        # We scan the detail for task names from our tasks list.
        target_tasks = [t for t in tasks if t != src and t in detail]

        # 3d. Process resource events containing "xQueue".
        if "xQueue" in detail:
            m_res = pattern_res.search(detail)
            if m_res:
                resource_address = m_res.group(1)
                resource_id = f"res_{resource_address}"
                resource_nodes[resource_id] = resource_address
            else:
                resource_id = "res_unknown"
                resource_nodes.setdefault(resource_id, "Unknown Resource")
            # Create an edge from the source task to the resource.
            res_edges_src.append({
                "src": src,
                "resource": resource_id,
                "event": event
            })
            # And for each target task mentioned, create an edge from the resource.
            for target in target_tasks:
                res_edges_target.append({
                    "resource": resource_id,
                    "target": target,
                    "event": event,
                    "delays": [delay_value] if delay_value is not None else []
                })
        else:
            # 3e. For normal events, create an edge from source task to each target task.
            for target in target_tasks:
                # Merge events with the same (src, target, event) by appending delays.
                found = False
                for edge in normal_edges:
                    if edge["src"] == src and edge["target"] == target and edge["event"] == event:
                        if delay_value is not None:
                            edge["delays"].append(delay_value)
                        found = True
                        break
                if not found:
                    normal_edges.append({
                        "src": src,
                        "target": target,
                        "event": event,
                        "delays": [delay_value] if delay_value is not None else []
                    })

    # ------------------------------------------------------------------------
    # 4. Deduplicate resource edges.
    # a) For source-to-resource edges.
    unique_res_edges_src = {}
    for r_edge in res_edges_src:
        key = (r_edge["src"], r_edge["resource"], r_edge["event"])
        if key not in unique_res_edges_src:
            unique_res_edges_src[key] = r_edge
    dedup_res_edges_src = list(unique_res_edges_src.values())

    # b) For resource-to-target edges, merge delays if needed.
    unique_res_edges_target = {}
    for r_edge in res_edges_target:
        key = (r_edge["resource"], r_edge["target"], r_edge["event"])
        if key not in unique_res_edges_target:
            unique_res_edges_target[key] = {
                "resource": r_edge["resource"],
                "target": r_edge["target"],
                "event": r_edge["event"],
                "delays": []
            }
        if r_edge["delays"]:
            unique_res_edges_target[key]["delays"].extend(r_edge["delays"])
    dedup_res_edges_target = list(unique_res_edges_target.values())

    # ------------------------------------------------------------------------
    # 5. Compute per-task execution statistics.
    stats = {}
    for task in tasks:
        avg_run, min_run, max_run, spread_run = compute_stats(exec_times.get(task, []))
        stats[task] = {
            "avg_run": avg_run,
            "min_run": min_run,
            "max_run": max_run,
            "spread_run": spread_run,
        }

    # ------------------------------------------------------------------------
    # 6. Build the Graphviz graph.
    dot = Digraph(comment='Task/Interrupt Interaction Graph', format='pdf')
    # dot = Digraph(comment='Task/Interrupt Interaction Graph', format='pdf', engine='neato')
    # Set graph attributes for a more relaxed layout.
    # dot.attr(overlap='false', splines='true', sep='+10')
    
    # Create a node for each task.
    for task in tasks:
        # Build a label that includes:
        #   - A prefix listing the cores where the task was observed.
        #   - The task name.
        #   - Priority (if available).
        #   - Execution statistics (if available).
        cores = sorted(task_cores.get(task, []))
        label = f"{','.join(cores)}: {task}" if cores else task
        if task in task_priority:
            label += f" p:{task_priority[task][0]}"
        st = stats.get(task, {})
        if st.get("avg_run") is not None:
            label += f"\nExec: avg {st['avg_run']:.2f}us (spread {st['spread_run']:.2f}us)"
        dot.node(task, label, style='filled', fillcolor=name_to_color(task))

    # Create a node for each resource (displayed as a square).
    for res_id, res_label in resource_nodes.items():
        # A simple gray fill for resource nodes.
        dot.node(res_id, res_label, shape="box", fillcolor="#787878")

    # Add normal (direct) edges.
    for edge in normal_edges:
        label = edge["event"]
        if edge["delays"]:
            avg_delay, min_delay, max_delay, spread_delay = compute_stats(edge["delays"])
            label += f"\nDelay: avg {avg_delay:.2f}us (spread {spread_delay:.2f}us)"
            # label = ""
        dot.edge(edge["src"], edge["target"], label=label)

    # Add resource edges:
    # a) From calling task to resource.
    for r_edge in dedup_res_edges_src:
        label = r_edge["event"]
        if "xQueueGeneric" in label:
            label = label.split("xQueueGeneric")[1]
        # label = ""
        dot.edge(r_edge["src"], r_edge["resource"], label=label, color=name_to_color(r_edge["src"]), fontcolor=name_to_color(r_edge["src"]))
    # b) From resource to target task.
    for r_edge in dedup_res_edges_target:
        label = r_edge["event"]
        if r_edge["delays"]:
            avg_delay, min_delay, max_delay, spread_delay = compute_stats(r_edge["delays"])
            label += f"\nDelay: avg {avg_delay:.2f}us (spread {spread_delay:.2f}us)"
        dot.edge(r_edge["resource"], r_edge["target"], label=label)

    # Render (and open, if supported) the resulting graph as a PDF.
    dot.render('task_graph', view=True)
