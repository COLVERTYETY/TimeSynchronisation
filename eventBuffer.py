import gdb

# Adjust this value to match the ring-buffer size defined in your C code.
MAX_EVENTS = 1024

# Global dictionary to store stats for observed functions.
# Keys are function names, and values are dicts with keys: max_count, count, filename.
observed_function_stats = {}

# Global dictionary to store dump data in memory.
# Each key is a function name and its value is a list of dump strings.
internal_dump_buffers = {}

def record_ring_buffer_dump(func_name):
    """
    Retrieve the current contents of the ring buffer, format the dump with a header
    (including the dump iteration), and append the dump text to an internal (RAM) buffer.
    """
    try:
        event_index_val = gdb.parse_and_eval("event_index")
        event_index = int(event_index_val)
    except gdb.error as e:
        gdb.write("Error retrieving event_index: {}\n".format(e))
        return

    # Calculate how many events are stored.
    num_events = event_index if event_index < MAX_EVENTS else MAX_EVENTS
    # Determine the chronological start index.
    start = 0
    if event_index >= MAX_EVENTS:
        start = event_index % MAX_EVENTS

    try:
        event_buffer = gdb.parse_and_eval("event_buffer")
    except gdb.error as e:
        gdb.write("Error retrieving event_buffer: {}\n".format(e))
        return

    # Retrieve the current dump iteration for this function.
    iteration = observed_function_stats[func_name]["count"]

    # Build the dump text.
    dump_text = ""
    dump_text += ("Dump iteration {} for function '{}': Dumping {} events "
                  "(total events recorded = {})\n"
                  .format(iteration, func_name, num_events, event_index))
    dump_text += "cpu cycles     Type  Function pointer    Call site pointer\n"
    dump_text += "---------------------------------------------------------------\n"
    for i in range(num_events):
        idx = (start + i) % MAX_EVENTS
        # Each entry is a struct with members: timestamp, event_type, func, call_site.
        event = event_buffer[idx]
        timestamp = int(event["timestamp"])
        event_type = chr(int(event["event_type"]))
        func_ptr = event["func"]
        call_site_ptr = event["call_site"]
        dump_text += "{:20d} {}    {}    {}\n".format(timestamp, event_type, func_ptr, call_site_ptr)
    dump_text += "\n"

    # Append this dump text to the internal buffer for the function.
    if func_name not in internal_dump_buffers:
        internal_dump_buffers[func_name] = []
    internal_dump_buffers[func_name].append(dump_text)

def flush_internal_dump_buffer(func_name):
    """
    Flush the buffered dump data for a given function to its associated output file,
    then clear the internal buffer.
    """
    if func_name not in internal_dump_buffers or not internal_dump_buffers[func_name]:
        gdb.write("No dump data recorded for function '{}'.\n".format(func_name))
        return
    filename = observed_function_stats[func_name]["filename"]
    try:
        # Open in append mode so that the flush adds to existing file contents.
        f = open(filename, "a")
    except Exception as e:
        gdb.write("Error opening file '{}': {}\n".format(filename, e))
        return

    for dump in internal_dump_buffers[func_name]:
        f.write(dump)
    f.close()
    gdb.write("Flushed {} dump(s) for function '{}' to file '{}'.\n"
            .format(len(internal_dump_buffers[func_name]), func_name, filename))
    # Clear the internal buffer for that function.
    internal_dump_buffers[func_name] = []

class DumpOnExitFinishBreakpoint(gdb.FinishBreakpoint):
    """
    A finish breakpoint that records the ring buffer dump into the internal buffer when
    the observed function exits. If the dump count reaches max_count, it then flushes the
    internal buffer to the output file.
    """
    def __init__(self, func_name):
        super(DumpOnExitFinishBreakpoint, self).__init__(internal=True)
        self.func_name = func_name

    def stop(self):
        # Increment the dump count for this function.
        observed_function_stats[self.func_name]["count"] += 1
        # Record the current ring buffer dump into the internal buffer.
        record_ring_buffer_dump(self.func_name)
        # If we've reached the maximum dump count, flush the internal buffer to file.
        if observed_function_stats[self.func_name]["count"] >= observed_function_stats[self.func_name]["max_count"]:
            gdb.write("Maximum dump iterations reached for function '{}'. "
                    "Flushing dump data to file.\n".format(self.func_name))
            flush_internal_dump_buffer(self.func_name)
        return False  # Continue execution without stopping.

class ObserveFunctionExitBreakpoint(gdb.Breakpoint):
    """
    A breakpoint set at the entry of the function you wish to observe.
    When hit, it installs a finish breakpoint so that on exit the ring buffer dump is recorded.
    """
    def __init__(self, func, func_name):
        super(ObserveFunctionExitBreakpoint, self).__init__(func, gdb.BP_BREAKPOINT, internal=False)
        self.func_name = func_name

    def stop(self):
        stats = observed_function_stats[self.func_name]
        if stats["count"] >= stats["max_count"]:
            gdb.write("Maximum dump iterations reached for function '{}'. Disabling breakpoint.\n"
                      .format(self.func_name))
            self.delete()
            return False
        # At function entry, install a finish breakpoint for this frame.
        DumpOnExitFinishBreakpoint(self.func_name)
        return False

class ObserveFunctionExitCommand(gdb.Command):
    """
    Observe a function's exit and record the instrumentation ring buffer to an internal buffer.
    After all dumps are recorded (reaching max count or via manual flush), the dumps are saved to a file.
    
    Usage: observe_function_exit FUNCTION [MAX_COUNT] [OUTPUT_FILE]
    
    - If MAX_COUNT is not provided, the default maximum iterations is 300.
    - If OUTPUT_FILE is not provided, the dump will go to 'ring_buffer_dump.txt'.
    
    Each dump is tagged with a dump iteration indicator.
    """
    def __init__(self):
        super(ObserveFunctionExitCommand, self).__init__("observe_function_exit", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        if not args:
            gdb.write("Usage: observe_function_exit FUNCTION [MAX_COUNT] [OUTPUT_FILE]\n")
            return

        func = args[0]
        max_count = 300
        filename = "ring_buffer_dump.txt"
        if len(args) > 1:
            try:
                max_count = int(args[1])
            except ValueError:
                gdb.write("Invalid MAX_COUNT value: {}. Using default 300.\n".format(args[1]))
                max_count = 300
        if len(args) > 2:
            filename = args[2]

        # Initialize stats for this observed function.
        observed_function_stats[func] = {"max_count": max_count, "count": 0, "filename": filename}
        # Clear any pre-existing internal buffer for this function.
        internal_dump_buffers[func] = []
        # Set a breakpoint at the entry of the specified function.
        ObserveFunctionExitBreakpoint(func, func)
        gdb.write("Set a breakpoint on function '{}' to record ring buffer dumps to an internal buffer. "
                "They will be flushed to '{}' after {} iterations or via manual flush.\n"
                .format(func, filename, max_count))

class FlushRingBufferDumpsCommand(gdb.Command):
    """
    Flush the recorded ring buffer dumps from the internal buffer to the output file.
    
    Usage: flush_ring_buffer_dumps [FUNCTION]
    
    If FUNCTION is provided, flush dumps for that function; otherwise, flush dumps for all observed functions.
    """
    def __init__(self):
        super(FlushRingBufferDumpsCommand, self).__init__("flush_ring_buffer_dumps", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        if args:
            func = args[0]
            flush_internal_dump_buffer(func)
        else:
            for func in list(internal_dump_buffers.keys()):
                flush_internal_dump_buffer(func)

# Register the commands with GDB.
ObserveFunctionExitCommand()
FlushRingBufferDumpsCommand()
