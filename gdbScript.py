import gdb

class BacktraceTracer:
    def __init__(self, func_names, output_file="backtrace_trace.txt", max_records=100):
        self.func_names = func_names
        self.output_file = output_file
        self.max_records = max_records
        self.records = []         # List to store recorded backtrace events.
        self.breakpoints = []     # List to hold breakpoints created by this tracer.
        gdb.write("Backtrace tracing started for: {} with max records: {}\n".format(
            ", ".join(self.func_names), self.max_records))

    def record_event(self, func, pc, backtrace):
        """Record a backtrace event and, if the maximum record count is reached,
        automatically save the records and disable further tracing."""
        record = {"func": func, "pc": pc, "bt": backtrace}
        self.records.append(record)
        gdb.write("Recorded backtrace for {} at {}:\n".format(func, pc))
        for line in backtrace.splitlines():
            gdb.write("  " + line + "\n")
        # If maximum records reached, save and disable further recording.
        if len(self.records) >= self.max_records:
            gdb.write("Maximum record count reached ({} records). Saving records to {} "
                      "and disabling breakpoints.\n".format(self.max_records, self.output_file))
            self.save_records()
            for bp in self.breakpoints:
                bp.delete()

    def save_records(self):
        """Save all recorded backtraces to the specified output file."""
        try:
            with open(self.output_file, "w") as f:
                for rec in self.records:
                    f.write("Function: {} at {}\n".format(rec["func"], rec["pc"]))
                    f.write("Backtrace:\n")
                    for line in rec["bt"].splitlines():
                        f.write("  " + line + "\n")
                    f.write("\n")
            gdb.write("Saved backtrace trace to {}\n".format(self.output_file))
        except Exception as e:
            gdb.write("Error saving backtrace trace: {}\n".format(e))

class BacktraceBreakpoint(gdb.Breakpoint):
    """A breakpoint that fires when one of the target functions is hit and records its backtrace."""
    def __init__(self, func, tracer):
        # Create a breakpoint at the given function.
        super().__init__(func, gdb.BP_BREAKPOINT, internal=False)
        self.tracer = tracer
        self.func = func

    def stop(self):
        # When the breakpoint is hit, record the backtrace.
        frame = gdb.newest_frame()
        if frame is not None:
            func_name = frame.name()
            pc = frame.pc() 
            bt = gdb.execute("bt", to_string=True)
            self.tracer.record_event(func_name, pc, bt)
        return False  # Continue execution automatically.

class TraceBacktraceCommand(gdb.Command):
    """Trace and record backtraces when a target function is hit.
Usage: trace_backtrace FUNCTION [FUNCTION] [MAX_RECORDS] [OUTPUT_FILE]
If two function names are provided, backtraces are recorded when either is hit.
The default output file is 'backtrace_trace.txt' and the default maximum records is 100.
Once the maximum record count is reached, the records are automatically saved and tracing stops.
"""
    def __init__(self):
        super().__init__("trace_backtrace", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        if not args:
            gdb.write("Usage: trace_backtrace FUNCTION [FUNCTION] [MAX_RECORDS] [OUTPUT_FILE]\n")
            return

        # Parse the output file if provided (it should end with '.txt').
        output_file = "backtrace_trace.txt"
        if args[-1].endswith(".txt"):
            output_file = args.pop(-1)

        # Parse max_records if provided (an integer).
        max_records = 100
        try:
            possible_max = int(args[-1])
            max_records = possible_max
            args.pop(-1)
        except ValueError:
            pass

        if len(args) < 1 or len(args) > 2:
            gdb.write("Usage: trace_backtrace FUNCTION [FUNCTION] [MAX_RECORDS] [OUTPUT_FILE]\n")
            return

        # The remaining arguments are the function name(s).
        func_names = args
        tracer = BacktraceTracer(func_names, output_file, max_records)
        for func in func_names:
            bp = BacktraceBreakpoint(func, tracer)
            tracer.breakpoints.append(bp)
        gdb.write("Backtrace tracer installed for: {}\n".format(", ".join(func_names)))
        gdb.write("Maximum records: {}\n".format(max_records))
        gdb.write("Output file: {}\n".format(output_file))
        gdb.write("To save the recorded backtraces manually (if needed), call:\n  python tracer_instance.save_records()\n")
        global tracer_instance
        tracer_instance = tracer

TraceBacktraceCommand()
