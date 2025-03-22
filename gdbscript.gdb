# Define log file
set logging file function_trace.log
set logging enabled on
set logging overwrite on  # Overwrite file on restart
set logging redirect on  # Avoid console clutter

# Counter for function calls
set $count = 0
set $max_count = 10

# Define timestamps (ESP32 lacks real clocks in GDB, so use CPU cycles)
define hook-stop
    if ($count < $max_count)
        set $start = (uint32_t)__builtin_readcyclecounter()
        printf "Function entered at cycle: %u\n", $start
    end
end

# Break on function entry
break actual_interrupt
commands
    if ($count >= $max_count)
        printf "Captured %d calls, stopping trace.\n", $count
        detach
        quit
    end
    set $start_time = (uint32_t)__builtin_readcyclecounter()
    printf "Entering function at cycle: %u (Call #%d)\n", $start_time, $count
    set $count = $count + 1
    continue
end

# Break on function return
finish
commands
    set $end_time = (uint32_t)__builtin_readcyclecounter()
    set $duration = $end_time - $start_time
    printf "Function exited at cycle: %u, Duration: %u cycles\n", $end_time, $duration
    continue
end

# Start program
continue
