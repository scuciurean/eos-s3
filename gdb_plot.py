import gdb
import numpy as np
import matplotlib.pyplot as plt

class PlotCommand(gdb.Command):
    """Plot buffer data from a memory address."""
    
    def __init__(self):
        super(PlotCommand, self).__init__("plot_buffer", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        if len(args) < 2:
            print("Usage: plot_buffer <pointer> <size>")
            return
        
        buffer_ptr = int(gdb.parse_and_eval(args[0]))
        size = int(args[1])
        
        data = []
        for i in range(size):
            data.append(int(gdb.parse_and_eval(f"((int*)({buffer_ptr}))[{i}]")))
        # data = np.array(data)

            # value = int(gdb.parse_and_eval(f"((int*)({buffer_ptr}))[{i}]"))
            # if value > 0x7FFF:  # Check if the value is greater than the max value of int16
            #     value -= 0x10000  # Convert to negative value
            # data.append(value)
        # data = np.array(data, dtype=np.int16)
        plt.plot(data)
        plt.title("Buffer Plot")
        plt.xlabel("Index")
        plt.ylabel("Value")
        plt.show()

# Register the command with GDB
PlotCommand()

