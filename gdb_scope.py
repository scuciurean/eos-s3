import gdb
import matplotlib.pyplot as plt
import numpy as np
import json

class VisualizeMemorySignals(gdb.Command):
    """Read a memory block based on JSON configuration and display signals like a waveform."""

    def __init__(self):
        super(VisualizeMemorySignals, self).__init__("visualize_signals", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        # Parse the argument: JSON config file path
        args = gdb.string_to_argv(arg)
        if len(args) != 1:
            print("Usage: visualize_signals <config_file.json>")
            return

        # Load the JSON configuration file
        config_file = args[0]
        with open(config_file, 'r') as f:
            config = json.load(f)

        # Extract configuration parameters
        address = int(config['start_address'], 16)  # Start address in hex
        word_size = config['word_size']             # Word size in bytes
        num_words = config['num_words']             # Number of words to read
        signals_info = config['signals']            # Signals info with names and bit positions

        # Create an empty list to hold signal data
        signals = {signal['name']: [] for signal in signals_info}  # Create a list for each signal

        # Read the memory from the target using GDB
        for i in range(num_words):
            value = gdb.selected_inferior().read_memory(address + i * word_size, word_size)
            value_int = int.from_bytes(value, byteorder='little')

            # Extract the specified signals based on their bit positions
            for signal in signals_info:
                bit_pos = signal['bit_position']
                signals[signal['name']].append((value_int >> bit_pos) & 1)  # Extract bit value (0 or 1)

        # Visualize the signals using matplotlib
        self.visualize_waveform(signals)

    def visualize_waveform(self, signals):
        num_signals = len(signals)
        num_samples = len(next(iter(signals.values()))) if num_signals > 0 else 0

        # Create a time vector (for plotting, it's just the sample index)
        time = np.arange(num_samples)

        # Set up the plot
        fig, ax = plt.subplots(figsize=(10, num_signals))

        for i, (signal_name, signal_data) in enumerate(signals.items()):
            ax.step(time, np.array(signal_data) + i * 2, where='post', label=signal_name)

        # Set labels, grid, and title
        ax.set_xlabel('Time (samples)')
        ax.set_ylabel('Value')
        ax.set_title('Probe Signals')
        ax.grid(True)
        ax.legend(loc='upper right')

        # Show the plot
        plt.show()

# Register the command with GDB
VisualizeMemorySignals()