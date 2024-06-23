import pandas as pd
import matplotlib.pyplot as plt
import sys
import multiprocessing

def plot_data(file1, file2, columns, title):
    # Create a figure and a 3x2 grid of subplots
    fig, axs = plt.subplots(3, 2, figsize=(10, 15))
    fig.suptitle('Planned vs Executed Joint States - ' + title)

    # Plot the data from each file in a separate subplot
    for i, column in enumerate(columns[:-1]):  # Exclude 'timestamp[ms]' column
        ax = axs[i//2, i%2]
        ax.plot(file1['timestamp[ms]'].to_numpy(), file1[column].to_numpy(), label=f'Generated {column}', color='blue')
        ax.plot(file2['timestamp[ms]'].to_numpy(), file2[column].to_numpy(), label=f'Executed {column}', linestyle='--', color='red')
        ax.legend()

    # Show the plot
    plt.tight_layout()
    plt.show()

def visualize_data(title, home_dir, inputfile1, inputfile2):
    print(f"Visualizing data from {home_dir}/{inputfile1} and {home_dir}/{inputfile2}")
    
    # Define column names
    columns = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'timestamp[ms]']

    # Load the data from the files
    file1 = pd.read_csv(home_dir + '/' + inputfile1, sep='\s+', names=columns, skiprows=1)
    file2 = pd.read_csv(home_dir + '/' + inputfile2, sep='\s+', names=columns, skiprows=1)

    # Start a separate process for plotting
    p = multiprocessing.Process(target=plot_data, args=(file1, file2, columns, title))
    p.start()

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python3 script.py <graph_title> <home_dir> <inputfile1> <inputfile2>")
    else:
        visualize_data(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])