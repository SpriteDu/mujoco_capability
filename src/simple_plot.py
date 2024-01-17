import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

file_path = "../build/New_Record.csv"

# Check if the file name is provided as a command-line argument
if len(sys.argv) > 1:
    file_name = sys.argv[1]
    file_path = f"../build/{file_name}"

    if os.path.exists(file_path):
        print("The file exists")
        # Add your data processing code here
    else:
        print("The file does not exist")

data = pd.read_csv(file_path)

df = pd.read_csv(file_path, skiprows=2)# Assigning names to the columns

df.columns = ['Source', 'Time', 'Force1', 'Force2', 'Force3'] #,'Force4', 'Force5', 'Force6'
unique_sources = df['Source'].unique()

# Plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 15))  # 6 rows, 1 column
fig.tight_layout(pad=3.0)

# Loop through each force column
for i in range(1, 4):
    for source in unique_sources:
        filtered_df = df[df['Source'] == source]
        # Convert to NumPy arrays before plotting
        time_values = filtered_df['Time'].to_numpy()
        force_values = filtered_df[f'Force{i}'].to_numpy()

        axs[i-1].plot(time_values, force_values, label=f'{source} - Force {i}')

    axs[i-1].set_title(f'Force {i} - Time for Multiple Sources')
    axs[i-1].set_xlabel('Time(s)')
    axs[i-1].set_ylabel(f'Force {i}(N)')
    axs[i-1].legend()
plt.show()


