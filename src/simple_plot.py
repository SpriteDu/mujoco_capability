import pandas as pd
import matplotlib.pyplot as plt

# Sample data (replace this with the path to your dataset file)



import os

file_path = "build/New_Record.csv"
if os.path.exists(file_path):
    print("The file exists")
else:
    print("The file does not exist")


data = pd.read_csv(file_path)

df = pd.read_csv(file_path, skiprows=1)# Assigning names to the columns

df.columns = ['Source', 'Time', 'Force1', 'Force2', 'Force3', 'Force4', 'Force5', 'Force6']
unique_sources = df['Source'].unique()

# Plotting
fig, axs = plt.subplots(6, 1, figsize=(10, 15))  # 6 rows, 1 column
fig.tight_layout(pad=3.0)

# Loop through each force column
for i in range(1, 7):
    for source in unique_sources:
        filtered_df = df[df['Source'] == source]
        # Convert to NumPy arrays before plotting
        time_values = filtered_df['Time'].to_numpy()
        force_values = filtered_df[f'Force{i}'].to_numpy()

        axs[i-1].plot(time_values, force_values, label=f'{source} - Force {i}')

    axs[i-1].set_title(f'Force {i} vs Time for Multiple Sources')
    axs[i-1].set_xlabel('Time')
    axs[i-1].set_ylabel(f'Force {i}')
    axs[i-1].legend()
plt.show()


