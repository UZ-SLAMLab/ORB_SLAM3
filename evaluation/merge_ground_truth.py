import pandas as pd

sub_datasets = ['MH01', 'MH02', 'MH03', 'MH04', 'MH05']
ground_truth_paths = [f'Datasets/euroc/MachineHall/{i}/mav0/state_groundtruth_estimate0/data.csv' for i in sub_datasets]
ground_truth_columns = ['timestamp', 'p_RS_R_x [m]', 'p_RS_R_y [m]', 'p_RS_R_z [m]', 'q_RS_w []', 'q_RS_x []', 'q_RS_y []', 'q_RS_z []', 'v_RS_R_x [m s^-1]', 'v_RS_R_y [m s^-1]', 'v_RS_R_z [m s^-1]', 'b_w_RS_S_x [rad s^-1]', 'b_w_RS_S_y [rad s^-1]', 'b_w_RS_S_z [rad s^-1]', 'b_a_RS_S_x [m s^-2]', 'b_a_RS_S_y [m s^-2]', 'b_a_RS_S_z [m s^-2]']
file_ground_truth_df_list = []

for file in ground_truth_paths:
    file_ground_truth_df = pd.read_csv(file)
    file_ground_truth_df_list.append(file_ground_truth_df)

df = pd.concat(file_ground_truth_df_list)
output_dir = 'Datasets/euroc/MachineHall/MH01_to_MH05/mav0/state_groundtruth_estimate0/data.csv'
df.to_csv(output_dir, index=False)