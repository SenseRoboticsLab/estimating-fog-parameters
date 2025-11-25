import os
import argparse

import pandas as pd
import matplotlib.pyplot as plt


class Sdirf:
    data_header_names_ours = ['F', 'KF',
                              'beta_B', 'atmos_B', 'no_mps_B', 'no_obs_B', 'no_iters_B',
                              'beta_G', 'atmos_G', 'no_mps_G', 'no_obs_G', 'no_iters_G',
                              'beta_R', 'atmos_R', 'no_mps_R', 'no_obs_R', 'no_iters_R',
                              'beta_gray', 'atmos_gray', 'no_mps_gray', 'no_obs_gray', 'no_iters_gray']
    data_header_names_lis = ['F', 'KF',
                             'beta_B', 'atmos_B', 'no_valid_betas_B', 'no_total_betas_B',
                             'beta_G', 'atmos_G', 'no_valid_betas_G', 'no_total_betas_G',
                             'beta_R', 'atmos_R', 'no_valid_betas_R', 'no_total_betas_R',
                             'beta_gray', 'atmos_gray', 'no_valid_betas_gray', 'no_total_betas_gray']

    def __init__(self, result_file_full_path):
        self.result_file_full_path = result_file_full_path
        self.sequence_name = self.result_file_full_path.split(os.sep)[-2]
        self.is_ours = 'ours' in self.result_file_full_path.lower()

        # read and populate the data
        if self.is_ours:
            self.result_df_raw = pd.read_csv(self.result_file_full_path, sep=" ", names=Sdirf.data_header_names_ours)
        else:
            self.result_df_raw = pd.read_csv(self.result_file_full_path, sep=" ", names=Sdirf.data_header_names_lis)

        self.result_df = None
        self.populate_data()

    def populate_data(self):
        """
        Populate results of frames in between frames that have estimation results, using the most recent estimation
        For example, if we have results for frames 5, 10, 17, then we fill in frames 6, 7, 8, 9 with the results of frame 5,
        and frames 11, 12, 13, 14, 15, 16 with the results of frame 10.
        """
        # Create full range of frames from min to max
        full_frame_range = pd.DataFrame({
            "F": range(self.result_df_raw["F"].min(), self.result_df_raw["F"].max() + 1)
        })

        # Merge full frame range with raw results
        self.result_df = pd.merge(full_frame_range, self.result_df_raw, on="F", how="left")

        # Forward-fill missing values (use most recent estimation result)
        self.result_df = self.result_df.ffill()

    def plot_beta_vs_frame(self, channel='gray'):
        assert channel in ['B', 'G', 'R', 'gray']
        colours = {'B': 'blue', 'G': 'green', 'R': 'red'}
        if channel == 'gray':
            plt.plot(self.result_df['F'], self.result_df['beta_' + channel], alpha=.5)
        else:
            plt.plot(self.result_df['F'], self.result_df['beta_' + channel], color=colours[channel], alpha=.5)


def main():
    parser = argparse.ArgumentParser(
        description="Plot beta vs frame for SDIRF sequences and save as a PDF file"
    )
    parser.add_argument(
        "--result_path", type=str, required=True, default="../results",
        help="Path to the results directory"
    )
    parser.add_argument(
        "--sequence", type=str, required=True, default="P11_FogThin",
        help="Sequence name (e.g., P11_FogThin)"
    )
    parser.add_argument(
        "--output_path", type=str, required=True, default="./",
        help="Path to save the plot as a PDF file"
    )

    args = parser.parse_args()

    print("Result path:", args.result_path)
    print("Sequence:", args.sequence)
    print("Output path:", args.output_path)

    # load results
    lis = Sdirf(os.path.join(args.result_path, 'OthersLi-AModeMax-PreservePositiveBetaFalse.txt'))
    lis_mod = Sdirf(os.path.join(args.result_path, 'OthersLi-AModeMedian-PreservePositiveBetaTrue.txt'))
    ours = Sdirf(os.path.join(args.result_path, 'Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight.txt'))
    ours_wo = Sdirf(os.path.join(args.result_path, 'Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight-WoGc.txt'))

    # plotting the beta (the grayscale by default) vs frame
    plt.figure(figsize=(6, 2.2))
    lis.plot_beta_vs_frame()
    lis_mod.plot_beta_vs_frame()
    ours.plot_beta_vs_frame()
    # ours_wo.plot_beta_vs_frame()  # add a legend entry below for ours_wo if needed

    plt.legend(["Li's",
                "Li's modified",
                "Ours"],
                ncol=3,
                loc='upper center',
                fontsize=8)
    plt.xlabel('Frame')
    plt.ylabel('Estimated beta')
    plt.title('Estimated beta vs frame - SDIRF '+args.sequence)
    plt.ylim(-0.005, 0.025)
    plt.yticks([0, 0.005, 0.01, 0.015, 0.02])
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(args.output_path, 'beta_vs_frame_SDIRF_'+args.sequence+'.pdf'))
    print('Saved plot to', os.path.join(args.output_path, 'beta_vs_frame_'+args.sequence+'.pdf'))
    plt.close()

if __name__ == '__main__':
    main()
