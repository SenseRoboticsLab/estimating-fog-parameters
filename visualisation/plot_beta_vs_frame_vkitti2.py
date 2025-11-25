import os
import argparse

import pandas as pd
import matplotlib.pyplot as plt


class Vkitti2:
    data_header_names_ours = ['F', 'KF', 'beta_gray', 'atmos_gray', 'no_mps_gray', 'no_obs_gray', 'no_iters_gray']
    data_header_names_lis = ['F', 'KF', 'beta_gray', 'atmos_gray', 'no_valid_betas_gray', 'no_total_betas_gray']

    def __init__(self, result_file_full_path):
        self.result_file_full_path = result_file_full_path
        self.sequence_name = self.result_file_full_path.split(os.sep)[-3]
        self.is_ours = 'ours' in self.result_file_full_path.lower()

        # read and populate the data
        if self.is_ours:
            self.result_df_raw = pd.read_csv(self.result_file_full_path, sep=" ", names=Vkitti2.data_header_names_ours)
        else:
            self.result_df_raw = pd.read_csv(self.result_file_full_path, sep=" ", names=Vkitti2.data_header_names_lis)

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
        description="Plot beta vs frame for foggy VKITTI2 sequences and save as a PDF file"
    )
    parser.add_argument(
        "--result_path", type=str, required=True, default="../results",
        help="Path to the results directory"
    )
    parser.add_argument(
        "--sequence", type=str, required=True, default="Scene01_fog_40_0.7",
        help="Sequence name (e.g., Scene01_fog_40_0.7)"
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
    lis = Vkitti2(os.path.join(args.result_path, 'OthersLi-AModeMax-PreservePositiveBetaFalse.txt'))
    lis_mod = Vkitti2(os.path.join(args.result_path, 'OthersLi-AModeMedian-PreservePositiveBetaTrue.txt'))
    ours = Vkitti2(os.path.join(args.result_path, 'Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight.txt'))

    # plotting the beta (the grayscale by default) vs frame
    plt.figure(figsize=(6, 2.2))
    lis.plot_beta_vs_frame()
    lis_mod.plot_beta_vs_frame()
    ours.plot_beta_vs_frame()

    plt.legend(["Li's",
                "Li's modified",
                "Ours"],
                ncol=3,
                loc='upper center',
                fontsize=8)
    plt.xlabel('Frame')
    plt.ylabel('Estimated beta')
    plt.title('Estimated beta vs frame - VKITTI2 '+args.sequence)
    plt.ylim(-0.005, 0.15)
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(args.output_path, 'beta_vs_frame_VKITTI2_'+args.sequence+'.pdf'))
    print('Saved plot to', os.path.join(args.output_path, 'beta_vs_frame_'+args.sequence+'.pdf'))
    plt.close()

if __name__ == '__main__':
    main()
