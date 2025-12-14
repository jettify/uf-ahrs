# /// script
# dependencies = [
#   "pandas",
#   "scipy",
#   "ipdb",
#   "pyarrow",
# ]
# ///

import pandas as pd
import pyarrow as pa
import scipy
from pathlib import Path


def mat_to_df(mat):
    data = {
        'imu_gyr_x': mat['imu_gyr'][:, 0],
        'imu_gyr_y': mat['imu_gyr'][:, 1],
        'imu_gyr_z': mat['imu_gyr'][:, 2],
        #
        'imu_acc_x': mat['imu_acc'][:, 0],
        'imu_acc_y': mat['imu_acc'][:, 1],
        'imu_acc_z': mat['imu_acc'][:, 2],
        #
        'imu_mag_x': mat['imu_mag'][:, 0],
        'imu_mag_y': mat['imu_mag'][:, 1],
        'imu_mag_z': mat['imu_mag'][:, 2],
        #
        'opt_quat_w': mat['opt_quat'][:, 0],
        'opt_quat_x': mat['opt_quat'][:, 1],
        'opt_quat_y': mat['opt_quat'][:, 2],
        'opt_quat_z': mat['opt_quat'][:, 3],
        #
        'opt_pos_x': mat['opt_pos'][:, 0],
        'opt_pos_y': mat['opt_pos'][:, 1],
        'opt_pos_z': mat['opt_pos'][:, 2],
        #
        'movement': mat['movement'][:, 0].astype(float),
    }
    sampling_rate = mat['sampling_rate'][0][0]
    df = pd.DataFrame(data)
    df['sampling_rate'] = sampling_rate
    return df


def main():
    locations = sorted(Path(r'../../broad/data_mat/').glob('*.mat'))
    for filename in locations:
        mat = scipy.io.loadmat(filename)
        df = mat_to_df(mat)
        print(mat['imu_acc'][0])
        # df.to_parquet(f"data/{filename.name}.parquet")
        df.to_csv(f"data/{filename.name}.csv", index=False)


if __name__ == "__main__":
    main()
