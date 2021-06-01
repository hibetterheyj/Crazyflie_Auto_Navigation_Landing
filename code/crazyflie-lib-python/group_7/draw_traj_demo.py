"""
x-y trajectory visualization with region annotation

python draw_traj_demo.py --logname overall-20210530_1930 -n cf_demo --zone_anno
"""

import argparse
from cf_utilis import plot_xy_traj

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='analyze x-y trajectory')

    # load from logs
    parser.add_argument('--log_folder', default='./logs/', type=str,
                        help="logs folder")
    parser.add_argument('--logname', required=True, default=None, type=str,
                        help="log filename")

    # save to image
    parser.add_argument('--img_folder', default='./pics/', type=str,
                        help="image folder")
    parser.add_argument('-n', '--name', default='cf_traj_test', type=str,
                        help="output image filename")

    # display
    parser.add_argument('--zone_anno', dest='za', action='store_true',
                        help="Annotate different zones")
    parser.set_defaults(za=False)

    args = parser.parse_args()

    x_filenpath = args.log_folder+args.logname+'_x.csv'
    y_filenpath = args.log_folder+args.logname+'_y.csv'
    img_folder = './pics/'

    # plotting
    plot_xy_traj(x_filenpath, y_filenpath, args.name,
                        args.img_folder, args.za)
