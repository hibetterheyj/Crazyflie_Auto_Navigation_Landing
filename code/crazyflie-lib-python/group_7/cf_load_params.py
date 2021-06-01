import argparse

def set_task_options(parser):
    ## connection
    parser.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7', type=str,
                    help="uri for connection")

    ## initialization
    ## start points
    parser.add_argument('-x', default=0.6, type=float,
                help="estimated x for the drone")
    parser.add_argument('-y', default=0.6, type=float,
                help="estimated y for the drone")
    parser.add_argument('--yaw', default=90, type=float,
                help="estimated heading angle for the drone")

    ## box detection and landing
    parser.add_argument('--avg_window', default=10, type=int,
                    help="window size for averaging past z data")
    parser.add_argument('--search_offset', default=0.05, type=float,
                    help = "searching offset away from the boundaries of the region")
    parser.add_argument('--box_threshold', default=0.025, type=float,
                    help = "height threshold for box edge detection")

def set_other_options(parser):
    # Fly with_visualization
    parser.add_argument('-v', '--visualization', dest='with_visualization', action='store_true')
    parser.set_defaults(with_visualization=False)
    # Log path name
    parser.add_argument('--log_path', default='/logs/', type=str,
                    help="path to save log files")
