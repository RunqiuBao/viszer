import open3d
import numpy
from typing import List
from scipy.spatial.transform import Rotation as R

import argparse
import os


def LoadTraj(format: str, pathToTraj: str, Tbias: numpy.ndarray=None):
    trajPoses = []
    if format == 'tum':
        with open(pathToTraj, 'r') as trajFile:
            trajListStr = trajFile.readlines()
        for line in trajListStr:
            if line[-1] == "\n":
                line = line[:-1]
            sLine = line.split(" ")
            if len(sLine) == 1:
                sLine = line.split(",")
            try:
                r = R.from_quat([float(sLine[-1]), float(sLine[-4]), float(sLine[-3]), float(sLine[-2])])
            except:
                import inspect; from IPython import embed; print('in {}!'.format(inspect.currentframe().f_code.co_name)); embed()
            rmatrix = r.as_matrix()
            onePose = numpy.eye(4)
            onePose[:3, :3] = rmatrix
            onePose[:3, 3] = numpy.array([
                float(sLine[1]),
                float(sLine[2]),
                float(sLine[3])
            ])
            if Tbias is not None:
                onePose = numpy.matmul(Tbias, onePose)
            trajPoses.append(onePose)
    else:
        raise NotImplementedError

    return trajPoses


def DrawTraj(format: str, listPathToTraj: List[str], listScaleFactors: List[int]):
    listPcd = []
    for pathToTraj, scaleFactor in zip(listPathToTraj, listScaleFactors):
        trajPoses = LoadTraj(format, pathToTraj)
        positions = numpy.array([pose[:3, 3] for pose in trajPoses]) * scaleFactor
        colors = numpy.random.rand(1, 3)  # Random RGB colors
        colors = numpy.tile(colors, (len(trajPoses), 1))
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(positions)
        pcd.colors = open3d.utility.Vector3dVector(colors)
        listPcd.append(pcd)

    open3d.visualization.draw_geometries(listPcd)


def main():
    parser = argparse.ArgumentParser(
        description="show a trajectory.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Examples: \n" +
                '%s --trajfile .../mydataset/aaa.txt \n' % os.path.basename(__file__)
        )
    )

    parser.add_argument('--trajfile', '-i', nargs='+', action='store', type=str,
                        help='Path to the trajectory file.')
    parser.add_argument('--scale', '-s', nargs='+', action='store', type=float,
                        help='scale factors for each trajfile.')
    parser.add_argument('--trajformat', '-m', action='store', type=str, default="tum",
                        help='Format of the traj. file. Default is %(default)s.')

    args, remaining = parser.parse_known_args()

    DrawTraj(args.trajformat, args.trajfile, args.scale)


if __name__ == "__main__":
    main()
