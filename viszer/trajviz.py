import open3d
import numpy
from scipy.spatial.transform import Rotation as R


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


def DrawTraj(format: str, pathToTraj: str):
    trajPoses = LoadTraj(format, pathToTraj)
    positions = numpy.array([pose[:3, 3] for pose in trajPoses])
    colors = numpy.zeros((len(trajPoses), 3))  # Random RGB colors
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(positions)
    pcd.colors = open3d.utility.Vector3dVector(colors)

    open3d.visualization.draw_geometries([pcd])
