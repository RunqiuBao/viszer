import open3d
import json
import numpy
import copy


def Draw3DPlane(cornersOnePlane: numpy.ndarray, isDraw: bool):
    """
    Args:
        cornersOnePlane: 4x3 shape, each row is a corner of the plane.
        isDraw: bool, whether to draw the open3d geometry or not. If True, then it opens open3d viewer and draw the geometry immediately.
    """
    trianglesTwoFaces = [numpy.array([[0, 1, 2], [0, 2, 3]], dtype='int'), numpy.array([[0, 2, 1], [0, 3, 2]], dtype='int')]  # Note: make the mesh visible from both normal directions.
    geometries = []
    for trianglesOneFace in trianglesTwoFaces:
        planeMesh = open3d.geometry.TriangleMesh()
        planeMesh.vertices = open3d.utility.Vector3dVector(cornersOnePlane)
        planeMesh.triangles = open3d.utility.Vector3iVector(trianglesOneFace)
        planeMesh.compute_vertex_normals()
        planeMesh.paint_uniform_color([0.1, 0.1, 0.5])
        geometries.append(planeMesh)
    if isDraw:
        open3d.visualization.draw_geometries(geometries)
        return None
    else:
        return geometries
    

def Draw3DPoints(points: numpy.ndarray, isDraw: bool):
    """
    Args:
        points: Nx3 shape, each row is a point.
    """
    pointCloud = open3d.geometry.PointCloud()
    pointCloud.points = open3d.utility.Vector3dVector(points)
    pointCloud.colors = open3d.utility.Vector3dVector(numpy.tile(numpy.array([0.5, 0.1, 0.1]), (points.shape[0], 1)))
    if isDraw:
        open3d.visualization.draw_geometries([pointCloud])
        return None
    else:
        return [pointCloud]


def ConvertContainerPlanesToCorners(planes: numpy.ndarray, containerExtents: numpy.ndarray, zBias = None):
    """
    Args:
        planes: 6x4 shape, the plane equations.
        containerExtents: 3d vector, the extents (half in size) of the container.
    """
    corners = numpy.zeros((6, 4, 3), dtype='float')
    signsForCorners = numpy.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
    for ii in range(6):
        plane = planes[ii, :]
        vn = plane[0:3]
        vref = abs(planes[ii - 1][-1]) * planes[ii - 1][:3] - abs(plane[-1]) * plane[:3] # Assume plane normal always pointing to outside of the container
        vref = vref / numpy.linalg.norm(vref)
        vref1 = numpy.cross(vn, vref)
        vref1 = vref1 / numpy.linalg.norm(vref1)
        vref2 = numpy.cross(vn, vref1)
        vref2 = vref2 / numpy.linalg.norm(vref2)
        planeCenter = abs(plane[-1]) * plane[:3]
        # print("vn", vn)
        # print("vref", vref)
        # print("vref1", vref1)
        # print("vref2", vref2)
        # print("planeCenter", planeCenter)
        for jj in range(4):
            corners[ii, jj, :] = planeCenter + signsForCorners[jj, 0] * containerExtents[ii % 3 - 2] * vref1 + signsForCorners[jj, 1] * containerExtents[ii % 3 - 1] * vref2
    if zBias is not None:
        corners[:, :, 2] += zBias
    return corners

    
def DrawPlanesWithPoints(planes: numpy.ndarray, points: numpy.ndarray, containerExtents: numpy.ndarray):
    """
    Args:
        planes: 6x4 shape, each row is a plane. The last column is the top plane of container.
        points: Nx3 shape, each row is a point. It is in the same frame as planes.
        containerExtents: 3d vector, the extents (half in size) of the container.
    """
    zBias = None
    if planes[2, -1] != planes[5, -1]:
        zBias = containerExtents[-1] + planes[2, -1]  # Note: if bottom plane of inner is above container origin, zBias should be positive.
        planesCentered = copy.deepcopy(planes)
        planesCentered[2, -1] = (planes[2, -1] + planes[5, -1]) / 2
        planesCentered[5, -1] = (planes[2, -1] + planes[5, -1]) / 2
    else:
        planesCentered = planes
    cornersInPlanes = ConvertContainerPlanesToCorners(planesCentered, containerExtents, zBias)

    # # Draw the container planes
    geometries = []
    for ii in range(5):  # Note: skip the top plane
        geometries.extend(Draw3DPlane(cornersInPlanes[ii], False))

    # Draw the points
    geometries.extend(Draw3DPoints(points, False))

    open3d.visualization.draw_geometries(geometries)


if __name__ == "__main__":
    import json
    import numpy
    import open3d
    from container_with_target import DrawPlanesWithPoints, Draw3DPoints, Draw3DPlane, ConvertContainerPlanesToCorners
    with open("/tmp/youcandelete/validateByContainer_0.json", 'r') as file:                                                         
        aa = json.load(file)
    
    DrawPlanesWithPoints(numpy.array(aa['containerPlanes']), numpy.array(aa['verticesInContainer']), numpy.array(aa['containerExtents']))
