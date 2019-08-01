import open3d as o3d
import matplotlib.pyplot as plt

if __name__ == "__main__":
    print("Read SUN dataset")
    color_raw = o3d.io.read_image(
        "image/img_0063.jpg")
    depth_raw = o3d.io.read_image(
        "depth/img_0063_abs.png")
    rgbd_image = o3d.geometry.create_rgbd_image_from_sun_format(
        color_raw, depth_raw)
    print(rgbd_image)
    plt.subplot(1, 2, 1)
    plt.title('SUN grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('SUN depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()
    pcd = o3d.geometry.create_point_cloud_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])