import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from pathlib import Path
import pandas as pd
from scipy.spatial import distance_matrix

import copy
import numpy as np
import open3d as o3d

class DockDetection():
    def __init__(self, 
                # set reference
                reference: np.array=None,
                # filter
                z_min: float=-0.3,
                z_max: float=0.3,
                min_reflectivity: int=250,
                # DBSCAN parameters
                eps: int=0.1,
                min_samples: int=9,
                # reflectivity cluster size check
                cluster_distance: float=0.75,
                cluster_distance_threshold: float=0.05,
                cluster_size: tuple=(0.175, 0.175), # (width, height)
                cluster_size_threshold: tuple=(0.02, 0.02), # (width, height)
                # filter
                filter_radius: float=0.5,
                filter_z_offset: float=0.07,
                # 2d match
                position_offset: float=0.02,
                angle_offset: float=0.5,
                max_interations: int=10,
                # 3d match
                voxel_size: float=0.02,
                similarity_threshold: float=5.0,
                max_interations_3d: int=2,
                reference_3d_step: float=0.01,
                reference_3d_height: float=0.15
                ):
        
        # filter
        self.z_min = z_min
        self.z_max = z_max
        self.min_reflectivity = min_reflectivity

        # DBSCAN parameters
        self.eps = eps
        self.min_samples = min_samples

        # reflectivity cluster size check
        self.cluster_distance = cluster_distance
        self.cluster_distance_threshold = cluster_distance_threshold
        self.cluster_size = cluster_size
        self.cluster_size_threshold = cluster_size_threshold

        # filter
        self.filter_radius = filter_radius
        self.filter_z_offset = filter_z_offset

        # 2d match
        self.position_offset = position_offset
        self.angle_offset = angle_offset
        self.max_interations = max_interations

        # 3d match
        self.voxel_size = voxel_size
        self.similarity_threshold = similarity_threshold
        self.max_interations_3d = max_interations_3d
        self.reference_3d_step = reference_3d_step
        self.reference_3d_height = reference_3d_height

        self.data_reflectivity = None

        # set reference
        if reference is not None:
            self.set_reference_2d(reference)
        else:
            self.reference_2d = reference

        if reference is not None:
            self.set_reference_3d(reference)
        else:
            self.reference_3d = reference


    def crop_zaxis(self, data, **kwargs):
        """
        Crop point cloud by z_min and z_max.

        Args:
            data (np.array or pd.Dataframe): 
            if np.array: array with shape [points, [x, y, z, **]]
            if pd.Dataframe: dataframe with columns [X, Y, Z, Reflectivity]

        Returns:
            np.array or pd.Datafrade: filtered pointcloud
        """

        z_min = kwargs.get("z_min", self.z_min)
        z_max = kwargs.get("z_max", self.z_max)

        if isinstance(data, pd.DataFrame):
            return data.loc[(data['Z']>=z_min) & (data['Z']<=z_max)]

        elif isinstance(data, np.ndarray):
            # np shape = [points, [x, y, z, reflectivity]]
            return data[(data[:,2]>=z_min) & (data[:,2]<=z_max)]
        

    # ---- CLUSTERS DETECTION ----
    
    def get_reflectivity_cluster(self, data, **kwargs):
        """
        Filtra uma nuvem de pontos a partir da refletividade e clusteriza os dados

        Parameters:
            data: Nuvem de pontos
            min_reflectivity: mínima reflectividade para filtras os dados
            eps: distância máxima entre pontos do mesmo cluster
            min_samples: número de pontos próximos necessários para pertencimento ao cluster

        Returns:
            data: nuvem de pontos filtrada com coluna referente ao cluster
        """
        
        min_reflectivity = kwargs.get("min_reflectivity", self.min_reflectivity)
        eps = kwargs.get("eps", self.eps)
        min_samples = kwargs.get("min_samples", self.min_samples)

        if isinstance(data, pd.DataFrame):
            # Filter high reflectivity
            filtered_data = data.loc[data['Reflectivity'] >= min_reflectivity].copy()
            
            # Check filtered_data length
            if len(filtered_data) == 0:
                info = 'error: no reflectivity points'
                return np.array([]), info
            
            # Clusterize
            clustering = DBSCAN(eps=eps, min_samples=min_samples)
            clustering.fit(filtered_data[['X','Y']])

            # Filter clusters
            filtered_data.loc[:,"cluster"] = clustering.labels_
            filtered_data = filtered_data.loc[filtered_data["cluster"] != -1]
            
            info = 'get_reflectivity_cluster ok'
            return filtered_data, info
        
        elif isinstance(data, np.ndarray):
            # Filter high reflectivity
            reflectivity_mask = data[:,-1] >= min_reflectivity
            filtered_data = data[reflectivity_mask]
            filtered_data = filtered_data[:,:3]
            
            # Check filtered_data length
            if len(filtered_data) == 0:
                info = 'error: no reflectivity points'
                return np.array([]), info
            
            # Clusterize
            clustering = DBSCAN(eps=eps, min_samples=min_samples)
            clustering.fit(filtered_data[:,0:2])

            # Filter clusters
            clusters_labels = clustering.labels_
            clustered_data = np.hstack([filtered_data, clusters_labels.reshape(-1,1)])
            valid_cluster_mask = clusters_labels != -1
            clustered_data[valid_cluster_mask]

            # Split clusters
            clustered_data_sorted = clustered_data[clustered_data[:,-1].argsort()]
            _, split_array = np.unique(clustered_data_sorted[:,-1], return_index=True)
            clustered_data = np.split(clustered_data_sorted[:,:3], split_array[1:])

            info = 'get_reflectivity_cluster ok'
            return clustered_data, info
    
    def get_dock_pose(self, clustered_data, **kwargs):
        """
        Calculate dock pose by clusters point cloud

        Args:
            data (np.ndarray): pointclouds with shape [cluster, row, [x, y]]

        Returns:
            _type_: _description_
        """
        cluster_distance = kwargs.get("cluster_distance", self.cluster_distance)
        cluster_distance = np.array(cluster_distance)
        cluster_distance_threshold = kwargs.get("cluster_distance_threshold", 
                                                self.cluster_distance_threshold)
        cluster_distance_threshold = np.array(cluster_distance_threshold)
        
        cluster_size = kwargs.get("cluster_size", self.cluster_size)
        cluster_size = np.array(cluster_size)
        cluster_size_threshold = kwargs.get("cluster_size_threshold", 
                                            self.cluster_size_threshold)
        cluster_size_threshold = np.array(cluster_size_threshold)
        
        # Calculate clusters width and height
        data_clusters_max = np.array([np.max(cluster, axis=0) for cluster in clustered_data])
        data_clusters_min = np.array([np.min(cluster, axis=0) for cluster in clustered_data])
        
        data_clusters_width = np.sqrt(np.power(data_clusters_max[:,0] - data_clusters_min[:,0], 2) + 
                                      np.power(data_clusters_max[:,1] - data_clusters_min[:,1], 2))
        
        data_clusters_height = data_clusters_max[:,2] - data_clusters_min[:,2]
                
        # Consolidate width and height in transposed array
        data_clusters_size = np.array([data_clusters_width, data_clusters_height]).T
        
        threshold_max = cluster_size + cluster_size_threshold
        threshold_min = cluster_size - cluster_size_threshold

        # filter clusters by size
        clusters_filtered = np.array([
            np.all(data_clusters_size <= threshold_max, axis=1) &
            np.all(data_clusters_size >= threshold_min, axis=1)
        ])

        clusters_filtered = np.squeeze(clusters_filtered, axis=0)
        
        true_clusters = np.where(clusters_filtered==True)[0]
        
        if len(true_clusters) < 2:
            info = 'error: clusters size'
            # info = f'error: clusters size {data_clusters_size}'
            # print(f'ERRO: Clusters identificados não coincidem com o tamanho esperado. Esperado {cluster_size}. Encontrados: {data_clusters_size}')
            return np.array([]), np.array([]), info

        # Filter clustered data and calculate centers
        clusters_centers = [np.mean(x[0], axis=0) for x in zip(clustered_data, clusters_filtered) if x[1] == True]
        clusters_centers = np.array(clusters_centers)
        
        # Calculate clusters distance
        clusters_centers_distances = distance_matrix(clusters_centers[:,0:3],
                                                     clusters_centers[:,0:3])
        
        threshold_max = cluster_distance + cluster_distance_threshold
        threshold_min = cluster_distance - cluster_distance_threshold
        
        # check distances between clusters
        distance_check = ((clusters_centers_distances >= threshold_min.reshape(1,-1)) &
                          (clusters_centers_distances <= threshold_max.reshape(1,-1)))
        
        clusters_combination = np.array(np.where(np.triu(distance_check))).T

        if len(clusters_combination) ==0:
            info = 'error: clusters distance'
            # print(f'ERRO: Clusters identificados não coincidem com a distância esperada. Esperado {cluster_distance}. Encontrados: \n {np.tril(clusters_centers_distances)}')
            return np.array([]), np.array([]), info
        
        # Set dock centers
        docks_centers = np.mean(clusters_centers[clusters_combination,:], axis=1)

        # Set dock angles
        docks_segments = clusters_centers[clusters_combination, :2]
        docks_normals = self.find_normals(docks_segments)
        
        normals_points_distance = np.sqrt(np.power(docks_normals[:,:,0], 2) +
                                          np.power(docks_normals[:,:,1], 2))
        
        normals_points_sort = normals_points_distance.argsort(axis=1)
                
        docks_angles = []
        for dock_normal, normal_point_sort in zip(docks_normals, normals_points_sort):
            
            docks_angles.append(
                np.arctan2(np.diff(dock_normal[normal_point_sort,1]), 
                           np.diff(dock_normal[normal_point_sort,0]))
            )

        docks_angles = np.array(docks_angles)
        
        info = 'get_dock_pose ok'
        return docks_centers, docks_angles, info

    def find_normals(self, segments):
        """_summary_

        Args:
            segments (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        segments = np.array(segments).reshape(-1, 4)
        
        midpoints = (segments[:, :2] + segments[:, 2:]) / 2
        dx = segments[:, 2] - segments[:, 0]
        dy = segments[:, 3] - segments[:, 1]

        norm_endpoint1 = midpoints + np.stack([-dy / 2, dx / 2], axis=1)
        norm_endpoint2 = midpoints + np.stack([dy / 2, -dx / 2], axis=1)

        return np.hstack((norm_endpoint1, norm_endpoint2)).reshape(-1, 2, 2)

    def filter_dock_points(self, data_to_filter, dock_centers, **kwargs):
        """
        Args:
            data_to_filter (np.ndarray): Point cloud with shape [[x,y,z], [x,y,z], ...]
            dock_centers (np.ndarray): Position of dock center with shape [center1 [x,y,z], center2 [x,y,z]]
            radius (float, optional): Radius to filter points. Defaults to 0.5.
        """

        filter_radius = kwargs.get("filter_radius", self.filter_radius)
        filter_z_offset = kwargs.get("filter_z_offset", self.filter_z_offset)

        data = np.copy(data_to_filter[:,0:3])
        
        # Add new dimension and dock index to data
        data = np.expand_dims(data, axis=0)
        data = data[np.zeros(len(dock_centers), dtype=int)]

        dock_index = np.array(range(data.shape[0]))
        dock_index = np.expand_dims(dock_index, axis=1)
        dock_index = np.expand_dims(dock_index, axis=1)
        
        data_len_mask = np.zeros(data.shape[1], dtype=int)
        dock_index = dock_index[:,data_len_mask,:]

        data = np.concatenate((data, dock_index), axis=2)

        # Filter data by radius
        data_mask = np.sqrt((data[:,:,0] - dock_centers[:,[0]])**2 +
                            (data[:,:,1] - dock_centers[:,[1]])**2) <= filter_radius
        data_filtered = data[data_mask]
        
        # Split filtered data by docks index
        _, split_array = np.unique(data_filtered[:,-1], return_index=True)
        
        if len(split_array) > 1:
            data_splited = np.split(data_filtered[:,:-1], split_array[1:])
        else:
            data_splited = [data_filtered[:,:-1]]

        # Filter z axis
        for idx, dock_center in enumerate(dock_centers):

            threshold_max = dock_center[2] + filter_z_offset
            threshold_min = dock_center[2] - filter_z_offset
            mask = ((data_splited[idx][:,2] >= threshold_min) &
                    (data_splited[idx][:,2] <= threshold_max))
            data_splited[idx] = data_splited[idx][mask]

        return data_splited

    
    # ---- 2D REFERENCE MATCH ----
    
    def set_reference_2d(self, reference, 
                         center: np.ndarray=np.array([0,0]), 
                         angle: float=0):
        """
        Set 2d reference for detection

        Parameters:
            reference (array): array of points
        """
        # Create array of segments
        reference = [[reference[i], reference[i + 1]] for i in range(len(reference) - 1)]

        self.reference_2d = np.array(reference)
        self.reference_2d_center = center
        self.reference_2d_angle = angle

    def rotate_points_2D(self, points, angle_radians, center):
        """
        Rotate a 2D array of points by a specific angle around a specified center.
        
        Parameters:
            points: A 2D array of points [[[x1, y1], [x2, y2]], [[x3, y3], [x4, y4]], ...].
            angle_degrees: The angle in degrees to rotate the points.
            center: A tuple (cx, cy) representing the center of rotation.
        
        Return: A 2D array of the rotated points.
        """
        # Convert the angle from degrees to radians
        # angle_radians = np.radians(angle_degrees)
        
        # Compute the cosine and sine of the angle
        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)
        
        # Construct the rotation matrix
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
                
        # Flatten the array to a 2D array of points
        reshaped_points = points.reshape(-1, 2)
        
        # Translate points to origin (center of rotation)
        cx, cy = center
        translated_points = reshaped_points - np.array([cx, cy])
        
        # Apply the rotation matrix to each point using matrix multiplication
        rotated_translated_points = translated_points @ rotation_matrix.T
        
        # Translate points back to the original center
        rotated_points = rotated_translated_points + np.array([cx, cy])
        
        # Reshape the rotated points back to the original shape of the input array
        rotated_points = rotated_points.reshape(points.shape)

        return rotated_points

    def rotate_reference_2d(self, angle_radians, inplace=True):

        reference_2d = self.rotate_points_2D(points=self.reference_2d,
                                                  angle_radians=angle_radians,
                                                  center=self.reference_2d_center)
        reference_2d_angle = self.reference_2d_angle +angle_radians
        
        if inplace:
            self.reference_2d = reference_2d
            self.reference_2d_angle = reference_2d_angle
        
        return reference_2d, reference_2d_angle
        

    def translate_reference_2d(self, translation, inplace=True):
        """
        Translate an array of points by a given translation vector.

        Parameters:
            points: An array of (x, y) coordinates of the points to translate.
            translation: The (x, y) translation vector.

        Returns:
            The new array of (x, y) coordinates of the translated points.
        """
                
        # Translate points
        reference_2d = self.reference_2d + np.resize(translation, (1,1,2))
        reference_2d_center = self.reference_2d_center + np.array(translation)

        if inplace:
            self.reference_2d = reference_2d
            self.reference_2d_center = reference_2d_center

        return reference_2d, reference_2d_center

    def perpendicular_distances_to_segments(self, points, segments):
        """
        Calculate the minimum perpendicular distances from multiple points to multiple line segments.

        :param points: A list of tuples [(x0, y0), (x1, y1), ...] representing the points.
        :param segments: A list of tuples [((x1, y1), (x2, y2)), ...] representing the line segments.
        :return: A NumPy array of minimum perpendicular distances from the points to the segments.
        """
        points = np.array(points)
        segments = np.array(segments)

        x0, y0 = points[:, 0], points[:, 1]
        x1, y1 = segments[:, 0, 0], segments[:, 0, 1]
        x2, y2 = segments[:, 1, 0], segments[:, 1, 1]

        # Vector AB
        ABx, ABy = x2 - x1, y2 - y1
        # Vector AP
        APx = x0[:, None] - x1
        APy = y0[:, None] - y1

        # Squared length of AB
        AB_len_squared = ABx ** 2 + ABy ** 2

        # Avoid division by zero by setting zero-length segments to a very small number
        AB_len_squared = np.where(AB_len_squared == 0, np.finfo(float).eps, 
                                  AB_len_squared)

        # Projection of AP onto AB
        t = (APx * ABx + APy * ABy) / AB_len_squared
        t = np.clip(t, 0, 1)

        # Closest point on the segment
        closest_x = x1 + t * ABx
        closest_y = y1 + t * ABy

        # Vector from P to the closest point
        closest_Px = x0[:, None] - closest_x
        closest_Py = y0[:, None] - closest_y

        # Distances from points to the closest points on the segments
        distances = np.sqrt(closest_Px ** 2 + closest_Py ** 2)

        # Minimum distances for each point
        min_distances = np.min(distances, axis=1)

        return min_distances

    def measure_reference_error_2d(self, data_points):

        distances = self.perpendicular_distances_to_segments(data_points, 
                                                             self.reference_2d)
       
        return np.mean(distances)
    
    def optimize_reference_position_2d(self, data_points, **kwargs):
        """
        Optimize the reference position applying offsets to actual position

        Args:
            position_offset (float): Position offset to estar interact and measure distances. Defaults to 0.01
            angle_offset (float): Angle offset to start interact and measure distances. Defaults to 0.5.
        """
        
        position_offset = kwargs.get("position_offset", self.position_offset)
        angle_offset = kwargs.get("angle_offset", self.angle_offset)

        # Set positions array
        positions = np.array([
            [0,0],
            [0, position_offset],
            [0, -position_offset],
            [position_offset, 0],
            [position_offset, position_offset],
            [position_offset, -position_offset],
            [-position_offset, 0],
            [-position_offset, position_offset],
            [-position_offset, -position_offset],
        ])
        # Set angles array
        angles = np.array([
            0, 
            angle_offset, 
            angle_offset/2,
            -angle_offset, 
            -angle_offset/2
        ])
        
        angles = np.radians(angles)
               
        # interact over positions and angles
        counter = 0
        interations = 0
        while counter < self.max_interations:
            error_updated = False

            min_pos = [0,0]
            min_ang = 0
            min_error = self.measure_reference_error_2d(data_points)

            for pos in positions:
                for ang in angles:
                    
                    # Move reference to new position
                    self.translate_reference_2d(pos)
                    self.rotate_reference_2d(ang)
                    # Measure error
                    error_int = self.measure_reference_error_2d(data_points)
                    # Return reference to origin
                    self.translate_reference_2d(-pos)
                    self.rotate_reference_2d(-ang)

                    # compare with previous error
                    if np.round(error_int, 4) < np.round(min_error, 4):
                        min_pos = pos
                        min_ang = ang
                        min_error = np.copy(error_int)
                        error_updated = True

            # Move referente to position with minnor error
            self.translate_reference_2d(min_pos)
            self.rotate_reference_2d(min_ang)

            if not error_updated:
                counter +=1
                positions = positions/2
                angles = angles/2
            else:
                counter = 0
            
            interations+=1

        return min_error, interations

    def optimize_docks_position_2d(self, docks_points, docks_center,
                                    dosks_angle, **kwargs):

        position_offset = kwargs.get("position_offset", self.position_offset)
        angle_offset = kwargs.get("angle_offset", self.angle_offset)

        centers = np.empty((0,3))
        angles = np.array([])
        errors = np.array([])
        interations = np.array([])

        initial_reference_center = self.reference_2d_center
        initial_reference_angle = self.reference_2d_angle

        for points, dock_center, dock_angle in zip(docks_points,
                                                docks_center,
                                                dosks_angle):

            # Desloc reference to dock pose
            dst_center = dock_center[0:2] - self.reference_2d_center
            dst_angle = dock_angle[0] - self.reference_2d_angle
            self.translate_reference_2d(dst_center)
            self.rotate_reference_2d(dst_angle)
            
            # Optimize reference pose
            error, interation = self.optimize_reference_position_2d(
                                        points,
                                        position_offset=position_offset,
                                        angle_offset=angle_offset
                                        )
            
            # Register reference final pose
            final_center = self.reference_2d_center - initial_reference_center
            final_angle = self.reference_2d_angle - initial_reference_angle

            # Add z coordinate from original center
            final_center = np.append(final_center, dock_center[-1])

            # Desloc referente to initial pose
            dst_center = initial_reference_center - self.reference_2d_center
            dst_angle = initial_reference_angle - self.reference_2d_angle
            self.translate_reference_2d(dst_center)
            self.rotate_reference_2d(dst_angle)

            centers = np.append(centers, [final_center], axis=0)
            angles = np.append(angles, final_angle)
            errors = np.append(errors, error)
            interations = np.append(interations, interation)

        return centers, angles, errors, interations


    # ---- 3D REFERENCE MATCH ----

    def set_reference_3d(self, points, **kwargs):
        """_summary_
        Set 3d reference for detection

        Parameters:
            reference (array): array of points
        """
        reference_3d_height = kwargs.get("reference_3d_height", self.reference_3d_height)
        reference_3d_step = kwargs.get("reference_3d_step", self.reference_3d_step)

        
        points = np.array(points)
        z_values = np.arange(-reference_3d_height/2, reference_3d_height/2+reference_3d_step, reference_3d_step)

        all_points = []

        for i in range(len(points) - 1):
            
            segment_start = points[i]
            segment_end = points[i + 1]
            
            distance = np.linalg.norm(segment_start - segment_end)
            qtd = int(distance / reference_3d_step) + 1
            segment_points = np.linspace(segment_start, segment_end, qtd)

            points_expanded = np.repeat(segment_points, len(z_values), axis=0)
            z_expanded = np.tile(z_values, len(segment_points)).reshape(-1, 1)
            
            segment_points_3d  = np.hstack((points_expanded, z_expanded))
            all_points.append(segment_points_3d)
        reference_pcd = np.vstack(all_points)
        reference_pcd = np.unique(reference_pcd, axis=0)

        # Add track points
        new_points = np.concatenate([reference_pcd, np.array([np.append(points[0], 0),
                                                              np.append(points[-1], 0)])])
        self.reference_3d = o3d.geometry.PointCloud()
        self.reference_3d.points = o3d.utility.Vector3dVector(np.asarray(new_points))

    def transform_points_3d(self, data_points, theta_x, theta_y, theta_z, translate_x, translate_y, translate_z):

        rotated_data_points = copy.deepcopy(data_points)

        theta_x = np.deg2rad(theta_x)
        theta_y = np.deg2rad(theta_y)
        theta_z = np.deg2rad(theta_z)
        translate = [translate_x, translate_y, translate_z]
        
        rx = np.array([[1, 0, 0],
                [0, np.cos(theta_x), -np.sin(theta_x)],
                [0, np.sin(theta_x), np.cos(theta_x)]])

        ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                [0, 1, 0],
                [-np.sin(theta_y), 0, np.cos(theta_y)]])

        rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                [np.sin(theta_z), np.cos(theta_z), 0],
                [0, 0, 1]])

        rxyz = np.dot(rx, np.dot(ry,rz))

        M = np.identity(4)
        M[:3, :3] = rxyz
        M[:3, 3] = translate

        rotated_data_points.transform(M)

        return rotated_data_points

    def transform_reference_3d(self, theta_x, theta_y, theta_z, translate_x, translate_y, translate_z, inplace=True):

        reference_3d = copy.deepcopy(self.reference_3d)

        reference_3d = self.transform_points_3d(data_points=reference_3d,
                                                theta_x=theta_x,
                                                theta_y=theta_y,
                                                theta_z=theta_z,
                                                translate_x=translate_x,
                                                translate_y=translate_y,
                                                translate_z=translate_z)
        if inplace:
            self.reference_3d = reference_3d
        
        return reference_3d

    def compute_fpfh(self, pcd_down, **kwargs):

        voxel_size = kwargs.get("voxel_size", self.voxel_size)

        radius_normal = voxel_size * 2
        # print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_fpfh

    def register_ICP(self, source_xyz, target_xyz, **kwargs):

        similarity_threshold = kwargs.get("similarity_threshold", self.similarity_threshold)

        source_down = copy.deepcopy(source_xyz)#.uniform_down_sample(every_k_points=100)
        target_down = copy.deepcopy(target_xyz)#.uniform_down_sample(every_k_points=100)

        # print("Global registration...")
        global_register = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, self.compute_fpfh(source_down), self.compute_fpfh(target_down),
            o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=similarity_threshold)
        )

        source_down = source_down.transform(global_register.transformation)

        # print("Refine (local) registration...")
        reg_p2p = o3d.pipelines.registration.registration_icp(
                source_down, target_down,
                max_correspondence_distance=similarity_threshold,
                init=np.eye(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )

        aligned_source = (
            copy.deepcopy(source_xyz)
            .transform(global_register.transformation)
            .transform(reg_p2p.transformation)
        )
        
        # return aligned_source, global_register.transformation, reg_p2p.transformation
        return aligned_source
    
    def similarity_chamfer(self, cloud1, cloud2):
        distx = np.square(np.asarray(cloud1.compute_point_cloud_distance(cloud2))).sum()
        disty = np.square(np.asarray(cloud2.compute_point_cloud_distance(cloud1))).sum()
        return distx+disty, distx, disty
    
    def optimize_reference_position_3d(self, points):
        source_xyz = self.reference_3d
        target_xyz = o3d.geometry.PointCloud()
        target_xyz.points = o3d.utility.Vector3dVector(points)
        
        self.aligned_source = source_xyz #

        min_error = self.similarity_chamfer(source_xyz, target_xyz)[1]
        counter = 0

        while counter < self.max_interations_3d:
            try:
                aligned_source = self.register_ICP(source_xyz, target_xyz)
                error = self.similarity_chamfer(aligned_source, target_xyz)[1]
                # print(f'error {counter}: {error}, actual: {min_error}')
                if error < min_error:
                    # print(f'error < min_error')
                    self.aligned_source = aligned_source
                    min_error = error
            except:
                pass
            counter +=1

        tracked_points = np.asarray(self.aligned_source.points)[-2:]

        # Set reference angles
        references_normals = self.find_normals(tracked_points[:, :2])
        
        normals_points_distance = np.sqrt(np.power(references_normals[:,:,0], 2) +
                                          np.power(references_normals[:,:,1], 2))
        
        normals_points_sort = normals_points_distance.argsort(axis=1)

        references_angles = []
        for reference_normal, normal_point_sort in zip(references_normals, normals_points_sort):
            
            references_angles.append(
                np.arctan2(np.diff(reference_normal[normal_point_sort,1]), 
                           np.diff(reference_normal[normal_point_sort,0]))
            )
        
        angle = references_angles[0]
 
        center = tracked_points[:, :].mean(axis=0)

        return center, angle, min_error
    
    def optimize_docks_position_3d(self, docks_points, docks_center,
                                    docks_angle, **kwargs):
        
        centers_3d = np.empty((0,3))
        angles_3d = np.array([])
        errors_3d = np.array([])

        initial_reference = self.reference_3d

        for points, dock_center, dock_angle in zip(docks_points,
                                                docks_center,
                                                docks_angle):

            # Move reference to dock pose
            self.transform_reference_3d(theta_x=0,
                                        theta_y=0,
                                        theta_z=np.rad2deg(dock_angle[0]),
                                        translate_x=dock_center[0],
                                        translate_y=dock_center[1],
                                        translate_z=dock_center[2],
                                        )
            
            # Optimize reference pose
            center, angle, error = self.optimize_reference_position_3d(points)

            centers_3d = np.append(centers_3d, [center], axis=0)
            angles_3d = np.append(angles_3d, angle)
            errors_3d = np.append(errors_3d, error)

            # Move reference to initial pose
            self.reference_3d = initial_reference

        return centers_3d, angles_3d, errors_3d

    
                
