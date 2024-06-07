# Terrain Analysis ext

## launchfile description

### terrain_analysis_ext.launch

This launchfile is used to run the terrain analysis node.

### terrain_analysis_ext_offline.launch + save_terrain_analysis_ext.launch + pub_analysis_result.launch

These launchfiles are used to run the terrain analysis node in offline mode. After tuning the parameters and get a good result, you can save the result pcd file by running the save_terrain_analysis_ext.launch. 

When runing the system, you can publish the result pcd file by running the pub_analysis_result.launch.

### pathNorm.launch

compute the norm of the path, used for slope analysis.

## param description

scanVoxelSize: the size of the voxel grid used to downsample the input point cloud.

decayTime: the time (in seconds) it takes for a voxel to decay.

noDecayDis: the distance (in meters) from the vehicle where voxels do not decay.

clearingDis: the distance (in meters) from the vehicle where voxels are cleared.

clearingCloud: a boolean flag indicating whether or not to clear the voxel grid.

useSorting: a boolean flag indicating whether or not to sort the point cloud before processing.

quantileZ: the quantile value used to filter out points with extreme Z values.

vehicleHeight: the height (in meters) of the vehicle.

voxelPointUpdateThre: the number of points required to update a voxel.

voxelTimeUpdateThre: the time (in seconds) required to update a voxel.

lowerBoundZ: the lower bound (in meters) of the Z range to consider for terrain.

upperBoundZ: the upper bound (in meters) of the Z range to consider for terrain.

disRatioZ: the ratio of the distance from the vehicle to the Z range to consider for terrain.

checkTerrainConn: a boolean flag indicating whether or not to check for terrain connectivity.

terrainUnderVehicle: the height (in meters) of the terrain directly under the vehicle.

terrainConnThre: the threshold value used to determine terrain connectivity.

ceilingFilteringThre: the threshold value used to filter out points above the terrain.

localTerrainMapRadius: the radius (in meters) of the local terrain map.